use std::option::Option;
use crate::communication::Event;
use crate::communication::communication_receiver;
use crate::robot_communication::robot_sender::{NetworkSender, RobotSender};
use prost::Message;
use prost_types::Timestamp;
use std::collections::HashMap;
use std::io::ErrorKind;
use std::time::SystemTime;
use crate::proto::{CpBall, CpInterfaceWrapper, CpTrackedRobot, CpVector2, InterfaceCommandCp, SslDetectionBall, TrackedBall, Vector2, Vector3};

mod communication;
mod config;
mod interface;
mod proto;
mod robot_communication;
mod ssl_communication;
mod utils;

#[tokio::main]
async fn main() {
  // Get config
  let config = match config::load_or_create_config("config.toml") {
    Ok(config) => config,
    Err(e) => panic!("{}", e),
  };

  // Receiver for the communication
  let comm = match communication_receiver(&config).await {
    Ok(comm) => comm,
    Err(e) => panic!("{}", e),
  };

  let rx = comm.events;
  let ws_out = comm.ws_out;

  // UDPSocket for robot communication
  let robot_socket = match tokio::net::UdpSocket::bind(
    format!("{}:{}",
            config.server.robot_socket_host,
            config.server.robot_socket_port)
  ).await {
    Ok(socket) => socket,
    Err(e) => match e.kind() {
      ErrorKind::AddrNotAvailable => {
        panic!(
          "Failed to bind UDP socket: Address not available. Please check if the IP address and port are correct and not in use."
        );
      }
      _ => panic!("Failed to bind UDP socket: {}", e),
    },
  };


  // Robots Hashmap
  let mut packet_id = 0;
  let mut robots: HashMap<u32, proto::CpRobot> = HashMap::new();
  for robot in config.robots.iter() {
    robots.insert(
      *robot.0,
      proto::CpRobot {
        robot_id: *robot.0,
        timestamp: Default::default(),
        packet_id,
        ball: Default::default(),
        robots_yellow: vec![],
        robots_blue: vec![],
        cmd: Default::default(),
      },
    );
  }
  // Initialize the hashmap for the websocket data, which will be used to store the last command received for each robot
  let mut robots_ws_data: HashMap<u32, proto::CpCommand> = HashMap::new();
  for robot in config.robots.iter() {
    robots_ws_data.insert(*robot.0, Default::default());
  }


  println!("Starting robots...");
  // Data packets
  let mut vis_raw: proto::SslWrapperPacket = Default::default();
  let mut vis_tracked: proto::TrackerWrapperPacket = Default::default();
  let mut interface_command: InterfaceCommandCp = Default::default();
  let mut referee: proto::Referee = Default::default();

  loop {
    let mut lock = rx.lock().await;

    let event = lock.0.take().map(Event::RawVision)
      .or_else(|| lock.1.take().map(Event::TrackedVision)
        .or_else(|| lock.2.take().map(Event::Websocket)
          .or_else(|| lock.3.take().map(Event::Referee))
        )
      );

    drop(lock);

    let Some(event) = event else {
      continue;
    };



    // Receive all packets and store them in the corresponding variables
    match event {
      Event::RawVision(packet) => {
        vis_raw = packet;
      }
      Event::TrackedVision(packet) => {
        vis_tracked = packet;
      }
      Event::Websocket(packet) => {
        println!("Received Websocket packet: {:?}", packet);
        for robot_command in packet.robot_commands {
          if robots_ws_data.contains_key(&robot_command.robot_id.unwrap()) {
            robots_ws_data.insert(robot_command.robot_id.unwrap(), robot_command.command.unwrap());
          }
        }
        interface_command = packet.interface_command;
      }
      Event::Referee(packet) => {
        referee = packet;
      }
    }

    // Create data for each robot
    for robot in robots.values_mut() {
      // Basic data
      robot.packet_id = packet_id;
      robot.timestamp = Timestamp::from(SystemTime::now());

      // Tracked frame, if not empty
      match vis_tracked.tracked_frame.clone() {
        Some(frame) => {
          // Robot
          // Clear robots already in array
          robot.robots_yellow = vec![];
          robot.robots_blue = vec![];
          for robot_tracked in frame.robots {
            let robot_vis: CpTrackedRobot = CpTrackedRobot {
              robot_id: robot.robot_id,
              pos: as_cp_vec2(robot_tracked.pos),
              orientation: robot_tracked.orientation as i32,
              vel: Option::from(as_cp_vec2(robot_tracked.vel.unwrap())),
            };

            match robot_tracked.robot_id.team {
              // Yellow robots
              Some(1) => {
                // Check if this yellow robot already exists
                if !robot.robots_yellow.iter().any(|robot| robot.robot_id == robot_vis.robot_id) {
                    robot.robots_yellow.push(robot_vis);
                }
              },
              // Blue Robots
              Some(2) => {
                // Check if this blue robot already exists
                if !robot.robots_blue.iter().any(|robot| robot.robot_id == robot_vis.robot_id) {
                  robot.robots_yellow.push(robot_vis);
                }
              },
              _ => (),
            }
          }

          // Ball
          if interface_command.ball_tracked {
            robot.ball = convert_ball(VisionBalls::Tracked(frame.balls), interface_command);
          } else {
            robot.ball = convert_ball(VisionBalls::Raw(vis_raw.detection.clone().unwrap().balls), interface_command);
          }
        }
        None => (),
      };

      // Commands
      // Check for the referee command and overwrite cp commands
      // HALT Command, all robots stop
      if referee.command == 0 && interface_command.gc_data {
        robot.cmd = *robots_ws_data.get(&robot.robot_id).unwrap();
        robot.cmd.state = 0;

      // STOP Command, all robots are only allowed to move with a max velocity of 1.5m/s and should avoid the ball with a clearance of 0.5m
      } else if referee.command == 1 && interface_command.gc_data {
        robot.cmd = *robots_ws_data.get(&robot.robot_id).unwrap();
        robot.cmd.state = 1;

      // Send the last command received by the interface
      } else {
        robot.cmd = *robots_ws_data.get(&robot.robot_id).unwrap();
      }
    }

    // Send the data to the robots
    let mut network_sender: NetworkSender = NetworkSender {
      socket: &robot_socket,
      data: robots.clone(),
    };
    network_sender.send_to_all_robots(&config).await;

    // Broadcast the latest state to all websocket clients (CP -> interface)
    // Note: if no clients are connected, send() returns an error; that's fine.
    let ws_packet = CpInterfaceWrapper {
      vision_raw: Some(vis_raw.clone()),
      vision_tracked: Some(vis_tracked.clone()),
      gc_data: if referee.packet_timestamp != 0 { Some(referee.clone()) } else { None },
      robot_commands: robots.values().cloned().collect(),
    };
    let mut buf = Vec::with_capacity(ws_packet.encoded_len());
    if let Err(e) = ws_packet.encode(&mut buf) {
      eprintln!("Failed to encode websocket packet: {}", e);
    } else {
      ws_out.publish(buf.into()).await;
    }

    // So the next packet has a higher id
    packet_id += 1;
  }
}

fn as_cp_vec2(v2: Vector2) -> CpVector2 {
  CpVector2 {
    x: v2.x as i32,
    y: v2.y as i32,
  }
}

enum VisionBalls {
  Raw(Vec<SslDetectionBall>),
  Tracked(Vec<TrackedBall>)
}
/// Convert a tracked ball into a CpBall
/// Also does only select the ball who is in the designated test area, if test mode is enabled
fn convert_ball(balls: VisionBalls, interface_command: InterfaceCommandCp) -> CpBall {
  // The correct ball, that gets passed on
  let mut correct_ball: TrackedBall = Default::default();
  // Converts all balls to the TrackedBall, so the function works with the default vision
  let mut balls_generic: Vec<TrackedBall> = vec![];

  // Matches the raw vision balls
  match balls {
    VisionBalls::Raw(balls) => {
      for ball in balls {
        balls_generic.push(TrackedBall {
          pos: Vector3 {
            x: ball.x,
            y: ball.y,
            z: 0.0,
          },
          vel: None,
          visibility: None,
        });
      }
    },
    VisionBalls::Tracked(balls) => {
      balls_generic = balls;
    }
  }

  // Test field check, filters for the a specific part of the field
  if interface_command.enable_testfield {
    let mut correct_balls: Vec<&TrackedBall> = vec![];
    // Correct Balls

    // Switch between the test areas
    // We different between the four areas with their omen
    match interface_command.testfield {
      // -x || +y
      0 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x < 0.0 && ball.pos.y > 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // +x || +y
      1 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x > 0.0 && ball.pos.y > 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // +x || -y
      2 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x > 0.0 && ball.pos.y < 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // -x || -y
      3 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x < 0.0 && ball.pos.y < 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      _ => (),
    }
    if !correct_balls.is_empty() {
      correct_ball = *correct_balls[0];
    }
  } else {
    correct_ball = balls_generic[0];
  }
  CpBall {
    pos: CpVector2 {
      x: correct_ball.pos.x as i32,
      y: correct_ball.pos.y as i32,
    },
    vel: Option::from(CpVector2 {
      x: correct_ball.vel.unwrap().x as i32,
      y: correct_ball.vel.unwrap().y as i32,
    }),
  }
}
