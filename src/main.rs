use crate::communication::communication_receiver;
use crate::proto::{CpInterfaceWrapper, CpTrackedRobot, InterfaceCommandCp, SslDetectionBall};
use crate::robot_communication::robot_sender::{NetworkSender, RobotSender};
use crate::utils::as_cp_vec2;
use prost::Message;
use prost_types::Timestamp;
use std::collections::HashMap;
use std::io::ErrorKind;
use std::option::Option;
use std::time::SystemTime;
use tokio::time::{interval, Duration, MissedTickBehavior};
use crate::data_handler::ball_data::{convert_ball, VisionBalls};

mod communication;
mod config;
mod interface;
mod proto;
mod robot_communication;
mod ssl_communication;
mod utils;
mod data_handler;

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

  // Sending should not depend on receiving new packets: when vision/GC packets pause,
  // we still want to keep sending the latest known command/state to the robots.
  // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
  let mut tick = interval(Duration::from_millis(8)); // ~120 Hz
  tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

  loop {
    tick.tick().await;

    // Drain the *latest* events from the shared state. We handle all types each tick,
    // not just one, so state stays fresh.
    let (raw, tracked, ws, gc) = {
      let mut lock = rx.lock().await;
      (lock.0.take(), lock.1.take(), lock.2.take(), lock.3.take())
    };

    if let Some(packet) = raw {
      vis_raw = packet;
    }
    if let Some(packet) = tracked {
      vis_tracked = packet;
    }
    if let Some(packet) = ws {
      // Avoid printing every packet: stdout can become a bottleneck and make the
      // program *look* like it stops sending.
      // eprintln!("Received Websocket packet: {:?}", packet);
      for robot_command in packet.robot_commands {
        robots_ws_data.insert(robot_command.robot_id, robot_command.command);
      }
      interface_command = packet.interface_command;
    }
    if let Some(packet) = gc {
      referee = packet;
    }

    // Create data for each robot
    for robot in robots.values_mut() {
      // Basic data
      robot.packet_id = packet_id;
      robot.timestamp = Timestamp::from(SystemTime::now());

      // Tracked frame, if not empty
      // Robot Position Data
      match vis_tracked.tracked_frame.clone() {
        Some(frame) => {
          // Robot
          // Clear robots already in array
          robot.robots_yellow = vec![];
          robot.robots_blue = vec![];
          for robot_tracked in frame.robots {
            let robot_vis: CpTrackedRobot = CpTrackedRobot {
              robot_id: robot_tracked.robot_id.id.unwrap_or_default(),
              pos: as_cp_vec2(robot_tracked.pos),
              orientation: robot_tracked.orientation.to_degrees() as i32,
              vel: Option::from(as_cp_vec2(robot_tracked.vel.unwrap_or_default())),
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

          // Raw or Tracked vision can be used here
          // Tracked vision is superior and will be used by default
          // Ball
          if !interface_command.ball_tracked {
            let vis_raw_balls: Vec<SslDetectionBall> = match vis_raw.detection.clone() {
              Some(frame) => frame.balls,
              None => vec![],
            };
            robot.ball = convert_ball(VisionBalls::Raw(vis_raw_balls), interface_command);
          } else {
            robot.ball = convert_ball(VisionBalls::Tracked(frame.balls), interface_command);
          }
        }
        None => (),
      };

      // Commands
      // Check for the referee command and overwrite cp commands
      // HALT Command, all robots stop
      if referee.command == 0 && interface_command.gc_data {
        robot.cmd = match robots_ws_data.get(&robot.robot_id) {
          Some(cmd) => *cmd,
          None => Default::default(),
        };
        robot.cmd.state = 0;

      // STOP Command, all robots are only allowed to move with a max velocity of 1.5m/s and should avoid the ball with a clearance of 0.5m
      } else if referee.command == 1 && interface_command.gc_data {
        robot.cmd = match robots_ws_data.get(&robot.robot_id) {
          Some(cmd) => *cmd,
          None => Default::default(),
        };
        robot.cmd.state = 1;

      // Send the last command received by the interface
      } else {
        robot.cmd = match robots_ws_data.get(&robot.robot_id) {
          Some(cmd) => *cmd,
          None => Default::default(),
        };
      }
    }

    // Send the data to the robots
    let network_sender: NetworkSender = NetworkSender {
      socket: &robot_socket,
      data: &robots,
    };
    let send_report = network_sender.send_to_all_robots(&config).await;
    if !send_report.failed.is_empty() {
      eprintln!(
        "Robot send: {} ok, {} failed",
        send_report.sent,
        send_report.failed.len()
      );
      for failure in send_report.failed {
        eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
      }
    }

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
