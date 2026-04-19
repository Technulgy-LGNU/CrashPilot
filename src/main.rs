use std::option::Option;
use crate::communication::Event;
use crate::communication::communication_receiver;
use crate::robot_communication::robot_sender::{NetworkSender, RobotSender};
use prost_types::Timestamp;
use std::collections::HashMap;
use std::io::ErrorKind;
use std::time::SystemTime;
use crate::proto::CpTrackedRobot;

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
  let mut rx = match communication_receiver(&config).await {
    Ok(rx) => rx,
    Err(e) => panic!("{}", e),
  };

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
        kicked_ball: Default::default(),
        robots_yellow: vec![],
        robots_blue: vec![],
        cmd: Default::default(),
      },
    );
  }
  // Initialize the hashmap for the websocket data, which will be used to store the last command received for each robot
  let mut robots_ws_data: HashMap<u32, proto::CpInterface> = HashMap::new();
  for robot in config.robots.iter() {
    robots_ws_data.insert(*robot.0, Default::default());
  }


  println!("Starting robots...");
  // Data packets
  let mut referee: proto::Referee = Default::default();
  let mut ssl_wrapper: proto::TrackerWrapperPacket = Default::default();

  loop {
    let mut lock = rx.lock().await;

    let event = lock.0.take().map(Event::SslWrapper).or_else(|| lock.1.take().map(Event::Websocket));

    drop(lock);

    let Some(event) = event else {
      continue;
    };



    // Receive all packets and store them in the corresponding variables
    match event {
      Event::Referee(packet) => {
        referee = packet;
      }
      Event::SslWrapper(packet) => {
        ssl_wrapper = packet;
      }
      Event::Websocket(mut packet) => {
        println!("Received Websocket packet: {:?}", packet);
        // Check if robot exists in hashmap
        if robots_ws_data.contains_key(&packet.robot_id) {
          packet.command.pos.as_mut().map(|pos| {
            pos.x /= 1000.0;
            pos.y /= 1000.0;
          });

          robots_ws_data.insert(packet.robot_id, packet);
        }
      }
    }

    // Create data for each robot
    for robot in robots.values_mut() {
      // Basic data
      robot.packet_id = packet_id;
      robot.timestamp = Timestamp::from(SystemTime::now());

      // Tracked frame, if not empty
      match ssl_wrapper.tracked_frame.clone() {
        Some(frame) => {
          // Robot
          // Clear robots already in array
          robot.robots_yellow = vec![];
          robot.robots_blue = vec![];
          for robot_tracked in frame.robots {
            // Yellow  Team
            if robot_tracked.robot_id.team == Some(proto::Team::Yellow as i32) {
              let robot_yellow: CpTrackedRobot = CpTrackedRobot {
                robot_id: robot_tracked.robot_id.id.unwrap(),
                pos: robot_tracked.pos,
                orientation: robot_tracked.orientation,
                vel: robot_tracked.vel,
                vel_angular: robot_tracked.vel_angular,
              };
              robot.robots_yellow.push(robot_yellow);

            // Blue Team
            } else if robot_tracked.robot_id.team == Some(proto::Team::Blue as i32) {
              let robot_blue: CpTrackedRobot = CpTrackedRobot {
                robot_id: robot_tracked.robot_id.id.unwrap(),
                pos: robot_tracked.pos,
                orientation: robot_tracked.orientation,
                vel: robot_tracked.vel,
                vel_angular: robot_tracked.vel_angular,
              };
              let mut robot_exists = false;
              for rb in &robot.robots_blue {
                if rb.robot_id == robot_blue.robot_id {
                  robot_exists = true;
                }
              }
              if !robot_exists {
                robot.robots_blue.push(robot_blue);
              }
            }
          }

          // Balls
          if frame.balls.len() != 0 {
            robot.ball.pos = Option::from(frame.balls[0].pos);
            robot.ball.vel = frame.balls[0].vel;
          }

          match frame.kicked_ball {
            Some(kicked_ball) => {
              robot.kicked_ball.pos = kicked_ball.pos;
              robot.kicked_ball.vel = kicked_ball.vel;
              robot.kicked_ball.stop_pos = kicked_ball.stop_pos;
            }
            None => (),
          }
        }
        None => (),
      };

      // Commands
      // Check for the referee command and overwrite cp commands
      // HALT Command, all robots stop
      if referee.command == 0 {
        robot.cmd = robots_ws_data.get(&robot.robot_id).unwrap().command;
        robot.cmd.state = 0;

      // STOP Command, all robots are only allowed to move with a max velocity of 1.5m/s and should avoid the ball with a clearance of 0.5m
      } else if referee.command == 1 {
        robot.cmd = robots_ws_data.get(&robot.robot_id).unwrap().command;
        robot.cmd.state = 1;

      // Send the last command received by the interface
      } else {
        robot.cmd = robots_ws_data.get(&robot.robot_id).unwrap().command;
      }
    }

    // Send the data to the robots
    let mut network_sender: NetworkSender = NetworkSender {
      socket: &robot_socket,
      data: robots.clone(),
    };
    network_sender.send_to_all_robots(&config).await;
    // So the next packet has a higher id
    packet_id += 1;
  }
}
