use std::collections::HashMap;
use std::string::String;
use prost::Message;
use tokio::net::{ UdpSocket, UnixStream };
use crate::config::Config;
use crate::proto::CpRobot;

pub struct NetworkSender<'a> {
  pub(crate) socket: &'a UdpSocket,
  pub(crate) data: HashMap<u32, CpRobot>,
}

pub struct SocketSender {
  stream: UnixStream,
  data: HashMap<String, CpRobot>,
}

pub trait RobotSender {
  async fn send_to_all_robots(&mut self, cfg: &Config);
}

impl RobotSender for NetworkSender<'_> {
  async fn send_to_all_robots(&mut self, cfg: &Config) {
    let mut buf = Vec::new(); // New buffer
    for robot in self.data.keys() {
      let robot_data = self.data.get(robot).unwrap();

      if robot == &2 {
        println!("==========================");
        println!("{:?}", robot_data);
        println!("==========================");
      }

      match robot_data.encode(&mut buf) {
        Ok(_) => {
          if buf.is_empty() {
            println!("Buffer is empty for robot: {}", robot_data.robot_id);
            return;
          }
        },
        Err(e) => {
          eprintln!("Failed to encode protobuf for robot {}: {}", robot, e);
          continue; // Move to the next robot if encoding fails
        }
      };

      let robot_addr = format!("{}:{}", cfg.robots.get(robot).unwrap().ip, cfg.server.robots_port);
      match self.socket.send_to(&buf, robot_addr.clone()).await {
        Ok(_) => (),
        Err(e) => eprintln!("Failed to send data to robot {}: {} || Message size: {}", robot, e, buf.len()),
      };
      buf.clear(); // Clear the buffer for the next robot
    }
  }
}

impl RobotSender for SocketSender {
  async fn send_to_all_robots(&mut self, cfg: &Config) {
    todo!()
  }
}
