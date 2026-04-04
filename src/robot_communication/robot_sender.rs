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
    let mut buf = vec![0u8; 1024];
    for robot in self.data.keys() {
      let robot_data = self.data.get(robot).unwrap();
      robot_data.encode(&mut buf).unwrap();

      let robot_addr = format!("{}:{}", cfg.robots.get(robot).unwrap().ip, 1024);
      match self.socket.send_to(&buf, robot_addr).await {
        Ok(_) => (),
        Err(e) => eprintln!("Failed to send data to robot {}: {}", robot, e),
      };
    }
  }
}

impl RobotSender for SocketSender {
  async fn send_to_all_robots(&mut self, cfg: &Config) {
    todo!()
  }
}
