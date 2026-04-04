use std::collections::HashMap;
use prost::Message;
use tokio::net::UdpSocket;
use crate::config;
use crate::proto::CpRobot;

mod robot_sender;
mod robot_receiver;

pub async fn send_to_all_robots(socket: &UdpSocket, data: &HashMap<u32, CpRobot>, cfg: &config::Config) {
  let mut buf = vec![0u8; 1024];
  for robot in data.keys() {
    let robot_data = data.get(robot).unwrap();
    robot_data.encode(&mut buf).unwrap();

    let robot_addr = format!("{}:{}", cfg.robots.get(robot).unwrap().ip, 1024);
    match socket.send_to(&buf, robot_addr).await {
      Ok(_) => println!("Sent data to robot {}", robot),
      Err(e) => eprintln!("Failed to send data to robot {}: {}", robot, e),
    };
  }
}
