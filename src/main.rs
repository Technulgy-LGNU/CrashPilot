use std::collections::HashMap;
use std::io::ErrorKind;
use std::net::Ipv4Addr;
use crate::communication::communication_receiver;
use crate::communication::Event;

mod ssl_communication;
mod proto;
mod utils;
mod robot_communication;
mod config;
mod interface;
mod communication;

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
  let robot_socket = match tokio::net::UdpSocket::bind("10.0.64.10:8080").await {
    Ok(socket) => socket,
    Err(e) => match e.kind() {
      ErrorKind::AddrNotAvailable => {
        panic!("Failed to bind UDP socket: Address not available. Please check if the IP address and port are correct and not in use.");
      }
      _ => panic!("Failed to bind UDP socket: {}", e),
    },
  };

  // Robots Hashmap
  let mut packet_id =0;
  let mut robots: HashMap<Ipv4Addr, proto::CpRobot> = HashMap::new();
  for robot in config.robots.iter() {
    robots.insert(robot.1.ip, proto::CpRobot {
      robot_id: *robot.0,
      timestamp: Default::default(),
      packet_id,
      ball: None,
      kicked_ball: None,
      robots_yellow: vec![],
      robots_blue: vec![],
      cmd: Default::default(),
    });
  }


  while let Some(event) = rx.recv().await {
    match event {
      Event::Referee(referee) => {
        println!("Referee: {:?}", referee);
      }
      Event::SslWrapper(wrapper) => {
        println!("SslWrapper: {:?}", wrapper);
      }
      Event::Websocket(ws) => {
        println!("Websocket: {:?}", ws);
      }
    }
  }
}
