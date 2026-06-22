use std::time::{SystemTime, UNIX_EPOCH};
use crate::communication::{EventShare, Events};
use crate::config;
use core_dump::proto::RobotCp;
use prost::Message;
use tokio::sync::MutexGuard;

pub async fn robot_receiver(
  cfg: &config::Config,
  tx: EventShare,
  wrap: fn(RobotCp, MutexGuard<Events>),
) {
  let addr = format!(
    "{}:{}",
    cfg.server.robot_socket_host, cfg.server.robot_receive_port
  );
  let robots = cfg.robots.clone();

  tokio::spawn(async move {
    let socket = match tokio::net::UdpSocket::bind(addr.clone()).await {
      Ok(s) => s,
      Err(e) => {
        panic!("Couldn't bind socket: {}", e);
      }
    };

    println!("Robot receiver listening on {}", addr);

    let mut buf = [0u8; 65535];
    loop {
      match socket.recv_from(&mut buf).await {
        Ok((size, addr)) => {
          if robots.iter().find(|x| addr.ip() == x.1.ip).is_some() {
            if let Ok(msg) = RobotCp::decode(&buf[..size]) {
              let lock = tx.lock().await;
              let now_ts = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs_f64();
              println!("Delay from Crashpilot -> Robot -> Crashpilot: {:?}", now_ts - msg.timestamp);
              wrap(msg, lock);
            } else {
              eprintln!("Failed to decode message from robot: {:?}", addr);
            }
          } else {
            eprintln!("IP not found for robot: {:?}", addr);
            continue;
          }
        }
        Err(e) => {
          eprintln!("Error receiving robot message{:?}", e);
        }
      }
    }
  });
}
