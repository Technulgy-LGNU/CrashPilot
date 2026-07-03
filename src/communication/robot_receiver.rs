use crate::communication::RobotHeartbeat;
use crate::communication::{EventShare, Events};
use crate::config;
use core_dump::proto::RobotCp;
use prost::Message;
use std::sync::atomic::Ordering;
use std::time::Instant;
use tokio::sync::RwLockWriteGuard;

pub fn robot_receiver(
  cfg: &config::Config,
  heartbeats: RobotHeartbeat,
  tx: EventShare,
  wrap: fn(RobotCp, RwLockWriteGuard<Events>),
  process_start: Instant,
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

    loop {
      let mut buf = [0u8; 1024];
      match socket.recv_from(&mut buf).await {
        Ok((size, addr)) => {
          if let Ok(msg) = RobotCp::decode(&buf[..size]) {
            let robot_id = msg.robot_id;
            if robots
              .get(&robot_id)
              .is_some_and(|robot| robot.ip == addr.ip())
            {
              let now_ms = process_start.elapsed().as_millis() as u64;
              heartbeats[robot_id as usize].store(now_ms, Ordering::Relaxed);

              let lock = tx.write().await;
              wrap(msg, lock);
            }
          }
        }
        Err(e) => {
          eprintln!("Error receiving robot message{:?}", e);
        }
      }
    }
  });
}
