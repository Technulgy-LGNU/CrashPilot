use crate::communication::RobotHeartbeat;
use crate::communication::{EventShare, Events};
use crate::config;
use core_dump::proto::RobotCp;
use prost::Message;
use std::sync::atomic::Ordering;
use tokio::sync::RwLockWriteGuard;
use tokio::time::Instant;

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
          if let Some(robot_idx) = robots.iter().find(|x| addr.ip() == x.1.ip) {
            {
              let now_ms = process_start.elapsed().as_millis() as u64;

              heartbeats[*robot_idx.0 as usize].store(now_ms, Ordering::Relaxed);
            }

            if let Ok(msg) = RobotCp::decode(&buf[..size]) {
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
