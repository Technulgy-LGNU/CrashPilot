use crate::communication::EventShare;
use crate::config;
use crate::proto::{InterfaceWrapperCp, Referee, RobotCp, SslWrapperPacket, TrackerWrapperPacket};
use prost::Message;
use tokio::sync::MutexGuard;

pub async fn robot_receiver(
  cfg: &config::Config,
  tx: EventShare,
  wrap: fn(
    RobotCp,
    MutexGuard<(
      Option<SslWrapperPacket>,
      Option<TrackerWrapperPacket>,
      Option<InterfaceWrapperCp>,
      Option<Referee>,
      Option<RobotCp>,
    )>,
  ),
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
              println!("Received message from robot: {:?}", msg);

              let lock = tx.lock().await;
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

/*
I want to publish the robot stats to Prometheus, can you implement me a clean solution, which does not block any of my other code and associates the data in Prometheus with the robot_id?
Use something like hyper1 as a http server, because its minimal and has basically zero overhead
 */
