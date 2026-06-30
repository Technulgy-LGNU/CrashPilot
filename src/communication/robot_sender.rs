use crate::RobotData;
use crate::communication::RobotHeartbeat;
#[cfg(feature = "loki")]
pub(crate) use crate::communication::loki::LokiPublisher;
use crate::config::Config;
use anyhow::{Error, anyhow};
use prost::Message;
use std::collections::HashMap;
use std::net::{SocketAddr, SocketAddrV4};
use std::sync::atomic::Ordering;
use std::time::Instant;
use tokio::net::UdpSocket;
use tokio::time::Duration;

pub struct NetworkSender<'a> {
  pub socket: &'a UdpSocket,
  pub data: &'a HashMap<u32, RobotData>,
  pub heartbeats: &'a RobotHeartbeat,
  #[cfg(feature = "loki")]
  pub loki: Option<&'a LokiPublisher>,
  pub cfg: &'a Config,
  pub process_start: Instant,
}

#[derive(Debug, Default)]
pub struct SendReport {
  /// Number of robots we successfully sent a UDP datagram to.
  pub sent: usize,
  /// Per-robot failures. Sending is best-effort; one failure does not stop the loop.
  pub failed: Vec<RobotSendFailure>,
}

#[derive(Debug)]
pub struct RobotSendFailure {
  pub _robot_id: u32,
  pub _error: Error,
}

impl SendReport {
  fn push_failure(&mut self, robot_id: u32, error: Error) {
    self.failed.push(RobotSendFailure {
      _robot_id: robot_id,
      _error: error,
    });
  }
}

pub trait RobotSender {
  /// Sends the current `CpRobot` message to all robots found in `self.data`.
  ///
  /// The function is **best-effort**: it will continue sending even if some robots fail.
  /// Returned `SendReport` describes successes and failures.
  fn send_to_all_robots(&self) -> SendReport;
}

impl RobotSender for NetworkSender<'_> {
  fn send_to_all_robots(&self) -> SendReport {
    let now_ms = self.process_start.elapsed().as_millis() as u64;
    let mut report = SendReport::default();
    let mut buf = Vec::new();

    for (&robot_id, robot_data) in self.data.iter() {
      if now_ms.saturating_sub(self.heartbeats[robot_id as usize].load(Ordering::Relaxed)) < 100 {
        // Keep the buffer re-used but always reset before encoding.
        buf.clear();
        buf.reserve(robot_data.msg.encoded_len());

        if robot_data.msg.robot_id != robot_id {
          report.push_failure(
            robot_id,
            anyhow!(
              "robot_id mismatch: map key is {robot_id} but message.robot_id is {}",
              robot_data.msg.robot_id
            ),
          );
          // Still attempt to send using the map key, since that's what we have configured.
        }

        if let Err(e) = robot_data.msg.encode(&mut buf) {
          report.push_failure(
            robot_id,
            Error::new(e).context("failed to encode CpRobot protobuf"),
          );
          continue;
        }

        if buf.is_empty() {
          report.push_failure(robot_id, anyhow!("encoded CpRobot message is empty"));
          continue;
        }

        let robot_cfg = match self.cfg.robots.get(&robot_id) {
          Some(c) => c,
          None => {
            report.push_failure(
              robot_id,
              anyhow!("no robot configuration found for id {robot_id}"),
            );
            continue;
          }
        };

        let addr = SocketAddr::V4(SocketAddrV4::new(robot_cfg.ip, self.cfg.server.robots_port));

        let start = Instant::now();
        // Wrap send_to with timeout to prevent hanging on unreachable robots
        match self.socket.try_send_to(&buf, addr) {
          Ok(bytes_sent) if bytes_sent == buf.len() => {
            report.sent += 1;
            #[cfg(feature = "loki")]
            if let Some(loki) = &self.loki {
              loki.publish_robot_message(robot_data.msg.clone());
            }
          }
          Ok(bytes_sent) => {
            report.push_failure(
              robot_id,
              anyhow!("partial UDP send: sent {bytes_sent} of {} bytes", buf.len()),
            );
          }
          Err(e) => {
            report.push_failure(
              robot_id,
              Error::new(e).context(format!("failed to send UDP datagram to {addr}")),
            );
          }
        }
        let elapsed = start.elapsed();
        if elapsed > Duration::from_millis(50) {
          println!("Elapsed: {:.1?}", elapsed);
          println!("Send to robot: {:?} with delay: {:?}", robot_id, elapsed);
        }
      } else {
        report.push_failure(
          robot_id,
          anyhow!("Not reachable: Heartbeat exceeded maximum"),
        )
      }
    }

    report
  }
}

// #[cfg(test)]
// mod tests {
//   use super::*;
//   use crate::config::{Config, RobotConfig};
//   use core_dump::proto::{CpBall, CpCommand, CpRobot, CpVector2};
//   use std::net::Ipv4Addr;
//   use std::time::Duration;
//   use tokio::time::timeout;
//
//   fn sample_robot(robot_id: u32) -> RobotData {
//     RobotData {
//       msg: CpRobot {
//         robot_id,
//         timestamp: 0f64,
//         packet_id: 1,
//         ball: CpBall {
//           pos: CpVector2 { x: 0, y: 0 },
//           vel: None,
//         },
//         robots_yellow: vec![],
//         robots_blue: vec![],
//         cmd: CpCommand {
//           state: 0,
//           task: 0,
//           pos: None,
//           speed: None,
//           orientation: None,
//           kick_orient: None,
//           kick_speed: None,
//           enemy_id: None,
//         },
//         infos: Default::default(),
//       },
//       feedback: Default::default(),
//     }
//   }
//
//   #[tokio::test]
//   async fn sends_udp_datagram_to_configured_robot() {
//     let receiver = UdpSocket::bind("127.0.0.1:0").await.expect("bind receiver");
//     let recv_addr = receiver.local_addr().expect("receiver local addr");
//
//     let sender_socket = UdpSocket::bind("127.0.0.1:0").await.expect("bind sender");
//     let data = HashMap::from([(1u32, sample_robot(1))]);
//     let sender: NetworkSender = NetworkSender {
//       socket: &sender_socket,
//       data: &data,
//       #[cfg(feature = "loki")]
//       loki: None,
//     };
//
//     let mut cfg = Config::default();
//     cfg.server.robots_port = recv_addr.port();
//     cfg.robots.insert(
//       1,
//       RobotConfig {
//         ip: Ipv4Addr::new(127, 0, 0, 1),
//         substitution_pos: Default::default(),
//       },
//     );
//
//     let report = sender.send_to_all_robots(&cfg).await;
//     assert_eq!(report.sent, 1);
//     assert!(
//       report.failed.is_empty(),
//       "unexpected failures: {:#?}",
//       report.failed
//     );
//
//     let mut buf = [0u8; 2048];
//     let (n, _from) = timeout(Duration::from_millis(200), receiver.recv_from(&mut buf))
//       .await
//       .expect("timed out waiting for udp datagram")
//       .expect("recv_from failed");
//
//     let decoded = CpRobot::decode(&buf[..n]).expect("decode CpRobot");
//     assert_eq!(decoded.robot_id, 1);
//     assert_eq!(decoded.packet_id, 1);
//   }
//
//   #[tokio::test]
//   async fn reports_missing_robot_config_without_panicking() {
//     let sender_socket = UdpSocket::bind("127.0.0.1:0").await.expect("bind sender");
//     let data = HashMap::from([(123u32, sample_robot(123))]);
//     let sender: NetworkSender = NetworkSender {
//       socket: &sender_socket,
//       data: &data,
//       #[cfg(feature = "loki")]
//       loki: None,
//     };
//
//     let mut cfg = Config::default();
//     cfg.robots.clear();
//
//     let report = sender.send_to_all_robots(&cfg).await;
//     assert_eq!(report.sent, 0);
//     assert_eq!(report.failed.len(), 1);
//     assert_eq!(report.failed[0].robot_id, 123);
//   }
// }
