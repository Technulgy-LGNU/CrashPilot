use crate::config::Config;
use crate::proto::CpRobot;
use anyhow::{anyhow, Error};
use prost::Message;
use std::collections::HashMap;
use std::net::{SocketAddr, SocketAddrV4};
use tokio::net::UdpSocket;

pub struct NetworkSender<'a> {
  pub(crate) socket: &'a UdpSocket,
  pub(crate) data: &'a HashMap<u32, CpRobot>,
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
  pub robot_id: u32,
  pub error: Error,
}

impl SendReport {
  fn push_failure(&mut self, robot_id: u32, error: Error) {
    self.failed.push(RobotSendFailure { robot_id, error });
  }
}

pub trait RobotSender {
  /// Sends the current `CpRobot` message to all robots found in `self.data`.
  ///
  /// The function is **best-effort**: it will continue sending even if some robots fail.
  /// Returned `SendReport` describes successes and failures.
  async fn send_to_all_robots(&self, cfg: &Config) -> SendReport;
}

impl RobotSender for NetworkSender<'_> {
  async fn send_to_all_robots(&self, cfg: &Config) -> SendReport {
    let mut report = SendReport::default();
    let mut buf = Vec::new();

    for (&robot_id, robot_data) in self.data.iter() {
      // Keep the buffer re-used but always reset before encoding.
      buf.clear();
      buf.reserve(robot_data.encoded_len());

      if robot_data.robot_id != robot_id {
        report.push_failure(
          robot_id,
          anyhow!(
            "robot_id mismatch: map key is {robot_id} but message.robot_id is {}",
            robot_data.robot_id
          ),
        );
        // Still attempt to send using the map key, since that's what we have configured.
      }

      if let Err(e) = robot_data.encode(&mut buf) {
        report.push_failure(robot_id, Error::new(e).context("failed to encode CpRobot protobuf"));
        continue;
      }

      if buf.is_empty() {
        report.push_failure(robot_id, anyhow!("encoded CpRobot message is empty"));
        continue;
      }

      // Print data, for current testing
      // println!("====================");
      // println!("Robot ID: {}", robot_id);
      // println!("Encoded CpRobot ({} bytes): {:02X?}", buf.len(), buf);
      // println!("====================");

      let robot_cfg = match cfg.robots.get(&robot_id) {
        Some(c) => c,
        None => {
          report.push_failure(robot_id, anyhow!("no robot configuration found for id {robot_id}"));
          continue;
        }
      };

      let addr = SocketAddr::V4(SocketAddrV4::new(robot_cfg.ip, cfg.server.robots_port));
      match self.socket.send_to(&buf, addr).await {
        Ok(bytes_sent) if bytes_sent == buf.len() => {
          report.sent += 1;
        }
        Ok(bytes_sent) => {
          report.push_failure(
            robot_id,
            anyhow!(
              "partial UDP send: sent {bytes_sent} of {} bytes",
              buf.len()
            ),
          );
        }
        Err(e) => {
          report.push_failure(
            robot_id,
            Error::new(e).context(format!("failed to send UDP datagram to {addr}")),
          );
        }
      }
    }

    report
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::config::{Config, RobotConfig};
  use crate::proto::{CpBall, CpCommand, CpVector2};
  use prost_types::Timestamp;
  use std::net::Ipv4Addr;
  use std::time::Duration;
  use tokio::time::timeout;

  fn sample_robot(robot_id: u32) -> CpRobot {
    CpRobot {
      robot_id,
      timestamp: Timestamp { seconds: 0, nanos: 0 },
      packet_id: 1,
      ball: CpBall {
        pos: CpVector2 { x: 0, y: 0 },
        vel: None,
      },
      robots_yellow: vec![],
      robots_blue: vec![],
      cmd: CpCommand {
        state: 0,
        task: 0,
        pos: None,
        speed: None,
        orientation: None,
        kick_orient: None,
        kick_speed: None,
      },
    }
  }

  #[tokio::test]
  async fn sends_udp_datagram_to_configured_robot() {
    let receiver = UdpSocket::bind("127.0.0.1:0").await.expect("bind receiver");
    let recv_addr = receiver.local_addr().expect("receiver local addr");

    let sender_socket = UdpSocket::bind("127.0.0.1:0").await.expect("bind sender");
    let data = HashMap::from([(1u32, sample_robot(1))]);
    let sender: NetworkSender = NetworkSender {
      socket: &sender_socket,
      data: &data,
    };

    let mut cfg = Config::default();
    cfg.server.robots_port = recv_addr.port();
    cfg.robots.insert(
      1,
      RobotConfig {
        ip: Ipv4Addr::new(127, 0, 0, 1),
      },
    );

    let report = sender.send_to_all_robots(&cfg).await;
    assert_eq!(report.sent, 1);
    assert!(report.failed.is_empty(), "unexpected failures: {:#?}", report.failed);

    let mut buf = [0u8; 2048];
    let (n, _from) = timeout(Duration::from_millis(200), receiver.recv_from(&mut buf))
      .await
      .expect("timed out waiting for udp datagram")
      .expect("recv_from failed");

    let decoded = CpRobot::decode(&buf[..n]).expect("decode CpRobot");
    assert_eq!(decoded.robot_id, 1);
    assert_eq!(decoded.packet_id, 1);
  }

  #[tokio::test]
  async fn reports_missing_robot_config_without_panicking() {
    let sender_socket = UdpSocket::bind("127.0.0.1:0").await.expect("bind sender");
    let data = HashMap::from([(123u32, sample_robot(123))]);
    let sender: NetworkSender = NetworkSender {
      socket: &sender_socket,
      data: &data,
    };

    let mut cfg = Config::default();
    cfg.robots.clear();

    let report = sender.send_to_all_robots(&cfg).await;
    assert_eq!(report.sent, 0);
    assert_eq!(report.failed.len(), 1);
    assert_eq!(report.failed[0].robot_id, 123);
  }
}

