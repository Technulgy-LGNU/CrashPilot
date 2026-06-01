use crate::config::Config;
use crate::proto::{CpBall, CpCommand, CpRobot, CpTrackedRobot, CpVector2};
use http_body_util::Full;
use hyper::Request;
use hyper::body::Bytes;
use hyper::header::CONTENT_TYPE;
use hyper_util::client::legacy::Client;
use hyper_util::client::legacy::connect::HttpConnector;
use hyper_util::rt::TokioExecutor;
use prost_types::Timestamp;
use serde::Serialize;
use serde_json::json;
use std::collections::BTreeMap;
use std::net::SocketAddrV4;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::mpsc;

const MAX_BATCH_SIZE: usize = 128;
const FLUSH_INTERVAL: Duration = Duration::from_millis(200);
const APP_NAME: &str = "CrashPilot";

#[derive(Clone)]
pub struct LokiPublisher {
  tx: mpsc::Sender<OutgoingRobotLog>,
}

#[derive(Debug, Clone)]
struct OutgoingRobotLog {
  robot: CpRobot,
  ts_ns: u128,
}

#[derive(Debug, Serialize)]
struct LokiPushBody {
  streams: Vec<LokiStream>,
}

#[derive(Debug, Serialize)]
struct LokiStream {
  stream: BTreeMap<String, String>,
  values: Vec<[String; 2]>,
}

impl LokiPublisher {
  pub fn publish_robot_message(&self, robot: CpRobot) {
    let _ = self.tx.try_send(OutgoingRobotLog {
      robot,
      ts_ns: now_unix_nanos(),
    });
  }
}

pub fn spawn_loki_publisher(cfg: &Config) -> LokiPublisher {
  let (tx, rx) = mpsc::channel::<OutgoingRobotLog>(2048);
  let endpoint = loki_endpoint(cfg);

  tokio::spawn(async move {
    let client: Client<HttpConnector, Full<Bytes>> =
      Client::builder(TokioExecutor::new()).build(HttpConnector::new());
    run_loki_publisher(client, endpoint, rx).await;
  });

  LokiPublisher { tx }
}

async fn run_loki_publisher(
  client: Client<HttpConnector, Full<Bytes>>,
  endpoint: String,
  mut rx: mpsc::Receiver<OutgoingRobotLog>,
) {
  let mut pending = Vec::with_capacity(MAX_BATCH_SIZE);

  loop {
    match tokio::time::timeout(FLUSH_INTERVAL, rx.recv()).await {
      Ok(Some(item)) => {
        pending.push(item);
        drain_pending(&mut rx, &mut pending);
        if pending.len() >= MAX_BATCH_SIZE {
          flush_pending(&client, &endpoint, &mut pending).await;
        }
      }
      Ok(None) => {
        if !pending.is_empty() {
          flush_pending(&client, &endpoint, &mut pending).await;
        }
        break;
      }
      Err(_) => {
        if !pending.is_empty() {
          flush_pending(&client, &endpoint, &mut pending).await;
        }
      }
    }
  }
}

fn drain_pending(rx: &mut mpsc::Receiver<OutgoingRobotLog>, pending: &mut Vec<OutgoingRobotLog>) {
  while pending.len() < MAX_BATCH_SIZE {
    match rx.try_recv() {
      Ok(item) => pending.push(item),
      Err(_) => break,
    }
  }
}

async fn flush_pending(
  client: &Client<HttpConnector, Full<Bytes>>,
  endpoint: &str,
  pending: &mut Vec<OutgoingRobotLog>,
) {
  if pending.is_empty() {
    return;
  }

  let body = build_push_body(pending);
  pending.clear();

  let request = match Request::post(endpoint)
    .header(CONTENT_TYPE, "application/json")
    .body(Full::new(Bytes::from(body)))
  {
    Ok(req) => req,
    Err(e) => {
      eprintln!("Failed to build Loki request: {e}");
      return;
    }
  };

  if let Err(e) = client.request(request).await {
    eprintln!("Failed to push Loki logs: {e}");
  }
}

fn build_push_body(entries: &[OutgoingRobotLog]) -> String {
  let mut grouped: BTreeMap<u32, Vec<&OutgoingRobotLog>> = BTreeMap::new();
  for entry in entries {
    grouped.entry(entry.robot.robot_id).or_default().push(entry);
  }

  let streams: Vec<LokiStream> = grouped
    .into_iter()
    .map(|(robot_id, items)| {
      let mut stream = BTreeMap::new();
      stream.insert("app".to_string(), APP_NAME.to_string());
      stream.insert("direction".to_string(), "outbound".to_string());
      stream.insert("robot_id".to_string(), robot_id.to_string());

      let values = items
        .into_iter()
        .map(|item| {
          [
            item.ts_ns.to_string(),
            cp_robot_to_value(&item.robot).to_string(),
          ]
        })
        .collect();

      LokiStream { stream, values }
    })
    .collect();

  serde_json::to_string(&LokiPushBody { streams }).expect("failed to serialize Loki payload")
}

fn cp_robot_to_value(robot: &CpRobot) -> serde_json::Value {
  json!({
    "robot_id": robot.robot_id,
    "timestamp": timestamp_to_value(&robot.timestamp),
    "packet_id": robot.packet_id,
    "ball": cp_ball_to_value(&robot.ball),
    "robots_yellow": robot.robots_yellow.iter().map(cp_tracked_robot_to_value).collect::<Vec<_>>(),
    "robots_blue": robot.robots_blue.iter().map(cp_tracked_robot_to_value).collect::<Vec<_>>(),
    "cmd": cp_command_to_value(&robot.cmd),
  })
}

fn timestamp_to_value(timestamp: &Timestamp) -> serde_json::Value {
  json!({
    "seconds": timestamp.seconds,
    "nanos": timestamp.nanos,
  })
}

fn cp_ball_to_value(ball: &CpBall) -> serde_json::Value {
  json!({
    "pos": cp_vector2_to_value(&ball.pos),
    "vel": ball.vel.as_ref().map(cp_vector2_to_value),
  })
}

fn cp_tracked_robot_to_value(robot: &CpTrackedRobot) -> serde_json::Value {
  json!({
    "robot_id": robot.robot_id,
    "pos": cp_vector2_to_value(&robot.pos),
    "orientation": robot.orientation,
    "vel": robot.vel.as_ref().map(cp_vector2_to_value),
    "visibility": robot.visibility,
  })
}

fn cp_vector2_to_value(vec: &CpVector2) -> serde_json::Value {
  json!({
    "x": vec.x,
    "y": vec.y,
  })
}

fn cp_command_to_value(cmd: &CpCommand) -> serde_json::Value {
  json!({
    "state": cmd.state,
    "task": cmd.task,
    "pos": cmd.pos.as_ref().map(cp_vector2_to_value),
    "speed": cmd.speed,
    "orientation": cmd.orientation,
    "kick_orient": cmd.kick_orient,
    "kick_speed": cmd.kick_speed,
    "enemy_id": cmd.enemy_id,
  })
}

fn loki_endpoint(cfg: &Config) -> String {
  let addr = SocketAddrV4::new(cfg.logging.loki_host, cfg.logging.loki_port);
  format!("http://{addr}/loki/api/v1/push")
}

fn now_unix_nanos() -> u128 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .unwrap_or_default()
    .as_nanos()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::config::Config;
  use serde_json::Value;

  #[test]
  fn builds_loki_payload_grouped_by_robot() {
    let robot = CpRobot {
      robot_id: 1,
      timestamp: Timestamp {
        seconds: 12,
        nanos: 34,
      },
      packet_id: 10,
      ball: CpBall {
        pos: CpVector2 { x: 1, y: 2 },
        vel: Some(CpVector2 { x: 3, y: 4 }),
      },
      robots_yellow: vec![CpTrackedRobot {
        robot_id: 7,
        pos: CpVector2 { x: 5, y: 6 },
        orientation: 90,
        vel: Some(CpVector2 { x: 7, y: 8 }),
        visibility: 99,
      }],
      robots_blue: vec![],
      cmd: CpCommand {
        state: 1,
        task: 2,
        pos: Some(CpVector2 { x: 9, y: 10 }),
        speed: Some(11),
        orientation: Some(12),
        kick_orient: Some(13),
        kick_speed: Some(14),
        enemy_id: Some(15),
      },
    };

    let payload = build_push_body(&[
      OutgoingRobotLog {
        robot: robot.clone(),
        ts_ns: 111,
      },
      OutgoingRobotLog {
        robot: CpRobot {
          robot_id: 1,
          packet_id: 11,
          ..robot.clone()
        },
        ts_ns: 112,
      },
      OutgoingRobotLog {
        robot: CpRobot {
          robot_id: 2,
          packet_id: 12,
          ..robot
        },
        ts_ns: 113,
      },
    ]);

    let parsed: Value = serde_json::from_str(&payload).expect("valid loki json");
    let streams = parsed["streams"].as_array().expect("streams array");
    assert_eq!(streams.len(), 2);
    let labels = streams
      .iter()
      .map(|stream| stream["stream"]["robot_id"].as_str().unwrap().to_owned())
      .collect::<Vec<_>>();
    assert_eq!(labels, vec!["1".to_owned(), "2".to_owned()]);

    let values_1 = streams[0]["values"].as_array().expect("values array");
    assert_eq!(values_1.len(), 2);
    let first_inner: Value =
      serde_json::from_str(values_1[0][1].as_str().unwrap()).expect("inner json");
    assert_eq!(first_inner["packet_id"], 10);
    assert_eq!(first_inner["ball"]["pos"]["x"], 1);
    assert_eq!(first_inner["cmd"]["kick_speed"], 14);
    assert_eq!(first_inner["robots_yellow"][0]["robot_id"], 7);

    let second_inner: Value =
      serde_json::from_str(values_1[1][1].as_str().unwrap()).expect("inner json");
    assert_eq!(second_inner["packet_id"], 11);

    let values_2 = streams[1]["values"].as_array().expect("values array");
    assert_eq!(values_2.len(), 1);
    let third_inner: Value =
      serde_json::from_str(values_2[0][1].as_str().unwrap()).expect("inner json");
    assert_eq!(third_inner["packet_id"], 12);
  }

  #[test]
  fn constructs_endpoint_from_config() {
    let cfg = Config::default();
    let endpoint = loki_endpoint(&cfg);
    assert!(endpoint.starts_with("http://"));
    assert!(endpoint.ends_with("/loki/api/v1/push"));
  }
}
