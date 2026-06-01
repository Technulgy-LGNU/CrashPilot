use crate::config::Config;
use crate::proto::{RobotCp, TrackerWrapperPacket};
use anyhow::Context;
use http_body_util::Full;
use hyper::body::Incoming;
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use prost::bytes::Bytes;
use std::collections::HashMap;
use std::fmt::Write as _;
use std::net::{SocketAddr, SocketAddrV4};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::net::TcpListener;
use tokio::sync::RwLock;

#[derive(Clone, Default)]
pub struct PrometheusMetrics {
  inner: Arc<RwLock<MetricsState>>,
}

#[derive(Default, Clone)]
struct MetricsState {
  robots: HashMap<u32, RobotMetrics>,
  tracked_robot_velocities: HashMap<TrackedRobotKey, TrackedRobotVelocity>,
}

#[derive(Debug, Clone, Default)]
struct RobotMetrics {
  battery_voltage: Option<u32>,
  current: Option<u32>,
  kicker_ready: bool,
  has_ball: bool,
  has_error: Option<bool>,
  acting: Option<bool>,
  last_rec_packet: Option<u32>,
  feedback_seen_total: u64,
  send_success_total: u64,
  send_failure_total: u64,
  last_feedback_unix_seconds: Option<f64>,
  last_send_unix_seconds: Option<f64>,
  last_send_success: Option<bool>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct TrackedRobotKey {
  robot_id: u32,
  team: RobotTeam,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum RobotTeam {
  Yellow,
  Blue,
  Unknown,
}

impl RobotTeam {
  fn as_str(self) -> &'static str {
    match self {
      Self::Yellow => "yellow",
      Self::Blue => "blue",
      Self::Unknown => "unknown",
    }
  }
}

#[derive(Debug, Clone, Default)]
struct TrackedRobotVelocity {
  velocity_mps: f64,
  last_seen_unix_seconds: f64,
}

impl PrometheusMetrics {
  pub fn new() -> Self {
    Self::default()
  }

  pub async fn register_robot(&self, robot_id: u32) {
    let mut lock = self.inner.write().await;
    lock.robots.entry(robot_id).or_default();
  }

  pub async fn record_robot_feedback(&self, feedback: RobotCp) {
    let mut lock = self.inner.write().await;
    let entry = lock.robots.entry(feedback.robot_id).or_default();

    entry.battery_voltage = feedback.battery_voltage;
    entry.current = feedback.current;
    entry.kicker_ready = feedback.kicker_ready;
    entry.has_ball = feedback.has_ball;
    entry.has_error = feedback.has_error;
    entry.acting = feedback.acting;
    entry.last_rec_packet = feedback.last_rec_packet;
    entry.feedback_seen_total = entry.feedback_seen_total.saturating_add(1);
    entry.last_feedback_unix_seconds = Some(now_seconds());
  }

  pub async fn record_send_result(&self, robot_id: u32, success: bool) {
    let mut lock = self.inner.write().await;
    let entry = lock.robots.entry(robot_id).or_default();
    entry.last_send_unix_seconds = Some(now_seconds());
    entry.last_send_success = Some(success);

    if success {
      entry.send_success_total = entry.send_success_total.saturating_add(1);
    } else {
      entry.send_failure_total = entry.send_failure_total.saturating_add(1);
    }
  }

  pub async fn record_tracked_frame(&self, packet: &TrackerWrapperPacket) {
    let Some(frame) = packet.tracked_frame.as_ref() else {
      return;
    };

    let mut lock = self.inner.write().await;

    for robot in &frame.robots {
      let Some(robot_id) = robot.robot_id.id else {
        continue;
      };

      let key = TrackedRobotKey {
        robot_id,
        team: match robot.robot_id.team {
          Some(1) => RobotTeam::Yellow,
          Some(2) => RobotTeam::Blue,
          _ => RobotTeam::Unknown,
        },
      };

      let velocity_mps = robot
        .vel
        .map(|vel| f64::from((vel.x * vel.x + vel.y * vel.y).sqrt()))
        .unwrap_or(0.0);

      lock.tracked_robot_velocities.insert(
        key,
        TrackedRobotVelocity {
          velocity_mps,
          last_seen_unix_seconds: now_seconds(),
        },
      );
    }
  }

  pub async fn render(&self) -> String {
    let snapshot = {
      let lock = self.inner.read().await;
      (lock.robots.clone(), lock.tracked_robot_velocities.clone())
    };

    render_snapshot(&snapshot)
  }
}

pub async fn spawn_prometheus_server(cfg: &Config) -> anyhow::Result<PrometheusMetrics> {
  let metrics = PrometheusMetrics::new();
  let addr = SocketAddr::V4(SocketAddrV4::new(
    cfg.logging.prometheus_host,
    cfg.logging.prometheus_port,
  ));
  let listener = TcpListener::bind(addr)
    .await
    .with_context(|| format!("failed to bind Prometheus listener on {addr}"))?;

  let server_metrics = metrics.clone();
  tokio::spawn(async move {
    println!("Prometheus metrics listening on http://{addr}/metrics");

    loop {
      let (stream, remote_addr) = match listener.accept().await {
        Ok(pair) => pair,
        Err(e) => {
          eprintln!("Prometheus accept error: {e}");
          continue;
        }
      };

      let metrics = server_metrics.clone();
      tokio::spawn(async move {
        let io = TokioIo::new(stream);
        let service = service_fn(move |req: Request<Incoming>| {
          let metrics = metrics.clone();
          async move { Ok::<_, std::convert::Infallible>(handle_request(req, metrics).await) }
        });

        if let Err(e) = http1::Builder::new().serve_connection(io, service).await {
          eprintln!("Prometheus connection error from {remote_addr}: {e}");
        }
      });
    }
  });

  Ok(metrics)
}

async fn handle_request(
  req: Request<Incoming>,
  metrics: PrometheusMetrics,
) -> Response<Full<Bytes>> {
  match (req.method(), req.uri().path()) {
    (&Method::GET, "/metrics") => plain_response(StatusCode::OK, metrics.render().await),
    (&Method::GET, "/health") => plain_response(StatusCode::OK, "ok\n".to_owned()),
    (&Method::GET, _) => plain_response(StatusCode::NOT_FOUND, "not found\n".to_owned()),
    _ => plain_response(
      StatusCode::METHOD_NOT_ALLOWED,
      "method not allowed\n".to_owned(),
    ),
  }
}

fn plain_response(status: StatusCode, body: String) -> Response<Full<Bytes>> {
  Response::builder()
    .status(status)
    .header("Content-Type", "text/plain; charset=utf-8")
    .body(Full::new(Bytes::from(body)))
    .expect("failed to build HTTP response")
}

fn render_snapshot(
  snapshot: &(
    HashMap<u32, RobotMetrics>,
    HashMap<TrackedRobotKey, TrackedRobotVelocity>,
  ),
) -> String {
  let (robot_snapshot, velocity_snapshot) = snapshot;
  let mut out = String::with_capacity((robot_snapshot.len() + velocity_snapshot.len()) * 1024);

  macro_rules! metric {
    ($name:literal, $help:literal, $type:literal) => {{
      let _ = writeln!(out, "# HELP {} {}", $name, $help);
      let _ = writeln!(out, "# TYPE {} {}", $name, $type);
    }};
  }

  metric!(
    "crashpilot_robot_registered",
    "Whether CrashPilot knows about this robot_id.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_feedback_present",
    "Whether a feedback packet has been received for this robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_battery_voltage",
    "Battery voltage reported by the robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_current",
    "Current reported by the robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_kicker_ready",
    "Whether the kicker is ready.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_has_ball",
    "Whether the robot reports having the ball.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_has_error",
    "Whether the robot reports an error.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_acting",
    "Whether the robot reports that it is acting.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_last_rec_packet",
    "Last received packet id reported by the robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_feedback_seen_total",
    "Number of feedback packets received from this robot.",
    "counter"
  );
  metric!(
    "crashpilot_robot_send_success_total",
    "Number of UDP packets successfully sent to this robot.",
    "counter"
  );
  metric!(
    "crashpilot_robot_send_failure_total",
    "Number of UDP packets that failed to send to this robot.",
    "counter"
  );
  metric!(
    "crashpilot_robot_last_feedback_unix_seconds",
    "Unix timestamp of the latest feedback packet seen for this robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_last_send_unix_seconds",
    "Unix timestamp of the latest UDP send attempt for this robot.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_last_send_success",
    "Whether the latest UDP send attempt for this robot succeeded.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_velocity_mps",
    "Magnitude of the tracked robot velocity in meters per second.",
    "gauge"
  );
  metric!(
    "crashpilot_robot_velocity_last_seen_unix_seconds",
    "Unix timestamp of the latest tracked velocity sample for this robot.",
    "gauge"
  );

  let mut robot_ids: Vec<u32> = robot_snapshot.keys().copied().collect();
  robot_ids.sort_unstable();

  for robot_id in robot_ids {
    let robot = &robot_snapshot[&robot_id];
    let feedback_present = robot.last_feedback_unix_seconds.is_some();

    let _ = writeln!(
      out,
      "crashpilot_robot_registered{{robot_id=\"{}\"}} 1",
      robot_id
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_feedback_present{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(feedback_present)
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_battery_voltage{{robot_id=\"{}\"}} {}",
      robot_id,
      robot.battery_voltage.unwrap_or_default()
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_current{{robot_id=\"{}\"}} {}",
      robot_id,
      robot.current.unwrap_or_default()
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_kicker_ready{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(robot.kicker_ready)
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_has_ball{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(robot.has_ball)
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_has_error{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(robot.has_error.unwrap_or_default())
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_acting{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(robot.acting.unwrap_or_default())
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_last_rec_packet{{robot_id=\"{}\"}} {}",
      robot_id,
      robot.last_rec_packet.unwrap_or_default()
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_feedback_seen_total{{robot_id=\"{}\"}} {}",
      robot_id, robot.feedback_seen_total
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_send_success_total{{robot_id=\"{}\"}} {}",
      robot_id, robot.send_success_total
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_send_failure_total{{robot_id=\"{}\"}} {}",
      robot_id, robot.send_failure_total
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_last_feedback_unix_seconds{{robot_id=\"{}\"}} {}",
      robot_id,
      robot.last_feedback_unix_seconds.unwrap_or_default()
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_last_send_unix_seconds{{robot_id=\"{}\"}} {}",
      robot_id,
      robot.last_send_unix_seconds.unwrap_or_default()
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_last_send_success{{robot_id=\"{}\"}} {}",
      robot_id,
      bool_to_u8(robot.last_send_success.unwrap_or_default())
    );
  }

  let mut velocity_keys: Vec<&TrackedRobotKey> = velocity_snapshot.keys().collect();
  velocity_keys.sort_unstable_by(|a, b| {
    a.robot_id
      .cmp(&b.robot_id)
      .then(a.team.as_str().cmp(b.team.as_str()))
  });

  for key in velocity_keys {
    let robot = &velocity_snapshot[key];
    let _ = writeln!(
      out,
      "crashpilot_robot_velocity_mps{{robot_id=\"{}\",team=\"{}\"}} {}",
      key.robot_id,
      key.team.as_str(),
      robot.velocity_mps
    );
    let _ = writeln!(
      out,
      "crashpilot_robot_velocity_last_seen_unix_seconds{{robot_id=\"{}\",team=\"{}\"}} {}",
      key.robot_id,
      key.team.as_str(),
      robot.last_seen_unix_seconds
    );
  }

  out
}

fn bool_to_u8(value: bool) -> u8 {
  if value { 1 } else { 0 }
}

fn now_seconds() -> f64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .unwrap_or_default()
    .as_secs_f64()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::proto::{RobotId, Team, TrackedFrame, TrackedRobot, Vector2};

  #[tokio::test]
  async fn renders_robot_metrics_with_robot_id_label() {
    let metrics = PrometheusMetrics::new();
    metrics.register_robot(7).await;
    metrics
      .record_robot_feedback(RobotCp {
        robot_id: 7,
        battery_voltage: Some(12345),
        current: Some(678),
        kicker_ready: true,
        has_ball: false,
        has_error: Some(true),
        acting: Some(false),
        last_rec_packet: Some(99),
      })
      .await;
    metrics.record_send_result(7, true).await;

    let text = metrics.render().await;
    assert!(text.contains("crashpilot_robot_registered{robot_id=\"7\"} 1"));
    assert!(text.contains("crashpilot_robot_feedback_present{robot_id=\"7\"} 1"));
    assert!(text.contains("crashpilot_robot_battery_voltage{robot_id=\"7\"} 12345"));
    assert!(text.contains("crashpilot_robot_current{robot_id=\"7\"} 678"));
    assert!(text.contains("crashpilot_robot_kicker_ready{robot_id=\"7\"} 1"));
    assert!(text.contains("crashpilot_robot_has_ball{robot_id=\"7\"} 0"));
    assert!(text.contains("crashpilot_robot_has_error{robot_id=\"7\"} 1"));
    assert!(text.contains("crashpilot_robot_last_rec_packet{robot_id=\"7\"} 99"));
    assert!(text.contains("crashpilot_robot_send_success_total{robot_id=\"7\"} 1"));
    assert!(text.contains("crashpilot_robot_send_failure_total{robot_id=\"7\"} 0"));
  }

  #[tokio::test]
  async fn renders_tracked_velocity_with_robot_id_and_team_label() {
    let metrics = PrometheusMetrics::new();
    metrics
      .record_tracked_frame(&TrackerWrapperPacket {
        uuid: "demo".to_owned(),
        source_name: None,
        tracked_frame: Some(TrackedFrame {
          frame_number: 1,
          timestamp: 1.0,
          balls: vec![],
          robots: vec![TrackedRobot {
            robot_id: RobotId {
              id: Some(7),
              team: Some(Team::Yellow as i32),
            },
            pos: Vector2 { x: 0.0, y: 0.0 },
            orientation: 0.0,
            vel: Some(Vector2 { x: 3.0, y: 4.0 }),
            vel_angular: None,
            visibility: Some(1.0),
          }],
          kicked_ball: None,
          capabilities: vec![],
        }),
      })
      .await;

    let text = metrics.render().await;
    assert!(text.contains("crashpilot_robot_velocity_mps{robot_id=\"7\",team=\"yellow\"} 5"));
  }

  #[tokio::test]
  async fn registering_on_feedback_creates_series_for_unknown_robot() {
    let metrics = PrometheusMetrics::new();
    metrics
      .record_robot_feedback(RobotCp {
        robot_id: 42,
        battery_voltage: None,
        current: None,
        kicker_ready: false,
        has_ball: false,
        has_error: None,
        acting: None,
        last_rec_packet: None,
      })
      .await;

    let text = metrics.render().await;
    assert!(text.contains("robot_id=\"42\""));
    assert!(text.contains("crashpilot_robot_feedback_present{robot_id=\"42\"} 1"));
  }
}
