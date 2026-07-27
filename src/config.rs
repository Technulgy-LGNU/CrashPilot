use core_dump::proto::CrashpilotVector2;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::fs;
use std::net::Ipv4Addr;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
  pub ssl: SslConfig,
  pub server: ServerConfig,
  pub logging: LoggingConfig,
  pub robots: HashMap<u32, RobotConfig>,
  #[serde(default)]
  pub world_model: WorldModelConfig,
}
impl Default for Config {
  fn default() -> Self {
    let mut robots = HashMap::new();

    robots.insert(
      1,
      RobotConfig {
        ip: Ipv4Addr::new(10, 0, 64, 101),
        port: None,
        substitution_pos: Default::default(),
      },
    );
    robots.insert(
      2,
      RobotConfig {
        ip: Ipv4Addr::new(10, 0, 64, 102),
        port: None,
        substitution_pos: Default::default(),
      },
    );
    robots.insert(
      3,
      RobotConfig {
        ip: Ipv4Addr::new(10, 0, 64, 103),
        port: None,
        substitution_pos: Default::default(),
      },
    );
    robots.insert(
      4,
      RobotConfig {
        ip: Ipv4Addr::new(10, 0, 64, 104),
        port: None,
        substitution_pos: Default::default(),
      },
    );

    Self {
      ssl: SslConfig::default(),
      server: ServerConfig::default(),
      logging: LoggingConfig::default(),
      robots,
      world_model: WorldModelConfig::default(),
    }
  }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default)]
pub struct WorldModelConfig {
  pub enabled: bool,
  pub history_ms: u64,
  pub fit_window_ms: u64,
  pub predict_timeout_ms: u64,
  pub invalid_timeout_ms: u64,
  pub ball_max_speed_mm_s: f32,
  pub ball_max_accel_mm_s2: f32,
  pub ball_outlier_gate_mm: f32,
  pub wall_offset_mm: f32,
  pub wall_restitution: f32,
  pub own_robot_max_accel_mm_s2: f32,
  pub own_robot_max_decel_mm_s2: f32,
  pub opponent_max_accel_mm_s2: f32,
}

impl Default for WorldModelConfig {
  fn default() -> Self {
    Self {
      enabled: true,
      history_ms: 2_000,
      fit_window_ms: 350,
      predict_timeout_ms: 300,
      invalid_timeout_ms: 500,
      ball_max_speed_mm_s: 12_000.0,
      ball_max_accel_mm_s2: 5_000.0,
      ball_outlier_gate_mm: 300.0,
      wall_offset_mm: 200.0,
      wall_restitution: 0.78,
      own_robot_max_accel_mm_s2: 2_000.0,
      own_robot_max_decel_mm_s2: 3_000.0,
      opponent_max_accel_mm_s2: 4_500.0,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SslConfig {
  pub ssl_vision_raw_ip: Ipv4Addr,
  pub ssl_vision_raw_port: u16,
  pub ssl_vision_tracked_ip: Ipv4Addr,
  pub ssl_vision_tracked_port: u16,
  pub ssl_gc_ip: Ipv4Addr,
  pub ssl_gc_port: u16,
  pub ssl_interface: Ipv4Addr,
  pub ssl_gc_msg_port: u16,
  pub ssl_gc_msg_ip: Ipv4Addr,
}
impl Default for SslConfig {
  fn default() -> Self {
    Self {
      ssl_vision_raw_ip: Ipv4Addr::new(224, 5, 23, 2),
      ssl_vision_raw_port: 10006,
      ssl_vision_tracked_ip: Ipv4Addr::new(224, 5, 23, 2),
      ssl_vision_tracked_port: 10010,
      ssl_gc_ip: Ipv4Addr::new(224, 5, 23, 2),
      ssl_gc_port: 10003,
      ssl_gc_msg_ip: Ipv4Addr::new(127, 0, 0, 1),
      ssl_gc_msg_port: 10008,
      ssl_interface: Ipv4Addr::new(192, 168, 0, 1),
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ServerConfig {
  pub robot_socket_host: Ipv4Addr,
  pub robot_socket_port: u16,
  pub robots_port: u16,
  pub robot_receive_port: u16,
  pub websocket_host: Ipv4Addr,
  pub websocket_port: u16,
}
impl Default for ServerConfig {
  fn default() -> Self {
    Self {
      robot_socket_host: Ipv4Addr::new(0, 0, 0, 0),
      robot_socket_port: 8192,
      robots_port: 1024,
      robot_receive_port: 2048,
      websocket_host: Ipv4Addr::new(0, 0, 0, 0),
      websocket_port: 4096,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct LoggingConfig {
  pub prometheus_host: Ipv4Addr,
  pub prometheus_port: u16,
  pub loki_host: Ipv4Addr,
  pub loki_port: u16,
}
impl Default for LoggingConfig {
  fn default() -> Self {
    Self {
      prometheus_host: Ipv4Addr::new(10, 0, 64, 2),
      prometheus_port: 9000,
      loki_host: Ipv4Addr::new(10, 0, 64, 2),
      loki_port: 3100,
    }
  }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct RobotConfig {
  pub ip: Ipv4Addr,
  #[serde(skip_serializing_if = "Option::is_none")]
  pub port: Option<u16>,
  pub substitution_pos: Vector2,
}
impl Default for RobotConfig {
  fn default() -> Self {
    Self {
      ip: Ipv4Addr::new(10, 0, 64, 101),
      port: None,
      substitution_pos: Vector2::default(),
    }
  }
}
impl RobotConfig {
  #[inline]
  pub fn destination_port(&self, server: &ServerConfig) -> u16 {
    self.port.unwrap_or(server.robots_port)
  }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Vector2 {
  pub x: i32,
  pub y: i32,
}
impl Default for Vector2 {
  fn default() -> Self {
    Self { x: 6200, y: 400 }
  }
}
impl Vector2 {
  #[inline]
  pub fn to_crashpilot_vec2(&self) -> CrashpilotVector2 {
    CrashpilotVector2 {
      x: self.x,
      y: self.y,
    }
  }
}

pub fn load_or_create_config(path: &str) -> Result<Config, Box<dyn Error>> {
  if !Path::new(path).exists() {
    let default_config = Config::default();

    let toml_string = toml::to_string_pretty(&default_config)?;
    fs::write(path, toml_string)?;

    return Ok(default_config);
  }

  let content = fs::read_to_string(path)?;
  let config: Config = toml::from_str(&content)?;

  Ok(config)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn robot_config_without_port_uses_global_robot_port() {
    let robot: RobotConfig = toml::from_str(
      r#"
ip = "10.0.64.101"
substitution_pos = { x = 800, y = 0 }
"#,
    )
    .expect("robot config should parse without a port");
    let mut server = ServerConfig::default();
    server.robots_port = 2049;

    assert_eq!(robot.port, None);
    assert_eq!(robot.destination_port(&server), 2049);
  }

  #[test]
  fn robot_config_port_overrides_global_robot_port() {
    let robot: RobotConfig = toml::from_str(
      r#"
ip = "10.0.64.101"
port = 3050
substitution_pos = { x = 800, y = 0 }
"#,
    )
    .expect("robot config should parse with a port override");
    let mut server = ServerConfig::default();
    server.robots_port = 2049;

    assert_eq!(robot.port, Some(3050));
    assert_eq!(robot.destination_port(&server), 3050);
  }
}
