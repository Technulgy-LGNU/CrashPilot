use core_dump::vec::types::Vec2;
use serde::{Deserialize, Serialize};
use std::fs;
use std::net::Ipv4Addr;
use std::path::Path;

#[derive(Debug, Default, Deserialize, Serialize)]
pub struct Config {
  pub ssl: SslConfig,
  pub logging: LoggingConfig,
  pub cp_config: CPConfig,
  pub robots: Vec<RobotConfig>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SslConfig {
  pub vision_raw_ip: Ipv4Addr,
  pub vision_raw_port: u16,
  pub vision_tracked_ip: Ipv4Addr,
  pub vision_tracked_port: u16,
  pub game_controller_ip: Ipv4Addr,
  pub game_controller_port: u16,
  pub ssl_interface: Ipv4Addr,
}
impl Default for SslConfig {
  fn default() -> Self {
    Self {
      vision_raw_ip: Ipv4Addr::new(224, 5, 23, 2),
      vision_raw_port: 10006,
      vision_tracked_ip: Ipv4Addr::new(224, 5, 23, 2),
      vision_tracked_port: 10010,
      game_controller_ip: Ipv4Addr::new(224, 5, 23, 1),
      game_controller_port: 10003,
      ssl_interface: Ipv4Addr::new(0, 0, 0, 0),
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct LoggingConfig {
  pub prometheus_host: Ipv4Addr,
  pub prometheus_port: u16,
}
impl Default for LoggingConfig {
  fn default() -> Self {
    Self {
      prometheus_host: Ipv4Addr::new(10, 0, 64, 41),
      prometheus_port: 9090,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct CPConfig {
  pub interface_host: Ipv4Addr,
  pub interface_port: u16,
  pub cp_multicast_host: Ipv4Addr,
  pub cp_multicast_port: u16,
  pub cp_multicast_interface: Ipv4Addr,
}
impl Default for CPConfig {
  fn default() -> Self {
    Self {
      interface_host: Ipv4Addr::new(127, 0, 0, 1),
      interface_port: 8080,
      cp_multicast_host: Ipv4Addr::new(224, 42, 69, 1),
      cp_multicast_port: 1024,
      cp_multicast_interface: Ipv4Addr::new(0, 0, 0, 0),
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct RobotConfig {
  pub robot_id: u8,
  pub hold_point: Vec2<i32>,
}
impl Default for RobotConfig {
  fn default() -> Self {
    Self {
      robot_id: 0,
      hold_point: Vec2::new(0, 0),
    }
  }
}

pub fn load_or_create_config(path: &str) -> anyhow::Result<Config> {
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
