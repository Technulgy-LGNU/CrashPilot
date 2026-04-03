use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::net::Ipv4Addr;
use std::fs;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
  pub ssl: SslConfig,
  pub server: ServerConfig,
  pub robots: HashMap<u16, RobotConfig>,
}
impl Default for Config {
  fn default() -> Self {
    let mut robots = HashMap::new();

    robots.insert(1, RobotConfig { ip: Ipv4Addr::new(10, 0, 64, 101) });
    robots.insert(2, RobotConfig { ip: Ipv4Addr::new(10, 0, 64, 102) });
    robots.insert(3, RobotConfig { ip: Ipv4Addr::new(10, 0, 64, 103) });
    robots.insert(4, RobotConfig { ip: Ipv4Addr::new(10, 0, 64, 104) });

    Self {
      ssl: SslConfig::default(),
      server: ServerConfig::default(),
      robots,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SslConfig {
  pub ssl_vision_ip: Ipv4Addr,
  pub ssl_vision_port: u16,
  pub ssl_gc_ip: Ipv4Addr,
  pub ssl_gc_port: u16,
}
impl Default for SslConfig {
  fn default() -> Self {
    Self {
      ssl_vision_ip: Ipv4Addr::new(224, 5, 23, 1),
      ssl_vision_port: 10003,
      ssl_gc_ip: Ipv4Addr::new(224, 5, 23, 2),
      ssl_gc_port: 10006,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ServerConfig {
  pub host: Ipv4Addr,
  pub port: u16,
}
impl Default for ServerConfig {
  fn default() -> Self {
    Self {
      host: Ipv4Addr::new(0, 0, 0, 0),
      port: 8192,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct RobotConfig {
  pub ip: Ipv4Addr,
}
impl Default for RobotConfig {
  fn default() -> Self {
    Self {
      ip: Ipv4Addr::new(10, 0, 64, 101),
    }
  }
}


pub fn load_or_create_config(path: &str, ) -> Result<Config, Box<dyn Error>> {
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
