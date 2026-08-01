use std::sync::RwLockWriteGuard;
use crate::communication::{EventShare, Events, RobotHeartbeat};
use crate::config;

pub fn robot_wifi_receiver<T>(cfg: &config::Config, heartbeats: &RobotHeartbeat, tx: EventShare, wrap: fn(T, RwLockWriteGuard<Events>)) where T: Send + Sync + 'static {
  
}
