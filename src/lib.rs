use crate::communication::{CommunicationHandles, communication_receiver};
use crate::config::Config;
use crate::types::{PacketBuffer, WorldModel};
#[cfg(not(feature = "sim"))]
use core_dump::protocol::robot_command_frame::RobotCommandFrame;
use core_dump::protocol::robot_command_wire::RobotCommandWire;
use std::time::Duration;
use tokio::time::{MissedTickBehavior, interval};

mod communication;
mod config;
mod logic;
mod types;

pub struct CrashPilot<C = CommunicationHandles> {
  config: Config,
  state: WorldModel,
  robot_command: [RobotCommandWire; 6],
  comm: C,
  packet_buffer: PacketBuffer,
}

impl CrashPilot {
  pub async fn default() -> Self {
    let config_path =
      std::env::var("CRASHPILOT_CONFIG").unwrap_or_else(|_| "config.toml".to_string());
    let config = match config::load_or_create_config(&config_path) {
      Ok(config) => config,
      Err(e) => panic!("{}", e),
    };

    let comm = match communication_receiver(&config) {
      Ok(comm) => comm,
      Err(err) => panic!("Failed to initialize communication: {}", err),
    };

    Self {
      config,
      state: WorldModel::default(),
      robot_command: [RobotCommandWire::default(); 6],
      comm,
      packet_buffer: PacketBuffer::default(),
    }
  }

  pub async fn run(&mut self) {
    println!("Starting robots...");
    // Sending should not depend on receiving new packets: when vision/GC packets pause,
    // we still want to keep sending the latest known command/state to the robots.
    // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
    let mut tick = interval(Duration::from_millis(4)); // ~250 Hz
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
      tick.tick().await;

      self.step().await;
    }
  }

  #[inline]
  pub async fn step(&mut self) {
    self.recv().await;
    self.update();
    #[cfg(not(feature = "sim"))]
    self.send().await;
    #[cfg(feature = "sim")]
    self.send_sim().await;
  }

  /// Drains all the communications channels and puts them in the Packet Buffer
  #[inline]
  pub async fn recv(&mut self) {
    // Take every pending value while holding the lock only once. Producers can
    // start filling `events` again as soon as this block ends.
    let events = {
      let mut events = self.comm.events.write().await;
      events.take()
    };

    #[cfg(feature = "ssl_vision")]
    if let Some(packet) = events.raw {
      self.packet_buffer.vis_raw = packet;
    }

    #[cfg(feature = "ssl_vision")]
    if let Some(packet) = events.tracked {
      self.packet_buffer.vis_tracked = packet;
    }

    if let Some(packet) = events.gc {
      self.packet_buffer.gc = packet;
    }

    // Each robot-data kind arrives independently, so retain the last value for
    // kinds which did not receive a new packet during this tick.
    for (buffered, received) in self
      .packet_buffer
      .robot_data
      .iter_mut()
      .zip(events.robot_data)
    {
      if received.robot_telemetry.is_some() {
        buffered.robot_telemetry = received.robot_telemetry;
      }
      if received.robot_sensor.is_some() {
        buffered.robot_sensor = received.robot_sensor;
      }
      if received.robot_debug.is_some() {
        buffered.robot_debug = received.robot_debug;
      }
    }
  }

  /// Updates everything based on the received data
  /// And then creates the corresponding commands
  #[inline]
  pub fn update(&mut self) {}

  /// Sends the commands to the robots and
  /// the necessary data to the interface
  #[cfg(not(feature = "sim"))]
  pub async fn send(&mut self) {
    let mut commands = [RobotCommandWire::default(); 12];
    commands[..self.robot_command.len()].copy_from_slice(&self.robot_command);

    let frame = RobotCommandFrame {
      seq: self.packet_buffer.packet_id as u16,
      commands,
      ..RobotCommandFrame::default()
    };

    // The Wi-Fi task owns the socket and repeatedly transmits this latest frame.
    *self.comm.robots_out.write().await = Some(frame);
    self.packet_buffer.packet_id = self.packet_buffer.packet_id.wrapping_add(1);
  }

  /// Sends the data directly to the sim
  #[cfg(feature = "sim")]
  pub async fn send_sim(&mut self) {}
}
