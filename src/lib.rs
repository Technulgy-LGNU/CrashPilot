#[cfg(feature = "loki")]
use crate::communication::loki::spawn_loki_publisher;
#[cfg(feature = "loki")]
use crate::communication::loki::LokiPublisher;
use crate::communication::robot_sender::RobotSender;
use crate::communication::{communication_receiver, EventShare, WebsocketOut};
use crate::config::Config;
use crate::game_logic::game_logic;
use crate::game_logic::types::{BallData, Robot, WorldState};
use crate::helpers::robot_data::create_robot_data;
#[cfg(feature = "prometheus")]
use crate::metrics::PrometheusMetrics;
use crate::utils::{
  robot_sender, spawn_robot_socket, websocket_sender, FieldSetup, PacketBuffer, RobotData,
};
use core_dump::proto::{
  CpCommand, CpRobot, InterfaceWrapperCp, Referee, RobotCp, SslWrapperPacket, TrackerWrapperPacket,
};
use std::collections::HashMap;
#[cfg(feature = "interface")]
use std::fs;
#[cfg(feature = "interface")]
use std::os::unix::fs::PermissionsExt;
#[cfg(feature = "interface")]
use std::process::Command;
use tokio::net::UdpSocket;
use tokio::time::{interval, Duration, MissedTickBehavior};

mod communication;
mod config;
mod game_logic;
mod helpers;
mod interface;
#[cfg(feature = "prometheus")]
mod metrics;
mod utils;

pub struct CrashPilot {
  config: Config,
  #[cfg(feature = "prometheus")]
  metrics: PrometheusMetrics,
  #[cfg(feature = "loki")]
  loki: Option<LokiPublisher>,
  robot_socket: UdpSocket,
  robots: HashMap<u32, RobotData>,
  robots_ws_data: HashMap<u32, CpCommand>,
  rx: EventShare,
  ws_out: WebsocketOut,
  state: WorldState,
  team: i32,
  site: i32,
  field_setup: FieldSetup,
  packet_buffer: PacketBuffer,
}

impl CrashPilot {
  pub async fn default() -> Self {
    let config = match config::load_or_create_config("config.toml") {
      Ok(config) => config,
      Err(e) => panic!("{}", e),
    };

    // Prometheus metrics endpoint and shared per-robot registry.
    #[cfg(feature = "prometheus")]
    let metrics: PrometheusMetrics = match metrics::spawn_prometheus_server(&config).await {
      Ok(metrics) => metrics,
      Err(e) => panic!("{}", e),
    };

    #[cfg(feature = "prometheus")]
    for robot_id in config.robots.keys().copied() {
      metrics.register_robot(robot_id).await;
    }

    #[cfg(feature = "loki")]
    let loki = spawn_loki_publisher(&config);

    // Receiver for the communication
    let (rx, ws_out) = match communication_receiver(&config).await {
      Ok(comm) => (comm.events, comm.ws_out),
      Err(e) => panic!("{}", e),
    };

    // UDPSocket for robot communication
    let robot_socket = spawn_robot_socket(&config).await;

    // Robots Hashmap
    let mut robots: HashMap<u32, RobotData> = HashMap::new();
    for robot in config.robots.iter() {
      robots.insert(
        *robot.0,
        RobotData {
          msg: CpRobot {
            robot_id: *robot.0,
            timestamp: Default::default(),
            packet_id: 0,
            ball: Default::default(),
            robots_yellow: vec![],
            robots_blue: vec![],
            cmd: Default::default(),
          },
          feedback: Default::default(),
        },
      );
    }
    // Initialize the hashmap for the websocket data, which will be used to store the last command received for each robot
    let mut robots_ws_data: HashMap<u32, CpCommand> = HashMap::new();
    for robot in config.robots.iter() {
      robots_ws_data.insert(*robot.0, Default::default());
    }

    // Other Vars
    let state = WorldState::default();
    // Team
    //  - 0: Unknown
    //  - 1: Yellow
    //  - 2: Blue
    let team: i32 = 0;
    let site: i32 = 1;
    // Field config
    let field_setup = FieldSetup::default();

    Self {
      config,
      #[cfg(feature = "prometheus")]
      metrics,
      #[cfg(feature = "loki")]
      loki: Some(loki),
      robot_socket,
      robots,
      robots_ws_data,
      rx,
      ws_out,
      state,
      team,
      site,
      field_setup,
      packet_buffer: PacketBuffer::default(),
    }
  }

  pub fn interpret(
    &mut self,
    raw: Option<SslWrapperPacket>,
    tracked: Option<TrackerWrapperPacket>,
    ws: Option<InterfaceWrapperCp>,
    gc: Option<Referee>,
    rf: Option<RobotCp>,
  ) {
    if let Some(packet) = raw {
      self.packet_buffer.vis_raw = packet;

      // Create the FieldSetup Var
      if let Some(geometry) = self.packet_buffer.vis_raw.geometry.as_ref() {
        self.field_setup = geometry.into()
      }
    }

    if let Some(packet) = tracked {
      self.packet_buffer.vis_tracked = packet;
    }

    if let Some(packet) = ws {
      println!("{:?}", packet);
      for robot_command in packet.robot_commands {
        self
          .robots_ws_data
          .insert(robot_command.robot_id, robot_command.command);
      }
      self.packet_buffer.interface_command = packet.interface_command;

      if self.packet_buffer.interface_command.game.team_color {
        self.team = 2;
      } else {
        self.team = 1;
      }

      if self.packet_buffer.interface_command.game.side {
        self.site = -1;
      } else {
        self.site = 1;
      }
    }

    if let Some(packet) = gc {
      self.packet_buffer.referee = packet;
    }

    if let Some(packet) = rf {
      if let Some((_, data)) = self
        .robots
        .iter_mut()
        .find(|(_, data)| data.msg.robot_id == packet.robot_id)
      {
        data.feedback = packet;
      }
    }
  }

  pub async fn recv(&mut self) {
    // Drain the *latest* events from the shared state. We handle all types each tick,
    // not just one, so state stays fresh.
    let (raw, tracked, ws, gc, rf) = {
      let mut lock = self.rx.lock().await;
      (
        lock.0.take(),
        lock.1.take(),
        lock.2.take(),
        lock.3.take(),
        lock.4.take(),
      )
    };


    #[cfg(feature = "prometheus")]
    if let Some(packet) = tracked.as_ref() {
      self.metrics.record_tracked_frame(&packet).await;
    }

    #[cfg(feature = "prometheus")]
    if let Some(packet) = rf.as_ref() {
      self.metrics.record_robot_feedback(packet).await;
    }



    self.interpret(raw, tracked, ws, gc, rf);

  }

  pub fn update(&mut self) {
    // Create state
    let ball_data = BallData::new(&self.packet_buffer.vis_tracked);

    self.state.update(
      Robot::new_from_tracked(
        &self.packet_buffer.vis_tracked,
        &ball_data.ball,
        self.team,
        self.site as f32,
        &self.field_setup,
      ),
      ball_data,
      self.packet_buffer.referee.clone(),
      self.packet_buffer.interface_command.clone(),
    );

    create_robot_data(
      &mut self.robots,
      self.packet_buffer.packet_id,
      &self.packet_buffer.vis_tracked,
      &self.packet_buffer.vis_raw,
      &self.packet_buffer.interface_command,
    );

    // Actual game logic is going to happen here
    // First checks, on game state, and coordinating robots for that
    // Checks if one of multiple predetermine strategies apply
    //  - Goalie has Ball -> Chips automatically to the furthest own robot -> This robot should get the receive command
    game_logic(
      &self.config,
      &mut self.robots,
      &mut self.state,
      &self.robots_ws_data,
    )
  }

  pub async fn send(&mut self) {
    // Send the data to the robots
    robot_sender(
      &self.config,
      &self.robot_socket,
      &self.robots,
      #[cfg(feature = "prometheus")]
      &self.metrics,
      #[cfg(feature = "loki")]
      self.loki.as_ref(),
    )
        .await;

    // Websocket sender
    websocket_sender(
      &self.packet_buffer.vis_raw,
      &self.packet_buffer.vis_tracked,
      &self.packet_buffer.referee,
      &self.robots,
      &self.ws_out,
    )
        .await;

    // So the next packet has a higher id
    self.packet_buffer.packet_id += 1;

  }

  pub async fn run(&mut self) {
    println!("Starting robots...");

    // Sending should not depend on receiving new packets: when vision/GC packets pause,
    // we still want to keep sending the latest known command/state to the robots.
    // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
    let mut tick = interval(Duration::from_millis(2)); // ~500 Hz
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
      tick.tick().await;

      self.tick().await;
    }
  }

  pub async fn tick(&mut self) {
    self.recv().await;
    self.update();
    self.send().await;
  }
}
