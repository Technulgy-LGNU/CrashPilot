pub use crate::communication::Events;
#[cfg(feature = "loki")]
use crate::communication::loki::LokiPublisher;
#[cfg(feature = "loki")]
use crate::communication::loki::spawn_loki_publisher;
use crate::communication::robot_sender::{NetworkSender, RobotSender};
#[cfg(feature = "ssl_game_controller")]
pub use crate::communication::ssl_gc_handler::SslGameController;
use crate::communication::{EventShare, WebsocketOut, communication_receiver};
pub use crate::config::Config;
use crate::game_logic::game_logic;
use crate::game_logic::types::{BallData, GamePhase, PrepPhase, Robot, WorldState};
use crate::helpers::robot_data::create_robot_data;
#[cfg(feature = "prometheus")]
use crate::metrics::PrometheusMetrics;
use crate::utils::{FieldSetup, PacketBuffer, spawn_robot_socket};
use bangka::Bangka;
use core_dump::proto::cp_game_phase::{
  GamePhase as InterfaceGamePhase, PrepPhase as InterfacePrepPhase,
};
#[cfg(feature = "ssl_game_controller")]
use core_dump::proto::{AdvantageChoice, ControllerToTeam};
use core_dump::proto::{CpCommand, CpGamePhase, CpInterfaceWrapper, CpRobot};
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Instant;
use tokio::net::UdpSocket;
use tokio::time::{Duration, MissedTickBehavior, interval};

pub use crate::utils::RobotData;

pub mod communication;
pub mod config;
mod game_logic;
mod helpers;

#[cfg(feature = "interface")]
pub mod interface;
#[cfg(feature = "prometheus")]
mod metrics;
mod utils;

use crate::communication::RobotHeartbeat;
pub use core_dump;
use core_dump::types::Ai;
use core_dump::vec::types::Vec2;
use game_logic::types::Team;

#[cfg(feature = "ssl_game_controller")]
const TEAM_NAME: &str = "Robocup Junior SSL Team";

pub struct CrashPilot<C = CommunicationChannels, A: Ai = Bangka> {
  config: Config,
  #[cfg(feature = "prometheus")]
  metrics: PrometheusMetrics,
  #[cfg(feature = "loki")]
  loki: Option<LokiPublisher>,
  robots: HashMap<u32, RobotData>,
  robots_ws_data: HashMap<u32, CpCommand>,
  state: WorldState,
  ai_data: core_dump::types::GameState,
  #[cfg(feature = "viewer-debug")]
  last_ai_commands: core_dump::types::Commands,
  ai: A,
  team: i32,
  field_setup: FieldSetup,
  packet_buffer: PacketBuffer,
  comm: C,
  heartbeat: RobotHeartbeat,
  process_start: Instant,
  site: f32,
  sim_logic_dt: f32,
  #[cfg(feature = "sim-time")]
  last_sim_timestamp: Option<f64>,
  #[cfg(feature = "sim-time")]
  sim_timestamp: f32,
}

pub struct CommunicationChannels {
  robot_socket: UdpSocket,
  rx: EventShare,
  #[cfg(feature = "ssl_game_controller")]
  gc: SslGameController,
  ws_out: WebsocketOut,
}

pub trait Communication {
  fn request_desired_keeper(&self, _goalie: u8) {}
}

impl Communication for CommunicationChannels {
  #[cfg(feature = "ssl_game_controller")]
  fn request_desired_keeper(&self, goalie: u8) {
    let gc = self.gc.clone();
    tokio::spawn(async move {
      if let Err(err) = gc.desired_keeper(goalie as i32).await {
        eprintln!("Failed to request new goalie {goalie}: {err:#}");
      }
    });
  }
}

impl Communication for () {
  fn request_desired_keeper(&self, _goalie: u8) {}
}

impl CrashPilot {
  pub async fn default() -> Self {
    Self::with_ai(Bangka::default()).await
  }

  // pub async fn with_ai_checkpoint<P: AsRef<Path>>(path: P) -> Self {
  //   let ai = Bangka::load_auto(path).unwrap_or_else(|err| {
  //     panic!("failed to load AI checkpoint: {err}");
  //   });
  //
  //   Self::with_ai(ai).await
  // }

  pub async fn with_ai(ai: Bangka) -> Self {
    let process_start = Instant::now();
    // Interface as feature
    #[cfg(feature = "interface")]
    interface::spawn_interface();

    let config = match config::load_or_create_config("config.toml") {
      Ok(config) => config,
      Err(e) => panic!("{}", e),
    };

    // Heartbeats
    let robot_heartbeats = Arc::new((0..16).map(|_| AtomicU64::new(0)).collect::<Vec<_>>());

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
    #[cfg(feature = "ssl_game_controller")]
    let (rx, gc, ws_out) = match communication_receiver(&config, &robot_heartbeats, process_start) {
      Ok(comm) => (comm.events, comm.gc, comm.ws_out),
      Err(e) => panic!("{}", e),
    };
    #[cfg(not(feature = "ssl_game_controller"))]
    let (rx, ws_out) = match communication_receiver(&config, &robot_heartbeats, process_start) {
      Ok(comm) => (comm.events, comm.ws_out),
      Err(e) => panic!("{}", e),
    };

    // UDPSocket for robot communication
    let robot_socket = spawn_robot_socket(&config).await;

    let comm = CommunicationChannels {
      robot_socket,
      rx,
      #[cfg(feature = "ssl_game_controller")]
      gc,
      ws_out,
    };

    Self::from_parts(
      config,
      comm,
      ai,
      #[cfg(feature = "loki")]
      loki,
      #[cfg(feature = "prometheus")]
      metrics,
      robot_heartbeats,
      process_start,
    )
  }

  /// Sends the latest data to all robots
  #[inline]
  pub async fn robot_sender(&self) {
    let network_sender: NetworkSender = NetworkSender {
      socket: &self.comm.robot_socket,
      data: &self.robots,
      #[cfg(feature = "loki")]
      loki,
      heartbeats: &self.heartbeat,
      cfg: &self.config,
      process_start: self.process_start,
    };
    let _send_report = network_sender.send_to_all_robots();
    // if !send_report.failed.is_empty() {
    //   eprintln!(
    //     "Robot send: {} ok, {} failed",
    //     send_report.sent,
    //     send_report.failed.len()
    //   );
    //   for failure in &send_report.failed {
    //     eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
    //   }
    // }
    #[cfg(feature = "prometheus")]
    let failed_robot_ids: HashSet<u32> = send_report
      .failed
      .iter()
      .map(|failure| failure.robot_id)
      .collect();
    #[cfg(feature = "prometheus")]
    for robot_id in robots.keys().copied() {
      metrics
        .record_send_result(robot_id, !failed_robot_ids.contains(&robot_id))
        .await;
    }
  }

  pub async fn send(&mut self) {
    // Send the data to the robots
    self.robot_sender().await;

    // Websocket sender
    self.websocket_sender().await;

    // So the next packet has a higher id
    self.packet_buffer.packet_id += 1;
  }

  pub async fn step(&mut self) {
    self.recv().await;
    self.update();
    self.send().await;
  }

  pub async fn run(&mut self) {
    println!("Starting robots...");
    // Sending should not depend on receiving new packets: when vision/GC packets pause,
    // we still want to keep sending the latest known command/state to the robots.
    // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
    let mut tick = interval(Duration::from_millis(4)); // ~500 Hz
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
      tick.tick().await;

      self.step().await;
    }
  }

  pub async fn recv(&mut self) {
    // Drain the *latest* events from the shared state. We handle all types each tick,
    // not just one, so state stays fresh.
    let events = {
      let mut lock = self.comm.rx.write().await;
      lock.take()
    };

    #[cfg(feature = "prometheus")]
    if let Some(packet) = tracked.as_ref() {
      self.metrics.record_tracked_frame(&packet).await;
    }

    #[cfg(feature = "prometheus")]
    if let Some(packet) = rf.as_ref() {
      self.metrics.record_robot_feedback(packet).await;
    }

    self.interpret(events);
  }

  /// Broadcast the latest state to all websocket clients (CP -> interface)
  /// Note: if no clients are connected, send() returns an error; that's fine.
  #[inline]
  pub async fn websocket_sender(&self) {
    let ws_packet = self.interface_packet();

    self.comm.ws_out.publish(ws_packet).await; // Publish the packet to the WebSocketOut channel
  }

  #[cfg(feature = "ssl_game_controller")]
  pub fn game_controller(&self) -> &SslGameController {
    &self.comm.gc
  }

  #[cfg(feature = "ssl_game_controller")]
  pub async fn gc_desired_keeper(&self, id: i32) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.desired_keeper(id).await
  }

  #[cfg(feature = "ssl_game_controller")]
  pub async fn gc_advantage_choice(
    &self,
    choice: AdvantageChoice,
  ) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.advantage_choice(choice).await
  }

  #[cfg(feature = "ssl_game_controller")]
  pub async fn gc_substitute_bot(&self, requested: bool) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.substitute_bot(requested).await
  }

  #[cfg(feature = "ssl_game_controller")]
  pub async fn gc_ping(&self) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.ping().await
  }
}

impl<C: Default, A: Ai + Default> CrashPilot<C, A> {
  pub fn new(
    config: Config,
    #[cfg(feature = "loki")] loki: Option<LokiPublisher>,
    #[cfg(feature = "prometheus")] metrics: PrometheusMetrics,
  ) -> Self {
    Self::from_parts(
      config,
      C::default(),
      A::default(),
      #[cfg(feature = "loki")]
      loki,
      #[cfg(feature = "prometheus")]
      metrics,
      RobotHeartbeat::default(),
      Instant::now(),
    )
  }
}

impl<C, A: Ai> CrashPilot<C, A> {
  pub fn from_parts(
    config: Config,
    comm: C,
    ai: A,
    #[cfg(feature = "loki")] loki: Option<LokiPublisher>,
    #[cfg(feature = "prometheus")] metrics: PrometheusMetrics,
    heartbeats: RobotHeartbeat,
    process_start: Instant,
  ) -> Self {
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
            infos: Default::default(),
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
    // Field config
    let field_setup = FieldSetup::default();

    Self {
      config,
      #[cfg(feature = "prometheus")]
      metrics,
      #[cfg(feature = "loki")]
      loki,
      robots,
      robots_ws_data,
      state,
      ai_data: Default::default(),
      #[cfg(feature = "viewer-debug")]
      last_ai_commands: Default::default(),
      ai,
      team,
      field_setup,
      packet_buffer: PacketBuffer::default(),
      comm,
      heartbeat: heartbeats,
      process_start,
      site: 0.0,
      sim_logic_dt: 1.0,
      #[cfg(feature = "sim-time")]
      last_sim_timestamp: None,
      #[cfg(feature = "sim-time")]
      sim_timestamp: 0.0,
    }
  }

  pub fn get_ai(&self) -> &A {
    &self.ai
  }

  #[cfg(feature = "viewer-debug")]
  pub fn ai_commands(&self) -> &core_dump::types::Commands {
    &self.last_ai_commands
  }

  pub fn interpret(&mut self, events: Events) {
    if let Some(packet) = events.raw {
      self.packet_buffer.vis_raw = packet;

      // Create the FieldSetup Var
      if let Some(geometry) = self.packet_buffer.vis_raw.geometry.as_ref() {
        self.field_setup = geometry.into()
      }
    }

    if let Some(packet) = events.tracked {
      self.packet_buffer.vis_tracked = packet;
    }

    if let Some(packet) = events.ws {
      for robot_command in packet.robot_commands {
        self
          .robots_ws_data
          .insert(robot_command.robot_id, robot_command.command);
      }
      self.packet_buffer.interface_command = packet.interface_command;
      self.state.new_goalie = Some(self.packet_buffer.interface_command.game.goalkeeper_id as u8);
    }

    if let Some(packet) = events.gc {
      self.packet_buffer.referee = packet;
    }

    if let Some(packet) = events.rf
      && let Some((_, data)) = self
        .robots
        .iter_mut()
        .find(|(_, data)| data.msg.robot_id == packet.robot_id)
    {
      data.feedback = packet;
    }
  }

  pub fn update_data(&mut self) {
    self.update_sim_logic_dt();

    // Update site dependent on referee data && Also update team based on that
    // Start by getting own Team Color
    if self.packet_buffer.referee.yellow.name == "Robocup Junior SSL Team" {
      self.team = 1;
      // We are the yellow team, check on which site blue is and decide based on that
      if let Some(blue_pos_half) = self.packet_buffer.referee.blue_team_on_positive_half {
        self.site = if blue_pos_half { -1f32 } else { 1f32 }
      } else {
        // No valid gc data, use interface command
        self.site = if self.packet_buffer.interface_command.game.side {
          -1f32
        } else {
          1f32
        };
      }
    } else if self.packet_buffer.referee.blue.name == "Robpocup Junior SSL Team" {
      self.team = 2;
      // We are the blue team, check on whoch side we are and just assign that
      if let Some(blue_pos_half) = self.packet_buffer.referee.blue_team_on_positive_half {
        self.site = if blue_pos_half { 1f32 } else { -1f32 }
      } else {
        // No valid gc data, use interface command
        self.site = if self.packet_buffer.interface_command.game.side {
          -1f32
        } else {
          1f32
        };
      }
    } else {
      self.team = if self.packet_buffer.interface_command.game.team_color {
        2
      } else {
        1
      };
      // No valid gc data, use interface command
      self.site = if self.packet_buffer.interface_command.game.side {
        -1f32
      } else {
        1f32
      };
    }

    // Create state
    let ball_data = BallData::new(&self.packet_buffer.vis_tracked);
    let (robots_self, robots_opp) = Robot::new_from_tracked(
      &self.packet_buffer.vis_tracked,
      &ball_data.ball,
      self.team,
      &self.field_setup,
      self.site,
    );

    let state_team = Team::from_cp_team(self.team).unwrap_or(self.state.team);
    self.state.update(
      robots_self,
      robots_opp,
      ball_data,
      self.packet_buffer.referee.clone(),
      self.packet_buffer.interface_command.clone(),
      state_team,
      self.site,
    );

    create_robot_data(
      &mut self.robots,
      self.packet_buffer.packet_id,
      &self.packet_buffer.vis_tracked,
      &self.packet_buffer.vis_raw,
      &self.packet_buffer.interface_command,
      &self.field_setup,
    );

    // Create AI State
    self.update_ai_data();
  }

  pub fn logic_dt(&self) -> f32 {
    self.sim_logic_dt
  }

  #[cfg(feature = "sim-time")]
  fn update_sim_logic_dt(&mut self) {
    let Some(timestamp) = self
      .packet_buffer
      .vis_tracked
      .tracked_frame
      .as_ref()
      .map(|frame| frame.timestamp)
    else {
      return;
    };
    let dt = self
      .last_sim_timestamp
      .map(|previous| timestamp - previous)
      .filter(|dt| *dt > f64::EPSILON)
      .unwrap_or(1.0 / 60.0);
    self.last_sim_timestamp = Some(timestamp);
    self.sim_timestamp = timestamp as f32;
    self.sim_logic_dt = dt as f32;
  }

  #[cfg(not(feature = "sim-time"))]
  fn update_sim_logic_dt(&mut self) {}

  pub fn interpret_and_update(&mut self, events: Events) {
    self.interpret(events);
    self.update_data();
  }

  pub fn interface_packet(&self) -> CpInterfaceWrapper {
    CpInterfaceWrapper {
      vision_raw: Some(self.packet_buffer.vis_raw.clone()),
      vision_tracked: Some(self.packet_buffer.vis_tracked.clone()),
      gc_data: if self.packet_buffer.referee.packet_timestamp != 0 {
        Some(self.packet_buffer.referee.clone())
      } else {
        None
      },
      robot_commands: self
        .robots
        .values()
        .map(|robot| robot.msg.clone())
        .collect(),
      cp_gamephase: Some(CpGamePhase {
        game_phase: Some(interface_game_phase(self.state.phase) as i32),
        prep_phase: Some(interface_prep_phase(self.state.prep_phase) as i32),
      }),
    }
  }

  fn update_ai_data(&mut self) {
    // Update robots by filtering between own and opponent team
    self.ai_data.own_robots = self_robots_to_ai_robots(
      self.state.robots_self.clone(),
      self.field_setup,
      self.state.goalie.unwrap_or_default(),
    );
    self.ai_data.opp_robots = self_robots_to_ai_robots(
      self.state.robots_opp.clone(),
      self.field_setup,
      self.state.goalie.unwrap_or_default(),
    );

    let field = Vec2::new(
      self.field_setup.width as f32,
      self.field_setup.height as f32,
    );
    self.ai_data.ball.pos = self.state.ball.ball.pos / field;
    self.ai_data.ball.vel = self.state.ball.ball.vel / field;
    self.ai_data.ball.stop_pos = self
      .state
      .ball
      .kicked_ball
      .end_point
      .unwrap_or(self.state.ball.ball.pos)
      / field;
    self.ai_data.ball.stop_time = self
      .state
      .ball
      .kicked_ball
      .end_time
      .unwrap_or(self.default_ball_stop_time());
  }

  #[cfg(feature = "sim-time")]
  fn default_ball_stop_time(&self) -> f32 {
    self.sim_timestamp
  }

  #[cfg(not(feature = "sim-time"))]
  fn default_ball_stop_time(&self) -> f32 {
    Instant::now().elapsed().as_millis() as f32
  }
}

impl<C: Communication, A: Ai + Send> CrashPilot<C, A> {
  pub fn update_logic(&mut self) {
    // Actual game logic is going to happen here
    // First checks, on game state, and coordinating robots for that
    // Checks if one of multiple predetermine strategies apply
    //  - Goalie has Ball -> Chips automatically to the furthest own robot -> This robot should get the receive command
    game_logic(self)
  }

  pub fn update(&mut self) {
    self.update_data();
    self.update_logic();
  }

  pub fn step_with_data(
    &mut self,
    events: Events,
  ) -> (CpInterfaceWrapper, HashMap<u32, RobotData>) {
    self.interpret(events);
    self.update();

    let robot_data = self.robots.clone();

    (self.interface_packet(), robot_data)
  }

  pub fn step_logic(&mut self) -> (CpInterfaceWrapper, HashMap<u32, RobotData>) {
    self.update_logic();

    let robot_data = self.robots.clone();

    (self.interface_packet(), robot_data)
  }
}

fn self_robots_to_ai_robots(
  robots: Vec<Robot>,
  field: FieldSetup,
  goalie_robot: u8,
) -> core_dump::types::Robots {
  let mut ai_robots: core_dump::types::Robots = Default::default();
  for robot in robots {
    let robot_id = robot.robot_id;
    let is_goalie = robot_id == goalie_robot;

    let field_norm = Vec2::new(field.width as f32, field.height as f32);
    let ai_robot = core_dump::types::RobotState {
      id: robot_id,
      pos: (robot.pos.unwrap_or_default()) / field_norm,
      vel: (robot.vel.unwrap_or_default()) / field_norm,
      heading: robot.orientation / 360f32,
      angular_vel: robot.angular_vel / 3600f32,
      is_goalie,
    };

    ai_robots[robot_id as usize] = Some(ai_robot);
  }
  ai_robots
}

fn interface_game_phase(phase: GamePhase) -> InterfaceGamePhase {
  match phase {
    GamePhase::Unknown => InterfaceGamePhase::UnknownGamePhase,
    GamePhase::Halted => InterfaceGamePhase::Halted,
    GamePhase::Stopped => InterfaceGamePhase::Stopped,
    GamePhase::Running => InterfaceGamePhase::Running,
    GamePhase::Timeout => InterfaceGamePhase::Timeout,
    GamePhase::BallPlacement => InterfaceGamePhase::BallPlacement,
  }
}

fn interface_prep_phase(phase: PrepPhase) -> InterfacePrepPhase {
  match phase {
    PrepPhase::Unknown => InterfacePrepPhase::UnknownPrepPhase,
    PrepPhase::OffensiveKickoff => InterfacePrepPhase::OffensiveKickoff,
    PrepPhase::DefensiveKickoff => InterfacePrepPhase::DefensiveKickoff,
    PrepPhase::OffensivePenalty => InterfacePrepPhase::OffensivePenalty,
    PrepPhase::DefensivePenalty => InterfacePrepPhase::DefensivePenalty,
    PrepPhase::OffensiveFreeKick => InterfacePrepPhase::OffensiveFreeKick,
    PrepPhase::DefensiveFreeKick => InterfacePrepPhase::DefensiveFreeKick,
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn maps_game_phase_to_interface_enum() {
    assert_eq!(
      interface_game_phase(GamePhase::Unknown),
      InterfaceGamePhase::UnknownGamePhase
    );
    assert_eq!(
      interface_game_phase(GamePhase::Halted),
      InterfaceGamePhase::Halted
    );
    assert_eq!(
      interface_game_phase(GamePhase::Stopped),
      InterfaceGamePhase::Stopped
    );
    assert_eq!(
      interface_game_phase(GamePhase::Running),
      InterfaceGamePhase::Running
    );
    assert_eq!(
      interface_game_phase(GamePhase::Timeout),
      InterfaceGamePhase::Timeout
    );
    assert_eq!(
      interface_game_phase(GamePhase::BallPlacement),
      InterfaceGamePhase::BallPlacement
    );
  }

  #[test]
  fn maps_prep_phase_to_interface_enum() {
    assert_eq!(
      interface_prep_phase(PrepPhase::Unknown),
      InterfacePrepPhase::UnknownPrepPhase
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::OffensiveKickoff),
      InterfacePrepPhase::OffensiveKickoff
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::DefensiveKickoff),
      InterfacePrepPhase::DefensiveKickoff
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::OffensivePenalty),
      InterfacePrepPhase::OffensivePenalty
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::DefensivePenalty),
      InterfacePrepPhase::DefensivePenalty
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::OffensiveFreeKick),
      InterfacePrepPhase::OffensiveFreeKick
    );
    assert_eq!(
      interface_prep_phase(PrepPhase::DefensiveFreeKick),
      InterfacePrepPhase::DefensiveFreeKick
    );
  }
}
