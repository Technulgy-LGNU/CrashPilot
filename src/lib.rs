pub use crate::communication::Events;
#[cfg(feature = "loki")]
use crate::communication::loki::LokiPublisher;
#[cfg(feature = "loki")]
use crate::communication::loki::spawn_loki_publisher;
use crate::communication::robot_sender::{NetworkSender, RobotSender};
pub use crate::communication::ssl_gc_handler::SslGameController;
use crate::communication::{EventShare, WebsocketOut, communication_receiver};
pub use crate::config::Config;
use crate::game_logic::game_logic;
use crate::game_logic::types::{BallData, Robot, WorldState};
use crate::helpers::robot_data::create_robot_data;
#[cfg(feature = "prometheus")]
use crate::metrics::PrometheusMetrics;
use crate::utils::{FieldSetup, PacketBuffer, spawn_robot_socket};
use core_dump::proto::{AdvantageChoice, ControllerToTeam, CpCommand, CpInterfaceWrapper, CpRobot};
use std::collections::HashMap;
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

use artificial_incompetence::{Ai, ArtificialIncompetence};
pub use core_dump;
use core_dump::vec::types::Vec2;

const TEAM_NAME: &str = "Robocup Junior SSL Team";

pub struct CrashPilot<C = CommunicationChannels, A: Ai = ArtificialIncompetence> {
  config: Config,
  #[cfg(feature = "prometheus")]
  metrics: PrometheusMetrics,
  #[cfg(feature = "loki")]
  loki: Option<LokiPublisher>,
  robots: HashMap<u32, RobotData>,
  robots_ws_data: HashMap<u32, CpCommand>,
  state: WorldState,
  ai_data: artificial_incompetence::types::GameState,
  ai: A,
  team: i32,
  site: i32,
  field_setup: FieldSetup,
  packet_buffer: PacketBuffer,
  comm: C,
}

pub struct CommunicationChannels {
  robot_socket: UdpSocket,
  rx: EventShare,
  gc: SslGameController,
  ws_out: WebsocketOut,
}

impl CrashPilot {
  pub async fn default() -> Self {
    // Interface as feature
    #[cfg(feature = "interface")]
    interface::spawn_interface();

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
    let (rx, gc, ws_out) = match communication_receiver(&config) {
      Ok(comm) => (comm.events, comm.gc, comm.ws_out),
      Err(e) => panic!("{}", e),
    };

    // UDPSocket for robot communication
    let robot_socket = spawn_robot_socket(&config).await;

    let comm = CommunicationChannels {
      robot_socket,
      rx,
      gc,
      ws_out,
    };

    Self::from_parts(
      config,
      comm,
      ArtificialIncompetence::default(),
      #[cfg(feature = "loki")]
      loki,
      #[cfg(feature = "prometheus")]
      metrics,
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
    };
    let send_report = network_sender.send_to_all_robots(&self.config).await;
    if !send_report.failed.is_empty() {
      eprintln!(
        "Robot send: {} ok, {} failed",
        send_report.sent,
        send_report.failed.len()
      );
      for failure in &send_report.failed {
        eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
      }
    }
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
    let mut tick = interval(Duration::from_millis(8)); // ~500 Hz
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

  pub fn game_controller(&self) -> &SslGameController {
    &self.comm.gc
  }

  pub async fn gc_desired_keeper(&self, id: i32) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.desired_keeper(id).await
  }

  pub async fn gc_advantage_choice(
    &self,
    choice: AdvantageChoice,
  ) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.advantage_choice(choice).await
  }

  pub async fn gc_substitute_bot(&self, requested: bool) -> anyhow::Result<ControllerToTeam> {
    self.comm.gc.substitute_bot(requested).await
  }

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
    let site: i32 = 1;
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
      ai,
      team,
      site,
      field_setup,
      packet_buffer: PacketBuffer::default(),
      comm,
    }
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
    // Create state
    let ball_data = BallData::new(&self.packet_buffer.vis_tracked);
    let (robots_self, robots_opp) = Robot::new_from_tracked(
      &self.packet_buffer.vis_tracked,
      &ball_data.ball,
      self.team,
      self.site as f32,
      &self.field_setup,
    );

    self.state.update(
      robots_self,
      robots_opp,
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
      &self.field_setup,
    );

    // Create AI State
    self.update_ai_data();
  }

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

  pub fn interpret_and_update(&mut self, events: Events) {
    self.interpret(events);
    self.update_data();
  }

  pub fn step_logic(&mut self) -> (CpInterfaceWrapper, HashMap<u32, RobotData>) {
    self.update_logic();

    let robot_data = self.robots.clone();

    (self.interface_packet(), robot_data)
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
    }
  }

  fn update_ai_data(&mut self) {
    // Update robots by filtering between own and opponent team
    self.ai_data.own_robots = self_robots_to_ai_robots(
      self.state.robots_self.clone(),
      self.field_setup.clone(),
      self.state.goalie.unwrap_or_default(),
    );
    self.ai_data.opp_robots = self_robots_to_ai_robots(
      self.state.robots_opp.clone(),
      self.field_setup.clone(),
      self.state.goalie.unwrap_or_default(),
    );

    self.ai_data.ball.pos = self.state.ball.ball.pos
      / Vec2::new(
        self.field_setup.width as f32,
        self.field_setup.height as f32,
      );
    self.ai_data.ball.vel = self.state.ball.ball.vel / Vec2::new(10000f32, 10000f32);
    self.ai_data.ball.stop_pos = self.state.ball.kicked_ball.end_point.unwrap_or(
      self.state.ball.ball.pos
        / Vec2::new(
          self.field_setup.width as f32,
          self.field_setup.height as f32,
        ),
    ) / Vec2::new(
      self.field_setup.width as f32,
      self.field_setup.height as f32,
    );
    self.ai_data.ball.stop_time = self
      .state
      .ball
      .kicked_ball
      .end_time
      .unwrap_or(Instant::now().elapsed().as_millis() as f32);
  }
}

fn self_robots_to_ai_robots(
  robots: Vec<Robot>,
  field: FieldSetup,
  goalie_robot: u8,
) -> artificial_incompetence::types::Robots {
  let mut ai_robots: artificial_incompetence::types::Robots = Default::default();
  for robot in robots {
    let robot_id = robot.robot_id;
    let is_goalie = robot_id == goalie_robot;

    let ai_robot = artificial_incompetence::types::RobotState {
      id: robot_id,
      pos: (robot.pos.unwrap_or_default()) / Vec2::new(field.width as f32, field.height as f32),
      vel: (robot.vel.unwrap_or_default()) / Vec2::new(field.width as f32, field.height as f32),
      heading: robot.orientation / 360f32,
      angular_vel: robot.angular_vel / 3600f32,
      is_goalie,
    };

    ai_robots[robot_id as usize] = Some(ai_robot);
  }
  ai_robots
}
