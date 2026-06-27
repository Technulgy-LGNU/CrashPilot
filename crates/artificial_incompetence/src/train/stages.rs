use std::error::Error;
use std::f64::consts::PI;
use std::path::PathBuf;

use core_dump::vec::types::Vec2;
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use serde::{Deserialize, Serialize};
use simhark::viewer::{ViewerConfig, ViewerServer};
use simhark::{
  MoveCommand, SimulationEngine, TeamColor, TeleportBall, TeleportRobot, WorldCommand, WorldConfig,
  WorldState,
};
#[cfg(feature = "sumatra-opponent")]
use simhark::{SumatraSimNetConfig, SumatraSimNetServer};
#[cfg(feature = "sumatra-opponent")]
use simhark_sumatra::{SumatraInstance, SumatraLaunchConfig};
use tch::Device;
use tch::nn::VarStore;

use crate::loader::{CheckpointMetadata, ModelLoader};
use crate::train::reward::RewardMode;
use crate::train::trainer::Trainer;
use crate::{BallState, Commands, GameState, RobotCommand, RobotState, UpdateResult};

pub type TrainResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrainingStage {
  TouchBall,
  DribbleToGoal,
  ShootGoal,
  PassReceive,
  OneVsOne,
  ScriptedScrimmage,
  SumatraOpponent,
}

pub const TRAINING_STAGES_IN_ORDER: [TrainingStage; 7] = [
  TrainingStage::TouchBall,
  TrainingStage::DribbleToGoal,
  TrainingStage::ShootGoal,
  TrainingStage::PassReceive,
  TrainingStage::OneVsOne,
  TrainingStage::ScriptedScrimmage,
  TrainingStage::SumatraOpponent,
];

impl TrainingStage {
  pub fn name(self) -> &'static str {
    match self {
      Self::TouchBall => "touch_ball",
      Self::DribbleToGoal => "dribble_to_goal",
      Self::ShootGoal => "shoot_goal",
      Self::PassReceive => "pass_receive",
      Self::OneVsOne => "one_vs_one",
      Self::ScriptedScrimmage => "scripted_scrimmage",
      Self::SumatraOpponent => "sumatra_opponent",
    }
  }

  pub fn reward_mode(self) -> RewardMode {
    match self {
      Self::TouchBall => RewardMode::TouchBall,
      Self::DribbleToGoal => RewardMode::DribbleToGoal,
      Self::ShootGoal => RewardMode::ShootGoal,
      Self::PassReceive => RewardMode::PassReceive,
      Self::OneVsOne | Self::ScriptedScrimmage | Self::SumatraOpponent => RewardMode::Full,
    }
  }

  fn robots_per_team(self) -> usize {
    match self {
      Self::TouchBall | Self::DribbleToGoal | Self::ShootGoal | Self::OneVsOne => 1,
      Self::PassReceive => 2,
      Self::ScriptedScrimmage => 3,
      Self::SumatraOpponent => 6,
    }
  }
}

#[derive(Debug, Clone)]
pub struct TrainOptions {
  pub updates: usize,
  pub worlds: usize,
  pub rollout_steps: usize,
  pub checkpoint_every: usize,
  pub checkpoint_dir: PathBuf,
  pub run_name: Option<String>,
  pub model_path: Option<PathBuf>,
  pub learning_rate: f64,
  pub device: Device,
  pub sumatra_base_port: u16,
  pub sumatra_repo_root: Option<PathBuf>,
  pub viewer: bool,
  pub viewer_port: Option<u16>,
}

impl Default for TrainOptions {
  fn default() -> Self {
    Self {
      updates: 1_000,
      worlds: 64,
      rollout_steps: 128,
      checkpoint_every: 25,
      checkpoint_dir: PathBuf::from("checkpoints/artificial_incompetence"),
      run_name: None,
      model_path: None,
      learning_rate: 1e-3,
      device: Device::Cpu,
      sumatra_base_port: 14242,
      sumatra_repo_root: Some("../Sumatra".into()),
      viewer: false,
      viewer_port: None,
    }
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingReport {
  pub stage: TrainingStage,
  pub updates: usize,
  pub worlds: usize,
  pub rollout_steps: usize,
  pub last_checkpoint: Option<PathBuf>,
  pub last_loss: Option<f64>,
  pub last_policy_loss: Option<f64>,
  pub last_value_loss: Option<f64>,
  pub last_entropy: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct EvaluationOptions {
  pub checkpoint_dir: PathBuf,
  pub run_name: Option<String>,
  pub device: Device,
  pub steps: usize,
}

impl Default for EvaluationOptions {
  fn default() -> Self {
    Self {
      checkpoint_dir: PathBuf::from("checkpoints/artificial_incompetence"),
      run_name: None,
      device: Device::Cpu,
      steps: 240,
    }
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvaluationReport {
  pub stage: TrainingStage,
  pub worlds: usize,
  pub steps: usize,
  pub contact_worlds: usize,
  pub contact_rate: f64,
  pub avg_final_ball_distance: f64,
  pub avg_initial_ball_x: f64,
  pub avg_final_ball_x: f64,
  pub avg_ball_x_progress: f64,
  pub blue_goals: usize,
  pub yellow_goals: usize,
}

pub fn train_touch_ball(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::TouchBall, opts)
}

pub fn train_dribble_to_goal(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::DribbleToGoal, opts)
}

pub fn train_shoot_goal(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::ShootGoal, opts)
}

pub fn train_pass_receive(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::PassReceive, opts)
}

pub fn train_one_vs_one(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::OneVsOne, opts)
}

pub fn train_scripted_scrimmage(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::ScriptedScrimmage, opts)
}

pub fn train_sumatra_opponent(opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(TrainingStage::SumatraOpponent, opts)
}

pub fn train_all_stages(opts: TrainOptions) -> TrainResult<Vec<TrainingReport>> {
  let mut reports = Vec::with_capacity(TRAINING_STAGES_IN_ORDER.len());
  let mut stage_opts = opts.clone();
  let viewer = TrainingViewer::spawn(&opts)?;

  for (stage_index, stage) in TRAINING_STAGES_IN_ORDER.iter().copied().enumerate() {
    if let Some(run_name) = opts.run_name.as_ref() {
      stage_opts.run_name = Some(format!("{run_name}/{}", stage.name()));
    }

    eprintln!(
      "training stage {}/{}: {}",
      stage_index + 1,
      TRAINING_STAGES_IN_ORDER.len(),
      stage.name()
    );

    let report = train_stage_inner(stage, stage_opts.clone(), viewer.as_ref())?;
    if let Some(checkpoint) = report.last_checkpoint.as_ref() {
      stage_opts.model_path = Some(checkpoint.join("model.safetensors"));
    }
    reports.push(report);
  }

  Ok(reports)
}

pub fn train_single_stage(stage: TrainingStage, opts: TrainOptions) -> TrainResult<TrainingReport> {
  train_stage(stage, opts)
}

pub fn evaluate_latest_checkpoint(
  stage: TrainingStage,
  opts: EvaluationOptions,
) -> TrainResult<EvaluationReport> {
  let loader = ModelLoader::new(evaluation_checkpoint_dir(stage, &opts));
  let checkpoint = loader
    .latest_checkpoint()?
    .ok_or_else(|| format!("no checkpoint found for stage {}", stage.name()))?;
  let (trainer, metadata) = loader.load_checkpoint(checkpoint, opts.device)?;

  let mut config = WorldConfig::division_b();
  config.robots_per_team = metadata.stage.robots_per_team();
  let mut engine = SimulationEngine::new(metadata.worlds, config);
  let (mut sim_states, mut states) = reset_stage_worlds(&mut engine, metadata.stage, 0);
  let avg_initial_ball_x = average_ball_x(&sim_states);
  let mut touched = vec![false; metadata.worlds];

  for _ in 0..opts.steps {
    let batch = GameState::encode_multiple(&states, trainer.dev);
    let (_, _, _, plans) = tch::no_grad(|| trainer.policy.act(&batch, true));
    let commands = plans
      .iter()
      .enumerate()
      .map(|(world, ai_commands)| {
        world_command_from_ai(metadata.stage, &sim_states[world], *ai_commands)
      })
      .collect::<Vec<_>>();

    sim_states = engine.step_with_commands(&commands);

    for (world, state) in sim_states.iter().enumerate() {
      if state.blue_robots.iter().any(|robot| robot.infrared) {
        touched[world] = true;
      }
    }

    states = sim_states
      .iter()
      .map(|state| game_state_from_sim(metadata.stage, state))
      .collect();
  }

  let contact_worlds = touched.iter().filter(|&&value| value).count();
  let avg_final_ball_distance =
    sim_states.iter().map(final_ball_distance).sum::<f64>() / metadata.worlds.max(1) as f64;
  let avg_final_ball_x = average_ball_x(&sim_states);
  let blue_goals = sim_states.iter().filter(|state| state.goal_blue).count();
  let yellow_goals = sim_states.iter().filter(|state| state.goal_yellow).count();

  Ok(EvaluationReport {
    stage: metadata.stage,
    worlds: metadata.worlds,
    steps: opts.steps,
    contact_worlds,
    contact_rate: contact_worlds as f64 / metadata.worlds.max(1) as f64,
    avg_final_ball_distance,
    avg_initial_ball_x,
    avg_final_ball_x,
    avg_ball_x_progress: avg_final_ball_x - avg_initial_ball_x,
    blue_goals,
    yellow_goals,
  })
}

fn train_stage(stage: TrainingStage, opts: TrainOptions) -> TrainResult<TrainingReport> {
  let viewer = TrainingViewer::spawn(&opts)?;
  train_stage_inner(stage, opts, viewer.as_ref())
}

fn train_stage_inner(
  stage: TrainingStage,
  opts: TrainOptions,
  viewer: Option<&TrainingViewer>,
) -> TrainResult<TrainingReport> {
  let mut config = WorldConfig::division_b();
  config.robots_per_team = stage.robots_per_team();
  let dt = config.physics.delta_time as f32;

  let mut engine = SimulationEngine::new(opts.worlds, config);
  let mut trainer = build_trainer(stage, &opts)?;
  let loader = ModelLoader::new(run_checkpoint_dir(stage, &opts));
  let mut sumatra_runtime = if stage == TrainingStage::SumatraOpponent {
    Some(SumatraOpponentRuntime::spawn(&opts)?)
  } else {
    None
  };

  let (mut sim_states, mut states) = reset_stage_worlds(&mut engine, stage, 0);
  if let Some(viewer) = viewer {
    viewer.publish(&sim_states);
  }
  trainer.sync_states(sim_states.clone(), states.clone());

  let mut last_stats = None;
  let mut last_checkpoint = None;
  let progress = TrainingProgress::new(stage, &opts)?;

  for update in 0..opts.updates {
    progress.start_update(update);

    for rollout_step in 0..opts.rollout_steps {
      trainer.step(&states, dt);

      let commands = (0..opts.worlds)
        .map(|world| {
          let ai_commands = trainer.commands_for_world(world);
          world_command_from_ai(stage, &sim_states[world], ai_commands)
        })
        .collect::<Vec<_>>();

      sim_states = if let Some(runtime) = sumatra_runtime.as_mut() {
        runtime.step(&mut engine, &commands)?
      } else {
        engine.step_with_commands(&commands)
      };

      states = sim_states
        .iter()
        .map(|state| game_state_from_sim(stage, state))
        .collect();
      if let Some(viewer) = viewer {
        viewer.publish(&sim_states);
      }
      trainer.finish_step(sim_states.clone(), states.clone());
      progress.finish_rollout_step(rollout_step + 1);
    }

    let stats = trainer.finish_rollout(&states);
    last_stats = Some(stats);

    if should_checkpoint(update + 1, &opts) {
      let metadata =
        CheckpointMetadata::from_training(stage, update + 1, &opts, &trainer, Some(stats));
      last_checkpoint = Some(loader.save_checkpoint(&trainer, &metadata)?);
    }
    progress.finish_update(update + 1, stats, last_checkpoint.as_ref());

    let reset_seed = update + 1;
    let reset = reset_stage_worlds(&mut engine, stage, reset_seed);
    sim_states = reset.0;
    states = reset.1;
    if let Some(viewer) = viewer {
      viewer.publish(&sim_states);
    }
    trainer.sync_states(sim_states.clone(), states.clone());
    if let Some(runtime) = sumatra_runtime.as_mut() {
      runtime.reset_tracking();
    }
  }

  if opts.updates > 0 && !should_checkpoint(opts.updates, &opts) {
    if let Some(stats) = last_stats {
      let metadata =
        CheckpointMetadata::from_training(stage, opts.updates, &opts, &trainer, Some(stats));
      last_checkpoint = Some(loader.save_checkpoint(&trainer, &metadata)?);
      progress.saved_final_checkpoint(last_checkpoint.as_ref());
    }
  }

  progress.finish_stage(last_stats);

  let stats = last_stats;
  Ok(TrainingReport {
    stage,
    updates: opts.updates,
    worlds: opts.worlds,
    rollout_steps: opts.rollout_steps,
    last_checkpoint,
    last_loss: stats.map(|s| s.loss),
    last_policy_loss: stats.map(|s| s.policy_loss),
    last_value_loss: stats.map(|s| s.value_loss),
    last_entropy: stats.map(|s| s.entropy),
  })
}

struct TrainingViewer {
  server: ViewerServer,
}

impl TrainingViewer {
  fn spawn(opts: &TrainOptions) -> TrainResult<Option<Self>> {
    if !opts.viewer {
      return Ok(None);
    }

    let mut config = ViewerConfig::default();
    if let Some(port) = opts.viewer_port {
      config.http_port = port;
    }

    let world_config = WorldConfig::division_b();
    let server = ViewerServer::bind(config, opts.worlds, &world_config)?;
    eprintln!("viewer: {}", config.http_url());

    Ok(Some(Self { server }))
  }

  fn publish(&self, states: &[WorldState]) {
    self.server.publish_states(states);
  }
}

struct TrainingProgress {
  _multi: MultiProgress,
  updates: ProgressBar,
  rollout: ProgressBar,
  total_updates: usize,
  total_rollout_steps: usize,
}

impl TrainingProgress {
  fn new(stage: TrainingStage, opts: &TrainOptions) -> TrainResult<Self> {
    let multi = MultiProgress::new();
    let updates = multi.add(ProgressBar::new(opts.updates as u64));
    let rollout = multi.add(ProgressBar::new(opts.rollout_steps as u64));

    let update_style =
      ProgressStyle::with_template("{prefix:.bold} {bar:40.green/black} {pos:>4}/{len:<4} {msg}")?
        .progress_chars("=> ");
    let rollout_style =
      ProgressStyle::with_template("{prefix:.bold} {bar:40.cyan/black} {pos:>4}/{len:<4} {msg}")?
        .progress_chars("=> ");

    updates.set_style(update_style);
    updates.set_prefix(format!("stage {}", stage.name()));
    updates.set_message(format!(
      "{} worlds, {} rollout steps/update",
      opts.worlds, opts.rollout_steps
    ));

    rollout.set_style(rollout_style);
    rollout.set_prefix("rollout");
    rollout.set_message("waiting for first update");

    Ok(Self {
      _multi: multi,
      updates,
      rollout,
      total_updates: opts.updates,
      total_rollout_steps: opts.rollout_steps,
    })
  }

  fn start_update(&self, update: usize) {
    self.updates.set_position(update as u64);
    self
      .updates
      .set_message(format!("update {}/{}", update + 1, self.total_updates));
    self.rollout.reset();
    self.rollout.set_message(format!(
      "collecting rollout for update {}/{}",
      update + 1,
      self.total_updates
    ));
  }

  fn finish_rollout_step(&self, step: usize) {
    self.rollout.set_position(step as u64);
    self
      .rollout
      .set_message(format!("step {}/{}", step, self.total_rollout_steps));
  }

  fn finish_update(&self, update: usize, stats: UpdateResult, checkpoint: Option<&PathBuf>) {
    self.rollout.set_position(self.total_rollout_steps as u64);
    self
      .rollout
      .set_message("rollout complete; PPO update complete");
    self.updates.set_position(update as u64);

    let checkpoint_msg = checkpoint
      .and_then(|path| path.file_name())
      .and_then(|name| name.to_str())
      .map(|name| format!(", checkpoint {name}"))
      .unwrap_or_default();

    self.updates.set_message(format!(
      "update {}/{} loss {:.4} policy {:.4} value {:.4} entropy {:.4}{}",
      update,
      self.total_updates,
      stats.loss,
      stats.policy_loss,
      stats.value_loss,
      stats.entropy,
      checkpoint_msg
    ));
  }

  fn saved_final_checkpoint(&self, checkpoint: Option<&PathBuf>) {
    if let Some(checkpoint) = checkpoint {
      self
        .updates
        .set_message(format!("saved final checkpoint {}", checkpoint.display()));
    }
  }

  fn finish_stage(&self, stats: Option<UpdateResult>) {
    self.rollout.finish_and_clear();

    if let Some(stats) = stats {
      self.updates.finish_with_message(format!(
        "done loss {:.4} policy {:.4} value {:.4} entropy {:.4}",
        stats.loss, stats.policy_loss, stats.value_loss, stats.entropy
      ));
    } else {
      self.updates.finish_with_message("done without updates");
    }
  }
}

fn build_trainer(stage: TrainingStage, opts: &TrainOptions) -> TrainResult<Trainer> {
  let vs = VarStore::new(opts.device);
  let mut trainer =
    Trainer::new_with_reward(vs, opts.worlds, opts.learning_rate, stage.reward_mode());

  if let Some(model_path) = opts.model_path.as_ref() {
    trainer.vs.load(model_path)?;
  }

  Ok(trainer)
}

fn should_checkpoint(update: usize, opts: &TrainOptions) -> bool {
  opts.checkpoint_every > 0 && update % opts.checkpoint_every == 0
}

fn run_checkpoint_dir(stage: TrainingStage, opts: &TrainOptions) -> PathBuf {
  match opts.run_name.as_ref() {
    Some(name) => opts.checkpoint_dir.join(name),
    None => opts.checkpoint_dir.join(stage.name()),
  }
}

fn evaluation_checkpoint_dir(stage: TrainingStage, opts: &EvaluationOptions) -> PathBuf {
  match opts.run_name.as_ref() {
    Some(name) => opts.checkpoint_dir.join(name),
    None => opts.checkpoint_dir.join(stage.name()),
  }
}

#[cfg(feature = "sumatra-opponent")]
struct SumatraOpponentRuntime {
  servers: Vec<SumatraSimNetServer>,
  clients: Vec<SumatraInstance>,
}

#[cfg(feature = "sumatra-opponent")]
impl SumatraOpponentRuntime {
  fn spawn(opts: &TrainOptions) -> TrainResult<Self> {
    let mut servers = Vec::with_capacity(opts.worlds);
    let mut clients = Vec::with_capacity(opts.worlds);

    for world in 0..opts.worlds {
      let port = sumatra_port(opts.sumatra_base_port, world)?;
      let server = SumatraSimNetServer::bind_for_world(
        SumatraSimNetConfig {
          bind_addr: format!("127.0.0.1:{port}").parse()?,
        },
        world,
      )?;
      let mut launch_config = SumatraLaunchConfig {
        remote_client: true,
        host: Some("127.0.0.1".to_string()),
        sim_net_port: Some(port),
        ..SumatraLaunchConfig::default()
      };
      if let Some(repo_root) = opts.sumatra_repo_root.as_ref() {
        launch_config.repo_root = repo_root.clone();
      }
      let client = SumatraInstance::spawn(&launch_config)?;
      servers.push(server);
      clients.push(client);
    }

    let mut runtime = Self { servers, clients };
    runtime.wait_for_clients()?;
    Ok(runtime)
  }

  fn wait_for_clients(&mut self) -> TrainResult<()> {
    let timeout = std::time::Duration::from_secs(30);
    let start = std::time::Instant::now();
    let mut connected = vec![false; self.servers.len()];

    loop {
      for (world, server) in self.servers.iter_mut().enumerate() {
        if !connected[world] {
          connected[world] = server.has_clients()?;
        }
        if let Some(status) = self.clients[world].try_wait()? {
          return Err(
            format!("Sumatra client for world {world} exited early with {status}").into(),
          );
        }
      }

      if connected.iter().all(|value| *value) {
        return Ok(());
      }
      if start.elapsed() >= timeout {
        let missing = connected.iter().filter(|connected| !**connected).count();
        return Err(
          format!(
            "timed out waiting for {missing} Sumatra client(s) to connect after {}s",
            timeout.as_secs()
          )
          .into(),
        );
      }

      std::thread::sleep(std::time::Duration::from_millis(20));
    }
  }

  fn step(
    &mut self,
    engine: &mut SimulationEngine,
    local_commands: &[WorldCommand],
  ) -> TrainResult<Vec<WorldState>> {
    for (world, server) in self.servers.iter_mut().enumerate() {
      server.step_with_local_commands(engine, local_commands)?;
      if let Some(status) = self.clients[world].try_wait()? {
        return Err(format!("Sumatra client for world {world} exited early with {status}").into());
      }
    }

    Ok(engine.get_all_states())
  }

  fn reset_tracking(&mut self) {
    for server in &mut self.servers {
      server.reset_tracking();
    }
  }
}

#[cfg(feature = "sumatra-opponent")]
fn sumatra_port(base_port: u16, world: usize) -> TrainResult<u16> {
  let offset =
    u16::try_from(world).map_err(|_| format!("world index {world} does not fit in a u16 port"))?;
  base_port
    .checked_add(offset)
    .ok_or_else(|| format!("sumatra port range overflows u16 at world {world}").into())
}

#[cfg(not(feature = "sumatra-opponent"))]
struct SumatraOpponentRuntime;

#[cfg(not(feature = "sumatra-opponent"))]
impl SumatraOpponentRuntime {
  fn spawn(_opts: &TrainOptions) -> TrainResult<Self> {
    Err(
      "sumatra_opponent training requires the artificial_incompetence `sumatra-opponent` feature"
        .into(),
    )
  }

  fn step(
    &mut self,
    _engine: &mut SimulationEngine,
    _local_commands: &[WorldCommand],
  ) -> TrainResult<Vec<WorldState>> {
    Err("sumatra_opponent runtime was not started".into())
  }

  fn reset_tracking(&mut self) {}
}

fn reset_stage_worlds(
  engine: &mut SimulationEngine,
  stage: TrainingStage,
  reset_seed: usize,
) -> (Vec<WorldState>, Vec<GameState>) {
  engine.reset_all();
  let commands = (0..engine.count())
    .map(|world| stage_reset_command(stage, world, reset_seed))
    .collect::<Vec<_>>();
  let sim_states = engine.step_with_commands(&commands);
  let states = sim_states
    .iter()
    .map(|state| game_state_from_sim(stage, state))
    .collect();
  (sim_states, states)
}

fn stage_reset_command(stage: TrainingStage, world: usize, reset_seed: usize) -> WorldCommand {
  let offset = deterministic_offset(world, reset_seed);
  let mut command = WorldCommand::default();

  match stage {
    TrainingStage::TouchBall => {
      command.teleport_ball = Some(ball_at(-1.0 + offset.0 * 0.4, offset.1 * 0.8));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Blue, 0, -2.6, offset.1, 0.0, true));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Yellow, 0, 0.0, 0.0, PI, false));
    }
    TrainingStage::DribbleToGoal => {
      command.teleport_ball = Some(ball_at(-2.2, offset.1 * 0.7));
      command.teleport_robots.push(robot_at(
        TeamColor::Blue,
        0,
        -2.35,
        offset.1 * 0.7,
        0.0,
        true,
      ));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Yellow, 0, 0.0, 0.0, PI, false));
    }
    TrainingStage::ShootGoal => {
      let y = offset.1 * 1.2;
      command.teleport_ball = Some(ball_at(1.0 + offset.0 * 0.8, y));
      command.teleport_robots.push(robot_at(
        TeamColor::Blue,
        0,
        0.88 + offset.0 * 0.8,
        y,
        0.0,
        true,
      ));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Yellow, 0, 0.0, 0.0, PI, false));
    }
    TrainingStage::PassReceive => {
      let y = offset.1 * 0.8;
      command.teleport_ball = Some(ball_at(-2.3, y));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Blue, 0, -2.45, y, 0.0, true));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Blue, 1, -0.6, -y, 0.0, true));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Yellow, 0, 0.0, 0.0, PI, false));
      command
        .teleport_robots
        .push(robot_at(TeamColor::Yellow, 1, 0.0, 0.0, PI, false));
    }
    TrainingStage::OneVsOne => {
      command.teleport_ball = Some(ball_at(-1.8, offset.1 * 0.9));
      command.teleport_robots.push(robot_at(
        TeamColor::Blue,
        0,
        -2.1,
        offset.1 * 0.9,
        0.0,
        true,
      ));
      command.teleport_robots.push(robot_at(
        TeamColor::Yellow,
        0,
        -0.2,
        -offset.1 * 0.7,
        PI,
        true,
      ));
    }
    TrainingStage::ScriptedScrimmage | TrainingStage::SumatraOpponent => {
      command.teleport_ball = Some(ball_at(-1.0 + offset.0, offset.1));
      for id in 0..stage.robots_per_team() {
        let lane = id as f64 - (stage.robots_per_team() as f64 - 1.0) * 0.5;
        command.teleport_robots.push(robot_at(
          TeamColor::Blue,
          id,
          -2.8 + (id % 2) as f64 * 0.8,
          lane * 0.7,
          0.0,
          true,
        ));
        command.teleport_robots.push(robot_at(
          TeamColor::Yellow,
          id,
          1.2 + (id % 2) as f64 * 0.8,
          -lane * 0.7,
          PI,
          stage == TrainingStage::SumatraOpponent || id < 3,
        ));
      }
    }
  }

  command
}

fn deterministic_offset(world: usize, reset_seed: usize) -> (f64, f64) {
  let x = ((world * 37 + reset_seed * 17) % 101) as f64 / 50.0 - 1.0;
  let y = ((world * 53 + reset_seed * 29) % 101) as f64 / 50.0 - 1.0;
  (x, y)
}

fn ball_at(x: f64, y: f64) -> TeleportBall {
  TeleportBall {
    x: Some(x),
    y: Some(y),
    z: Some(0.0),
    vx: Some(0.0),
    vy: Some(0.0),
    vz: Some(0.0),
  }
}

fn robot_at(
  team: TeamColor,
  id: usize,
  x: f64,
  y: f64,
  orientation: f64,
  present: bool,
) -> TeleportRobot {
  TeleportRobot {
    id,
    team,
    x: Some(x),
    y: Some(y),
    orientation: Some(orientation),
    vx: Some(0.0),
    vy: Some(0.0),
    v_angular: Some(0.0),
    present: Some(present),
  }
}

fn game_state_from_sim(stage: TrainingStage, state: &WorldState) -> GameState {
  let mut own_robots = [None; 16];
  let mut opp_robots = [None; 16];
  let has_goalie = matches!(
    stage,
    TrainingStage::ScriptedScrimmage | TrainingStage::SumatraOpponent
  );

  for robot in &state.blue_robots {
    if robot.id < own_robots.len() && robot.is_on {
      own_robots[robot.id] = Some(robot_state_from_sim(robot, has_goalie && robot.id == 0));
    }
  }

  for robot in &state.yellow_robots {
    if robot.id < opp_robots.len() && robot.is_on {
      opp_robots[robot.id] = Some(robot_state_from_sim(robot, has_goalie && robot.id == 0));
    }
  }

  GameState {
    own_robots,
    opp_robots,
    ball: BallState {
      pos: Vec2::new(state.ball.x as f32, state.ball.y as f32),
      vel: Vec2::new(state.ball.vx as f32, state.ball.vy as f32),
      stop_pos: Vec2::new(state.ball.x as f32, state.ball.y as f32),
      stop_time: 0.0,
    },
  }
}

fn robot_state_from_sim(robot: &simhark::RobotState, is_goalie: bool) -> RobotState {
  RobotState {
    id: robot.id as u8,
    pos: Vec2::new(robot.x as f32, robot.y as f32),
    vel: Vec2::new(robot.vx as f32, robot.vy as f32),
    heading: robot.orientation as f32,
    angular_vel: robot.v_angular as f32,
    is_goalie,
  }
}

fn world_command_from_ai(
  stage: TrainingStage,
  state: &WorldState,
  commands: Commands,
) -> WorldCommand {
  let mut blue = Vec::new();
  for robot in &state.blue_robots {
    if !robot.is_on {
      continue;
    }

    let command = commands
      .get(robot.id)
      .copied()
      .flatten()
      .unwrap_or(RobotCommand::Hold);
    blue.push(sim_robot_command(stage, state, robot, command));
  }

  let yellow = if stage == TrainingStage::OneVsOne || stage == TrainingStage::ScriptedScrimmage {
    scripted_opponent_commands(stage, state)
  } else {
    Vec::new()
  };

  WorldCommand {
    blue,
    yellow,
    teleport_ball: None,
    teleport_robots: Vec::new(),
  }
}

fn sim_robot_command(
  stage: TrainingStage,
  state: &WorldState,
  robot: &simhark::RobotState,
  command: RobotCommand,
) -> simhark::RobotCommand {
  let mut target = None;
  let mut dribbler_on = false;
  let mut kick_speed = 0.0;
  let mut kick_angle = 0.0;

  match command {
    RobotCommand::Pos(pos) => target = Some((pos.x as f64, pos.y as f64)),
    RobotCommand::Dribble(pos) | RobotCommand::PosBall(pos) => {
      target = Some((pos.x as f64, pos.y as f64));
      dribbler_on = true;
    }
    RobotCommand::Steal | RobotCommand::RecPass => {
      target = Some((state.ball.x, state.ball.y));
      dribbler_on = true;
    }
    RobotCommand::Kick(power)
    | RobotCommand::RecKick(power)
    | RobotCommand::Kickoff(power)
    | RobotCommand::FreeKick(power) => {
      kick_speed = scaled_kick_speed(power);
      target = Some((state.ball.x, state.ball.y));
    }
    RobotCommand::Chip(power) => {
      kick_speed = scaled_kick_speed(power);
      kick_angle = 35.0;
      target = Some((state.ball.x, state.ball.y));
    }
    RobotCommand::KickGoal => {
      kick_speed = 6.0;
      target = Some((4.5, 0.0));
    }
    RobotCommand::PassTo(id) => {
      kick_speed = 4.0;
      target = state
        .blue_robots
        .iter()
        .find(|r| r.id == id as usize && r.is_on)
        .map(|r| (r.x, r.y))
        .or(Some((4.5, 0.0)));
    }
    RobotCommand::GoalWall => target = Some((-3.8, state.ball.y.clamp(-0.8, 0.8))),
    RobotCommand::GoalieGuard => target = Some((-4.0, state.ball.y.clamp(-0.45, 0.45))),
    RobotCommand::Hold => {}
  }

  if matches!(
    stage,
    TrainingStage::TouchBall | TrainingStage::DribbleToGoal
  ) && kick_speed == 0.0
  {
    dribbler_on = true;
  }

  let move_command = target.map(|target| move_to(robot, target));

  simhark::RobotCommand {
    id: robot.id,
    move_command,
    kick_speed,
    kick_angle,
    dribbler_on,
  }
}

fn scripted_opponent_commands(
  stage: TrainingStage,
  state: &WorldState,
) -> Vec<simhark::RobotCommand> {
  state
    .yellow_robots
    .iter()
    .filter(|robot| robot.is_on)
    .map(|robot| {
      let target = if stage == TrainingStage::ScriptedScrimmage && robot.id == 0 {
        (-4.0, state.ball.y.clamp(-0.5, 0.5))
      } else {
        (state.ball.x, state.ball.y)
      };

      simhark::RobotCommand {
        id: robot.id,
        move_command: Some(move_to(robot, target)),
        kick_speed: if near(robot.x, robot.y, state.ball.x, state.ball.y, 0.18) {
          5.0
        } else {
          0.0
        },
        kick_angle: 0.0,
        dribbler_on: true,
      }
    })
    .collect()
}

fn move_to(robot: &simhark::RobotState, target: (f64, f64)) -> MoveCommand {
  let dx = target.0 - robot.x;
  let dy = target.1 - robot.y;
  let dist = dx.hypot(dy).max(1e-6);
  let speed = (dist * 3.0).clamp(0.0, 2.5);
  let vx = dx / dist * speed;
  let vy = dy / dist * speed;
  let desired_heading = dy.atan2(dx);
  let heading_err = wrap_angle(desired_heading - robot.orientation);

  MoveCommand::GlobalVelocity {
    vx,
    vy,
    angular: (heading_err * 6.0).clamp(-8.0, 8.0),
  }
}

fn scaled_kick_speed(power: f32) -> f64 {
  (1.0 + power as f64 * 5.0).clamp(1.0, 6.0)
}

fn near(ax: f64, ay: f64, bx: f64, by: f64, threshold: f64) -> bool {
  (ax - bx).hypot(ay - by) <= threshold
}

fn final_ball_distance(state: &WorldState) -> f64 {
  state
    .blue_robots
    .iter()
    .filter(|robot| robot.is_on)
    .map(|robot| (robot.x - state.ball.x).hypot(robot.y - state.ball.y))
    .fold(f64::INFINITY, f64::min)
}

fn average_ball_x(states: &[WorldState]) -> f64 {
  states.iter().map(|state| state.ball.x).sum::<f64>() / states.len().max(1) as f64
}

fn wrap_angle(mut angle: f64) -> f64 {
  while angle > PI {
    angle -= 2.0 * PI;
  }
  while angle < -PI {
    angle += 2.0 * PI;
  }
  angle
}
