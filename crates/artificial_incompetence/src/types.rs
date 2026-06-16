use core_dump::vec::types::Vec2;

pub struct BallState {
  pub pos: Vec2<f32>,
  pub vel: Vec2<f32>,
  pub stop_pos: Vec2<f32>,
  pub stop_time: f32,
}

#[derive(Clone, Copy, Default)]
pub struct RobotState {
  pub id: u8,
  pub pos: Vec2<f32>,
  pub vel: Vec2<f32>,
  pub heading: f32,
  pub angular_vel: f32,
  pub is_goalie: bool,
}

pub type Robots = [Option<RobotState>; 8];

pub struct GameState {
  pub own_robots: Robots,
  pub opp_robots: Robots,
  pub ball: BallState,
}

#[derive(Debug, Copy, Clone, Default PartialEq)]
pub enum RobotCommand {
  Pos(Vec2<f32>),
  Kick(f32),
  Chip(f32),
  RecKick(f32),
  Steal,
  Dribble(Vec2<f32>),
  PosBall(Vec2<f32>),
  Kickoff(f32),
  FreeKick(f32),
  KickGoal,
  PassTo(u8),
  RecPass,
  GoalWall,
  GoalieGuard,
  #[default]
  Hold,
}

pub type Commands = [Option<RobotCommand>; 16];

pub struct ArtificialIncompetence {
  // internal state, if needed
}

impl ArtificialIncompetence {
  pub fn predict(&mut self, state: &GameState, dt: f32) -> Commands {
    todo!()
  }
}
