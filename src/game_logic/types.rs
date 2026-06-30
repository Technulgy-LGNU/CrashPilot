use crate::FieldSetup;
use core_dump::proto::referee::Command;
use core_dump::proto::{InterfaceCommandCp, Referee, TrackerWrapperPacket};
use core_dump::vec::types::Vec2;
use std::cmp::PartialEq;
use std::collections::HashMap;
use std::ops::Neg;

/// GameState, GamePhase, and RefMachine merged
/// Advanced Update functions
pub struct WorldState {
  // Data
  pub robots_self: Vec<Robot>,
  pub robots_opp: Vec<Robot>,
  pub ball: BallData,

  pub referee: Referee,
  pub iface_cmd: InterfaceCommandCp,

  pub team: Team,
  pub site: f32,

  // Thinks to keep track of
  pub goalie: Option<u8>,
  pub new_goalie: Option<u8>,
  #[cfg(feature = "ssl_game_controller")]
  pub last_requested_goalie: Option<u8>,
  pub defenders: Vec<u8>,
  pub acting_robot: Option<u8>,
  pub has_acted: bool,

  // States
  pub ref_machine: RefMachine,
  pub phase: GamePhase,
  pub prep_phase: PrepPhase,
  pub prep_task: Option<PrepTask>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum GamePhase {
  #[default]
  Unknown,

  Halted,
  Stopped,

  Running,
  Timeout,
  BallPlacement,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum PrepPhase {
  #[default]
  Unknown,

  OffensiveKickoff,
  DefensiveKickoff,

  OffensivePenalty,
  DefensivePenalty,

  OffensiveFreeKick,
  DefensiveFreeKick,
}

impl PrepPhase {
  #[inline]
  pub fn is_offensive(self) -> bool {
    matches!(
      self,
      PrepPhase::OffensiveKickoff | PrepPhase::OffensivePenalty | PrepPhase::OffensiveFreeKick
    )
  }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum PrepTaskStatus {
  #[default]
  Preparing,
  Ready,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrepTask {
  pub phase: PrepPhase,
  pub status: PrepTaskStatus,
  pub command_counter: u32,
  pub command_timestamp: u64,
  pub ball_pos: Vec2<f32>,
  pub acting_robot: Option<u8>,
  pub receiving_robot: Option<u8>,
  pub has_acted: bool,
  pub follow_up_acted: bool,
}

#[derive(Debug, Default, Clone)]
pub struct RefMachine {
  pub state: RefState,
  last_command: Option<Command>,
  last_command_counter: Option<u32>,
  last_command_timestamp: Option<u64>,
}

impl RefMachine {
  pub fn apply(&mut self, referee: &Referee) -> bool {
    let cmd = referee.command();
    let is_new_command = self.last_command != Some(cmd)
      || self.last_command_counter != Some(referee.command_counter)
      || self.last_command_timestamp != Some(referee.command_timestamp);

    if !is_new_command {
      return false;
    }

    self.last_command = Some(cmd);
    self.last_command_counter = Some(referee.command_counter);
    self.last_command_timestamp = Some(referee.command_timestamp);

    self.state = match cmd {
      Command::Halt => RefState::Halt,

      Command::Stop => RefState::Stop,

      Command::PrepareKickoffBlue => RefState::PrepareKickoff {
        attacking: Team::Blue,
      },

      Command::PrepareKickoffYellow => RefState::PrepareKickoff {
        attacking: Team::Yellow,
      },

      Command::NormalStart => RefState::Running,

      Command::ForceStart => RefState::Running,

      Command::PreparePenaltyYellow => RefState::PreparePenalty {
        attacking: Team::Yellow,
      },
      Command::PreparePenaltyBlue => RefState::PreparePenalty {
        attacking: Team::Blue,
      },
      Command::DirectFreeYellow => RefState::DirectFree {
        attacking: Team::Yellow,
      },
      Command::DirectFreeBlue => RefState::DirectFree {
        attacking: Team::Blue,
      },
      Command::IndirectFreeYellow => RefState::IndirectFree {
        attacking: Team::Yellow,
      },
      Command::IndirectFreeBlue => RefState::IndirectFree {
        attacking: Team::Blue,
      },
      Command::TimeoutYellow => RefState::Timeout,
      Command::TimeoutBlue => RefState::Timeout,
      Command::GoalYellow => RefState::Halt,
      Command::GoalBlue => RefState::Halt,
      Command::BallPlacementYellow => RefState::BallPlacement { team: Team::Yellow },
      Command::BallPlacementBlue => RefState::BallPlacement { team: Team::Blue },
    };

    true
  }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Team {
  // Dont differentiate between colors, so there need to be less checks
  Blue,
  Yellow,
}

impl Neg for Team {
  type Output = Self;
  fn neg(self) -> Self::Output {
    match self {
      Team::Blue => Team::Yellow,
      Team::Yellow => Team::Blue,
    }
  }
}

impl Team {
  #[inline]
  pub fn from_cp_team(team: i32) -> Option<Self> {
    match team {
      1 => Some(Team::Yellow),
      2 => Some(Team::Blue),
      _ => None,
    }
  }
}

#[derive(Debug, Copy, Default, Clone, Eq, PartialEq)]
pub enum RefState {
  #[default]
  Halt,
  Stop,

  PrepareKickoff {
    attacking: Team,
  },

  PreparePenalty {
    attacking: Team,
  },

  BallPlacement {
    team: Team,
  },

  DirectFree {
    attacking: Team,
  },

  IndirectFree {
    attacking: Team,
  },

  Running,
  Timeout,
}

impl WorldState {
  #[inline]
  #[warn(clippy::too_many_arguments)]
  pub fn update(
    &mut self,
    robots_self: Vec<Robot>,
    robots_opp: Vec<Robot>,
    ball: BallData,
    referee: Referee,
    iface_cmd: InterfaceCommandCp,
    team: Team,
    site: f32,
  ) {
    self.robots_self = robots_self;
    self.robots_opp = robots_opp;
    self.ball = ball;
    self.referee = referee;
    self.iface_cmd = iface_cmd;
    self.team = team;
    self.site = site;

    self.update_states();
  }

  #[inline]
  fn update_states(&mut self) {
    // Update RefMachine
    let is_new_ref_command = self.ref_machine.apply(&self.referee);
    if is_new_ref_command {
      self.update_prep_task();
    }

    // Apply goalie update
    #[cfg(feature = "ssl_game_controller")]
    if self.site.is_sign_positive() {
      self.goalie = Some(self.referee.yellow.goalkeeper as u8)
    } else {
      self.goalie = Some(self.referee.blue.goalkeeper as u8)
    }
    #[cfg(not(feature = "ssl_game_controller"))]
    if self.goalie != self.new_goalie {
      self.goalie = self.new_goalie;
    }

    // Apply those to GamePhase
    self.phase = match self.ref_machine.state {
      RefState::Halt => GamePhase::Halted,
      RefState::Stop => GamePhase::Stopped,
      RefState::PrepareKickoff { .. } => GamePhase::Stopped,
      RefState::PreparePenalty { .. } => GamePhase::Stopped,
      RefState::BallPlacement { team } => {
        if team == self.team {
          GamePhase::BallPlacement
        } else {
          GamePhase::Halted
        }
      }
      RefState::DirectFree { .. } => GamePhase::Stopped,
      RefState::IndirectFree { .. } => GamePhase::Stopped,
      RefState::Running => GamePhase::Running,
      RefState::Timeout => GamePhase::Timeout,
    };

    self.clear_finished_prep_task();
  }

  #[inline]
  fn update_prep_task(&mut self) {
    match self.ref_machine.state {
      RefState::PrepareKickoff { attacking } => {
        self.start_prep_task(
          self.prep_phase_for(attacking, RestartKind::Kickoff),
          PrepTaskStatus::Preparing,
        );
      }
      RefState::PreparePenalty { attacking } => {
        self.start_prep_task(
          self.prep_phase_for(attacking, RestartKind::Penalty),
          PrepTaskStatus::Preparing,
        );
      }
      RefState::DirectFree { attacking } | RefState::IndirectFree { attacking } => {
        self.start_prep_task(
          self.prep_phase_for(attacking, RestartKind::FreeKick),
          PrepTaskStatus::Ready,
        );
      }
      RefState::Running => {
        if let Some(task) = self.prep_task.as_mut()
          && task.status == PrepTaskStatus::Preparing
        {
          task.status = PrepTaskStatus::Ready;
        }
      }
      RefState::Halt | RefState::Stop | RefState::BallPlacement { .. } | RefState::Timeout => {
        self.clear_prep_task();
      }
    }
  }

  #[inline]
  fn start_prep_task(&mut self, phase: PrepPhase, status: PrepTaskStatus) {
    self.prep_phase = phase;
    self.acting_robot = None;
    self.has_acted = false;
    self.prep_task = Some(PrepTask {
      phase,
      status,
      command_counter: self.referee.command_counter,
      command_timestamp: self.referee.command_timestamp,
      ball_pos: self.ball.ball.pos,
      acting_robot: None,
      receiving_robot: None,
      has_acted: false,
      follow_up_acted: false,
    });
  }

  #[inline]
  pub fn mark_prep_actor(&mut self, robot_id: u8) {
    self.acting_robot = Some(robot_id);
    if let Some(task) = self.prep_task.as_mut() {
      task.acting_robot = Some(robot_id);
    }
  }

  #[inline]
  pub fn mark_prep_receiver(&mut self, robot_id: u8) {
    if let Some(task) = self.prep_task.as_mut() {
      task.receiving_robot = Some(robot_id);
    }
  }

  #[inline]
  pub fn mark_prep_acted(&mut self) {
    self.has_acted = true;
    if let Some(task) = self.prep_task.as_mut() {
      task.has_acted = true;
    }
  }

  #[inline]
  pub fn mark_prep_follow_up_acted(&mut self, ball_pos: Vec2<f32>) {
    if let Some(task) = self.prep_task.as_mut() {
      task.follow_up_acted = true;
      task.ball_pos = ball_pos;
    }
  }

  #[inline]
  pub fn clear_prep_task(&mut self) {
    self.prep_phase = PrepPhase::Unknown;
    self.prep_task = None;
    self.acting_robot = None;
    self.has_acted = false;
  }

  #[inline]
  fn clear_finished_prep_task(&mut self) {
    const BALL_MOVED_DISTANCE_MM: f32 = 50.0;

    let Some(task) = self.prep_task else {
      return;
    };

    if task.status != PrepTaskStatus::Ready {
      return;
    }

    let ball_delta = self.ball.ball.pos - task.ball_pos;
    if matches!(
      task.phase,
      PrepPhase::OffensiveKickoff | PrepPhase::OffensiveFreeKick
    ) && task.has_acted
      && !task.follow_up_acted
    {
      return;
    }

    if ball_delta.norm_squared() >= BALL_MOVED_DISTANCE_MM * BALL_MOVED_DISTANCE_MM {
      self.clear_prep_task();
    }
  }

  #[inline]
  fn prep_phase_for(&self, attacking: Team, kind: RestartKind) -> PrepPhase {
    match (attacking == self.team, kind) {
      (true, RestartKind::Kickoff) => PrepPhase::OffensiveKickoff,
      (false, RestartKind::Kickoff) => PrepPhase::DefensiveKickoff,
      (true, RestartKind::Penalty) => PrepPhase::OffensivePenalty,
      (false, RestartKind::Penalty) => PrepPhase::DefensivePenalty,
      (true, RestartKind::FreeKick) => PrepPhase::OffensiveFreeKick,
      (false, RestartKind::FreeKick) => PrepPhase::DefensiveFreeKick,
    }
  }
}

#[derive(Clone, Copy)]
enum RestartKind {
  Kickoff,
  Penalty,
  FreeKick,
}

impl Default for WorldState {
  fn default() -> Self {
    Self {
      robots_self: vec![],
      robots_opp: vec![],
      ball: Default::default(),
      referee: Default::default(),
      iface_cmd: Default::default(),
      team: Team::Yellow,
      site: 0f32,
      goalie: None,
      new_goalie: None,
      #[cfg(feature = "ssl_game_controller")]
      last_requested_goalie: None,
      defenders: vec![],
      ref_machine: Default::default(),
      phase: Default::default(),
      prep_phase: Default::default(),
      prep_task: None,
      acting_robot: None,
      has_acted: false,
    }
  }
}

/// This a single robot.
/// There are functions to create them from tracked
/// and raw visions. Tracked Vision is highly recommended
///
/// It's already converted to our own vector type,
/// which makes it way easier to do math with it
///
/// Everything is in:
///   - mm && mm/s
///   - degree && degree/s
#[derive(Clone, Debug)]
pub struct Robot {
  pub robot_id: u8,

  pub pos: Option<Vec2<f32>>,
  pub vel: Option<Vec2<f32>>,

  pub orientation: f32,
  pub angular_vel: f32,
  pub team: Team,

  // Distances to each robot
  pub distance_team: HashMap<u8, f32>,
  pub _distance_opponent: HashMap<u8, f32>,

  // Distance to the ball & goal mid point
  pub distance_ball: Option<f32>,
  pub _distance_goal: Option<f32>,

  /// Distance to each wall in the following order:
  ///  - 0: X+ Wall
  ///  - 1: Y+ Wall
  ///  - 2: X- Wall
  ///  - 3: Y- Wall
  ///
  /// This corresponds to the angle in each direction
  pub _distance_wall: Option<Vec<f32>>,
}

impl Robot {
  /// Converts robots from the tracked vision to our own robot type
  #[inline]
  pub fn new_from_tracked(
    vis_tracked: &TrackerWrapperPacket,
    ball: &Ball,
    team: i32,
    field_setup: &FieldSetup,
    site: f32,
  ) -> (Vec<Robot>, Vec<Robot>) {
    let robots_tracked = vis_tracked.tracked_frame.clone().unwrap_or_default().robots;
    if robots_tracked.is_empty() {
      return (vec![], vec![]);
    }
    let mut robots_self: Vec<Robot> = Vec::with_capacity(32);
    let mut robots_opp: Vec<Robot> = Vec::with_capacity(32);

    for robot in &robots_tracked {
      let goal_point = if robot.robot_id.team.unwrap_or_default() == team {
        Vec2::new(field_setup.width as f32 * site * 0.5, 0f32) // Midpoint of the field
      } else {
        Vec2::new(-(field_setup.width as f32 * 0.5 * site), 0f32) // Midpoint of the field
      };

      // Calculates the distances and puts them into the HashMap
      let mut dist_team: HashMap<u8, f32> = HashMap::new();
      let mut dist_opponent: HashMap<u8, f32> = HashMap::new();
      for robot_t in robots_tracked.clone() {
        let dist = Vec2::<f32>::new_from_cp(robot.pos)
          .scale_to(1000f32)
          .dot(&Vec2::new_from_cp(robot_t.pos).scale_to(1000f32))
          .sqrt();
        if team == robot_t.robot_id.team.unwrap_or_default()
          && !dist_team
            .iter()
            .any(|r| *r.0 == robot_t.robot_id.id.unwrap_or_default() as u8)
        {
          dist_team.insert(robot_t.robot_id.id.unwrap_or_default() as u8, dist);
        } else if !dist_opponent
          .iter()
          .any(|r| *r.0 == robot_t.robot_id.id.unwrap_or_default() as u8)
        {
          dist_opponent.insert(robot_t.robot_id.id.unwrap_or_default() as u8, dist);
        }
      }

      // Create the actual individual robot data
      if robot.robot_id.team.unwrap_or_default() == team {
        robots_self.push(Robot {
          robot_id: robot.robot_id.id.unwrap_or_default() as u8,
          pos: Some(Vec2::new(robot.pos.x, robot.pos.y).scale_to(1000f32)),
          vel: robot
            .vel
            .map(|vel| Vec2::new(vel.x, vel.y).scale_to(1000f32)),
          orientation: robot.orientation.to_degrees(),
          angular_vel: robot.vel_angular.unwrap_or_default().to_degrees(),
          team: Team::from_cp_team(robot.robot_id.team.unwrap_or_default()).unwrap_or(Team::Yellow),
          distance_team: dist_team,
          _distance_opponent: dist_opponent,
          distance_ball: Some(
            Vec2::new(robot.pos.x, robot.pos.y)
              .scale_to(1000f32)
              .dot(&ball.pos)
              .sqrt(),
          ),
          _distance_goal: Some(
            Vec2::new(robot.pos.x, robot.pos.y)
              .scale_to(1000f32)
              .dot(&goal_point)
              .sqrt(),
          ),
          _distance_wall: Option::from(create_wall_points(
            &Vec2::new_from_cp(robot.pos).scale_to(1000f32),
            field_setup,
          )),
        });
      } else {
        robots_opp.push(Robot {
          robot_id: robot.robot_id.id.unwrap_or_default() as u8,
          pos: Some(Vec2::new(robot.pos.x, robot.pos.y).scale_to(1000f32)),
          vel: robot
            .vel
            .map(|vel| Vec2::new(vel.x, vel.y).scale_to(1000f32)),
          orientation: robot.orientation.to_degrees(),
          angular_vel: robot.vel_angular.unwrap_or_default().to_degrees(),
          team: Team::from_cp_team(robot.robot_id.team.unwrap_or_default()).unwrap_or(Team::Yellow),
          distance_team: dist_team,
          _distance_opponent: dist_opponent,
          distance_ball: Some(
            Vec2::new(robot.pos.x, robot.pos.y)
              .scale_to(1000f32)
              .dot(&ball.pos)
              .sqrt(),
          ),
          _distance_goal: Some(
            Vec2::new(robot.pos.x, robot.pos.y)
              .scale_to(1000f32)
              .dot(&goal_point)
              .sqrt(),
          ),
          _distance_wall: Option::from(create_wall_points(
            &Vec2::new_from_cp(robot.pos).scale_to(1000f32),
            field_setup,
          )),
        });
      }
    }

    (robots_self, robots_opp)
  }
}

fn create_wall_points(robot_pos: &Vec2<f32>, field_setup: &FieldSetup) -> Vec<f32> {
  vec![
    robot_pos
      .dot(&Vec2::new(field_setup.width as f32 * 0.5, robot_pos.y))
      .sqrt(), // X+ Wall
    robot_pos
      .dot(&Vec2::new(robot_pos.x, field_setup.height as f32 * 0.5))
      .sqrt(), // Y+ Wall
    robot_pos
      .dot(&Vec2::new(field_setup.width as f32 * -0.5, robot_pos.y))
      .sqrt(), // X- Wall
    robot_pos
      .dot(&Vec2::new(robot_pos.x, field_setup.height as f32 * -0.5))
      .sqrt(), // Y- Wall
  ]
}

#[derive(Debug, Default)]
pub struct BallData {
  pub ball: Ball,
  pub kicked_ball: KickedBall,
}

#[derive(Debug, Default)]
pub struct Ball {
  pub pos: Vec2<f32>,
  pub vel: Vec2<f32>,
}

#[derive(Debug, Default)]
pub struct KickedBall {
  pub _pos: Vec2<f32>,
  pub _vel: Vec2<f32>,

  pub end_point: Option<Vec2<f32>>,
  pub end_time: Option<f32>,
}

impl BallData {
  #[inline]
  /// Takes either the tracked or raw vision and converts it to our own ball type
  /// The Test Field switcher is not implemented for this function for now
  pub fn new(vis_tracked: &TrackerWrapperPacket) -> Self {
    let frame = vis_tracked.tracked_frame.clone().unwrap_or_default();

    let mut ball = Ball::default();
    if !frame.balls.is_empty() {
      // Always uses the first Ball found in the TrackerFrame
      ball = Ball {
        pos: Vec2::new(frame.balls[0].pos.x, frame.balls[0].pos.y).scale_to(1000f32),
        vel: frame.balls[0]
          .vel
          .map(|vel| Vec2::new(vel.x, vel.y).scale_to(1000f32))
          .unwrap_or_default(),
      };
    }

    let kicked_ball = KickedBall {
      _pos: Vec2::new_from_cp(frame.kicked_ball.unwrap_or_default().pos).scale_to(1000f32),
      _vel: Vec2::new(
        frame.kicked_ball.unwrap_or_default().vel.x,
        frame.kicked_ball.unwrap_or_default().vel.y,
      )
      .scale_to(1000f32),
      end_point: frame
        .kicked_ball
        .unwrap_or_default()
        .stop_pos
        .map(|ep| Vec2::new(ep.x, ep.y).scale_to(1000f32)),
      end_time: Option::from(
        frame
          .kicked_ball
          .unwrap_or_default()
          .stop_timestamp
          .unwrap_or_default() as f32,
      ),
    };

    Self { ball, kicked_ball }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn referee(command: Command, counter: u32) -> Referee {
    Referee {
      command: command as i32,
      command_counter: counter,
      command_timestamp: counter as u64,
      ..Default::default()
    }
  }

  fn apply_referee(state: &mut WorldState, command: Command, counter: u32) {
    state.referee = referee(command, counter);
    state.update_states();
  }

  #[test]
  fn kickoff_prep_survives_normal_start_as_ready_task() {
    let mut state = WorldState {
      team: Team::Yellow,
      ..Default::default()
    };
    state.ball.ball.pos = Vec2::new(0.0, 0.0);

    apply_referee(&mut state, Command::PrepareKickoffYellow, 1);

    assert_eq!(state.phase, GamePhase::Stopped);
    let task = state.prep_task.expect("kickoff prep should be latched");
    assert_eq!(task.phase, PrepPhase::OffensiveKickoff);
    assert_eq!(task.status, PrepTaskStatus::Preparing);

    apply_referee(&mut state, Command::NormalStart, 2);

    assert_eq!(state.phase, GamePhase::Running);
    let task = state
      .prep_task
      .expect("normal start should not drop the kickoff task");
    assert_eq!(task.phase, PrepPhase::OffensiveKickoff);
    assert_eq!(task.status, PrepTaskStatus::Ready);
  }

  #[test]
  fn ready_penalty_prep_task_clears_after_ball_moves() {
    let mut state = WorldState {
      team: Team::Yellow,
      ..Default::default()
    };
    state.ball.ball.pos = Vec2::new(0.0, 0.0);

    apply_referee(&mut state, Command::PreparePenaltyYellow, 1);
    apply_referee(&mut state, Command::NormalStart, 2);

    state.ball.ball.pos = Vec2::new(60.0, 0.0);
    apply_referee(&mut state, Command::NormalStart, 2);

    assert_eq!(state.prep_phase, PrepPhase::Unknown);
    assert!(state.prep_task.is_none());
  }

  #[test]
  fn reset_command_clears_latched_prep_task() {
    let mut state = WorldState {
      team: Team::Yellow,
      ..Default::default()
    };

    apply_referee(&mut state, Command::PreparePenaltyYellow, 1);
    apply_referee(&mut state, Command::ForceStart, 2);
    assert!(state.prep_task.is_some());

    apply_referee(&mut state, Command::Stop, 3);

    assert_eq!(state.phase, GamePhase::Stopped);
    assert_eq!(state.prep_phase, PrepPhase::Unknown);
    assert!(state.prep_task.is_none());
  }

  #[test]
  fn direct_free_is_ready_immediately_and_uses_team_perspective() {
    let mut state = WorldState {
      team: Team::Blue,
      ..Default::default()
    };

    apply_referee(&mut state, Command::DirectFreeYellow, 1);

    let task = state.prep_task.expect("direct free should be latched");
    assert_eq!(state.phase, GamePhase::Stopped);
    assert_eq!(task.phase, PrepPhase::DefensiveFreeKick);
    assert_eq!(task.status, PrepTaskStatus::Ready);
  }

  #[test]
  fn offensive_free_kick_survives_pass_and_clears_after_follow_up_shot() {
    let mut state = WorldState {
      team: Team::Yellow,
      ..Default::default()
    };
    state.ball.ball.pos = Vec2::new(0.0, 0.0);

    apply_referee(&mut state, Command::DirectFreeYellow, 1);
    state.mark_prep_acted();

    state.ball.ball.pos = Vec2::new(60.0, 0.0);
    apply_referee(&mut state, Command::DirectFreeYellow, 1);

    let task = state
      .prep_task
      .expect("free kick should stay alive while pass is travelling");
    assert_eq!(task.phase, PrepPhase::OffensiveFreeKick);
    assert!(task.has_acted);
    assert!(!task.follow_up_acted);

    state.mark_prep_follow_up_acted(state.ball.ball.pos);
    state.ball.ball.pos = Vec2::new(120.0, 0.0);
    apply_referee(&mut state, Command::DirectFreeYellow, 1);

    assert_eq!(state.prep_phase, PrepPhase::Unknown);
    assert!(state.prep_task.is_none());
  }

  #[test]
  fn offensive_kickoff_survives_pass_until_receive_phase_finishes() {
    let mut state = WorldState {
      team: Team::Yellow,
      ..Default::default()
    };
    state.ball.ball.pos = Vec2::new(0.0, 0.0);

    apply_referee(&mut state, Command::PrepareKickoffYellow, 1);
    apply_referee(&mut state, Command::NormalStart, 2);
    state.mark_prep_acted();

    state.ball.ball.pos = Vec2::new(60.0, 0.0);
    apply_referee(&mut state, Command::NormalStart, 2);

    let task = state
      .prep_task
      .expect("kickoff should stay alive while pass is travelling");
    assert_eq!(task.phase, PrepPhase::OffensiveKickoff);
    assert!(task.has_acted);
    assert!(!task.follow_up_acted);

    state.mark_prep_follow_up_acted(state.ball.ball.pos);
    state.ball.ball.pos = Vec2::new(120.0, 0.0);
    apply_referee(&mut state, Command::NormalStart, 2);

    assert_eq!(state.prep_phase, PrepPhase::Unknown);
    assert!(state.prep_task.is_none());
  }
}
