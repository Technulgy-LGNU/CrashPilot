use std::cmp::PartialEq;
use std::collections::HashMap;

use crate::FieldSetup;
use core_dump::proto::referee::Command;
use core_dump::proto::{InterfaceCommandCp, Referee, TrackerWrapperPacket};
use core_dump::vec::types::Vec2;

/// GameState, GamePhase and RefMachine merged together with
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
  pub defenders: Vec<u8>,

  // States
  pub ref_machine: RefMachine,
  pub phase: GamePhase,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum GamePhase {
  #[default]
  UNKNOWN,

  Halted,
  Stopped,

  OffensiveKickoff,
  DefensiveKickoff,

  OffensivePenalty,
  DefensivePenalty,

  OffensiveFreeKick,
  DefensiveFreeKick,

  Running,
}

#[derive(Debug, Default, Clone)]
pub struct RefMachine {
  pub state: RefState,
}

impl RefMachine {
  pub fn apply(&mut self, cmd: Command) {
    match Command::try_from(cmd).unwrap_or_default() {
      Command::Halt => {
        self.state = RefState::Halt;
      }

      Command::Stop => {
        self.state = RefState::Stop;
      }

      Command::PrepareKickoffBlue => {
        self.state = RefState::PrepareKickoff {
          attacking: Team::Blue,
        };
      }

      Command::PrepareKickoffYellow => {
        self.state = RefState::PrepareKickoff {
          attacking: Team::Yellow,
        };
      }

      Command::NormalStart => {
        self.state = RefState::Running;
      }

      Command::ForceStart => {
        self.state = RefState::Running;
      }

      _ => {}
    }
  }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Team {
  // Dont differentiate between colors, so there need to be less checks
  Own,
  Enemy,
  Blue,
  Yellow,
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
}

impl WorldState {
  #[inline]
  pub fn update(
    &mut self,
    robots_self: Vec<Robot>,
    robots_opp: Vec<Robot>,
    ball: BallData,
    referee: Referee,
    iface_cmd: InterfaceCommandCp,
  ) {
    self.robots_self = robots_self;
    self.robots_opp = robots_opp;
    self.ball = ball;
    self.referee = referee;
    self.iface_cmd = iface_cmd;

    // Update team and site dependent on referee data
    // ToDo: Update site assignment
    if self.referee.yellow.name.to_string() == "" {
      self.site = 1f32
    } else {
      self.site = -1f32
    }

    self.update_states();
  }

  #[inline]
  fn update_states(&mut self) {
    // Update RefMachine
    self.ref_machine.apply(self.referee.command());

    // Apply those to GamePhase
    match RefState::try_from(self.ref_machine.state).unwrap_or_default() {
      RefState::Halt => {
        self.phase = GamePhase::Halted;
      }
      RefState::Stop => {
        self.phase = GamePhase::Stopped;
      }
      RefState::PrepareKickoff { attacking } => {}
      RefState::PreparePenalty { .. } => {}
      RefState::BallPlacement { .. } => {}
      RefState::DirectFree { .. } => {}
      RefState::IndirectFree { .. } => {}
      RefState::Running => {
        self.phase = GamePhase::Running;
      }
    }
    // For testing, just set always to running
    self.phase = GamePhase::Running;
  }
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
      defenders: vec![],
      ref_machine: Default::default(),
      phase: Default::default(),
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
///   - mm     &&     mm/s
///   - degree && degree/s
#[derive(Debug, Clone)]
pub struct Robot {
  pub robot_id: u8,
  // 0: Unknown
  // 1: Yellow
  // 2: Blue
  pub team: u8,

  pub pos: Option<Vec2<f32>>,
  pub vel: Option<Vec2<f32>>,

  pub orientation: f32,
  pub angular_vel: f32,

  // Distances to each robot
  pub distance_team: HashMap<u8, f32>,
  pub distance_opponent: HashMap<u8, f32>,

  // Distance to the ball & goal mid point
  pub distance_ball: Option<f32>,
  pub distance_goal: Option<f32>,

  /// Distance to each wall in following order:
  ///  - 0: X+ Wall
  ///  - 1: Y+ Wall
  ///  - 2: X- Wall
  ///  - 3: Y- Wall
  /// This corresponds to the angle in each direction
  pub distance_wall: Option<Vec<f32>>,
}

impl Robot {
  /// Converts robots from the tracked vision to our own robot type
  #[inline]
  pub fn new_from_tracked(
    vis_tracked: &TrackerWrapperPacket,
    ball: &Ball,
    team: i32,
    site: f32,
    field_setup: &FieldSetup,
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
        Vec2::new(field_setup.width as f32 * 0.5 * site * -1f32, 0f32) // Midpoint of the field
      };

      // Calculates the distances and puts them into the HashMap
      let mut dist_team: HashMap<u8, f32> = HashMap::new();
      let mut dist_opponent: HashMap<u8, f32> = HashMap::new();
      for robot_t in robots_tracked.clone() {
        let dist = Vec2::<f32>::new_from_cp(robot.pos)
          .dot(&Vec2::new_from_cp(robot_t.pos))
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
          team: robot.robot_id.team.unwrap_or_default() as u8,
          pos: Some(Vec2::new(robot.pos.x, robot.pos.y)),
          vel: robot.vel.map(|vel| Vec2::new(vel.x, vel.y)),
          orientation: robot.orientation.to_degrees(),
          angular_vel: robot.vel_angular.unwrap_or_default().to_degrees(),
          distance_team: dist_team,
          distance_opponent: dist_opponent,
          distance_ball: Some(Vec2::new(robot.pos.x, robot.pos.y).dot(&ball.pos).sqrt()),
          distance_goal: Some(Vec2::new(robot.pos.x, robot.pos.y).dot(&goal_point).sqrt()),
          distance_wall: Option::from(create_wall_points(
            &Vec2::new_from_cp(robot.pos),
            field_setup,
          )),
        });
      } else {
        robots_opp.push(Robot {
          robot_id: robot.robot_id.id.unwrap_or_default() as u8,
          team: robot.robot_id.team.unwrap_or_default() as u8,
          pos: Some(Vec2::new(robot.pos.x, robot.pos.y)),
          vel: robot.vel.map(|vel| Vec2::new(vel.x, vel.y)),
          orientation: robot.orientation.to_degrees(),
          angular_vel: robot.vel_angular.unwrap_or_default().to_degrees(),
          distance_team: dist_team,
          distance_opponent: dist_opponent,
          distance_ball: Some(Vec2::new(robot.pos.x, robot.pos.y).dot(&ball.pos).sqrt()),
          distance_goal: Some(Vec2::new(robot.pos.x, robot.pos.y).dot(&goal_point).sqrt()),
          distance_wall: Option::from(create_wall_points(
            &Vec2::new_from_cp(robot.pos),
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
  pub pos: Vec2<f32>,
  pub vel: Vec2<f32>,

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
        pos: Vec2::new(frame.balls[0].pos.x, frame.balls[0].pos.y),
        vel: frame.balls[0]
          .vel
          .map(|vel| Vec2::new(vel.x, vel.y))
          .unwrap_or_default(),
      };
    }

    let kicked_ball = KickedBall {
      pos: Vec2::new_from_cp(frame.kicked_ball.unwrap_or_default().pos),
      vel: Vec2::new(
        frame.kicked_ball.unwrap_or_default().vel.x,
        frame.kicked_ball.unwrap_or_default().vel.y,
      ),
      end_point: frame
        .kicked_ball
        .unwrap_or_default()
        .stop_pos
        .map(|ep| Vec2::new(ep.x, ep.y)),
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
