use core_dump::proto::TrackerWrapperPacket;
use core_dump::vec::types::Vec2;

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
  // Todo: add fields with vectors to all other robots for easier calculations
}

impl Robot {
  /// Converts robots from the tracked vision to our own robot type
  #[inline]
  pub fn new_from_tracked(vis_tracked: &TrackerWrapperPacket) -> Vec<Robot> {
    let robots_tracked = vis_tracked.tracked_frame.clone().unwrap_or_default().robots;
    if robots_tracked.is_empty() {
      return vec![];
    }
    let mut robots: Vec<Robot> = vec![];

    for robot in robots_tracked {
      robots.push(Robot {
        robot_id: robot.robot_id.id.unwrap_or_default() as u8,
        team: robot.robot_id.team.unwrap_or_default() as u8,
        pos: Some(Vec2::new(robot.pos.x, robot.pos.y)),
        vel: robot.vel.map(|vel| Vec2::new(vel.x, vel.y)),
        orientation: robot.orientation.to_degrees(),
        angular_vel: robot.vel_angular.unwrap_or_default().to_degrees(),
      });
    }

    robots
  }
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
}

impl BallData {
  #[inline]
  /// Takes either the tracked or raw vision and converts it to our own ball type
  /// The Test Field switcher is not implemented for this function for now
  pub fn new() -> Self {
    Self {
      ball: Default::default(),
      kicked_ball: Default::default(),
    }
  }
}

/// Stores central data about the game
/// This includes:
///   - Goalie
///   - Defenders
///   - Field side
///   - more to come
#[derive(Debug, Default)]
pub struct GameState {
  pub goalie: u8,
  pub defenders: Option<Vec<u8>>,
  pub field_side: u8,
}