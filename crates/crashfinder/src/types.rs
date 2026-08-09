use std::error::Error;
use std::fmt::{Display, Formatter};

use core_dump::types::cp_types::{Ball, FieldData, Robot};
use core_dump::vec::types::Vec2;

pub const TEAM_YELLOW: u8 = 1;
pub const TEAM_BLUE: u8 = 2;

pub const FLAG_AVOID_BALL: u8 = 1 << 0;
pub const FLAG_ALLOW_PENALTY_AREAS: u8 = 1 << 1;

/// All positions and linear velocities use millimetres and seconds.
#[derive(Debug, Clone)]
pub struct CrashfinderRequest {
  /// ID and team of the controlled robot.
  pub robot_id: u8,
  pub robot_team: u8,

  /// All currently tracked robots on the field, including the controlled one.
  pub robots: Vec<Robot>,

  /// Current ball state. It is only an obstacle when [`Self::avoid_ball`] is set.
  pub ball: Option<Ball>,

  pub field_data: FieldData,

  /// Requested robot-center position in mm.
  pub end_position: Vec2<f32>,

  /// Requested absolute orientation in radians.
  pub target_orientation: f32,

  /// Bit 0: avoid the ball. Bit 1: allow both penalty areas.
  pub flags: u8,

  /// Required ball-to-robot-center distance in mm.
  pub avoidance_zone: f32,
}

impl CrashfinderRequest {
  #[inline]
  pub fn avoid_ball(&self) -> bool {
    self.flags & FLAG_AVOID_BALL != 0
  }

  #[inline]
  pub fn avoid_penalty(&self) -> bool {
    self.flags & FLAG_ALLOW_PENALTY_AREAS == 0
  }
}

#[derive(Debug, Clone)]
pub struct CrashfinderConfig {
  pub max_linear_speed_mm_s: f32,
  pub min_linear_speed_mm_s: f32,
  pub max_linear_acceleration_mm_s2: f32,
  pub max_linear_deceleration_mm_s2: f32,
  pub max_lateral_acceleration_mm_s2: f32,

  pub max_angular_speed_mrad_s: f32,
  pub max_angular_acceleration_mrad_s2: f32,
  pub max_angular_deceleration_mrad_s2: f32,

  pub robot_radius_mm: f32,
  pub robot_clearance_mm: f32,
  pub static_clearance_mm: f32,
  pub position_tolerance_mm: f32,
  pub orientation_tolerance_rad: f32,

  pub grid_resolution_mm: f32,
  pub max_search_nodes: usize,
  pub corner_radius_mm: f32,
  pub smoothing_samples_per_corner: usize,
  pub path_lookahead_mm: f32,
  pub replan_interval_seconds: f32,
  pub global_prediction_horizon_seconds: f32,

  pub neighbor_distance_mm: f32,
  pub robot_time_horizon_seconds: f32,
  pub ball_time_horizon_seconds: f32,
  pub teammate_avoidance_responsibility: f32,
  pub opponent_avoidance_responsibility: f32,
  pub ball_escape_speed_mm_s: f32,
}

impl Default for CrashfinderConfig {
  fn default() -> Self {
    Self {
      max_linear_speed_mm_s: 2_000.0,
      min_linear_speed_mm_s: 200.0,
      max_linear_acceleration_mm_s2: 4_000.0,
      max_linear_deceleration_mm_s2: 4_000.0,
      max_lateral_acceleration_mm_s2: 3_000.0,

      max_angular_speed_mrad_s: 6_000.0,
      max_angular_acceleration_mrad_s2: 12_000.0,
      max_angular_deceleration_mrad_s2: 12_000.0,

      robot_radius_mm: 90.0,
      robot_clearance_mm: 40.0,
      static_clearance_mm: 20.0,
      position_tolerance_mm: 20.0,
      orientation_tolerance_rad: 0.02,

      grid_resolution_mm: 150.0,
      max_search_nodes: 20_000,
      corner_radius_mm: 250.0,
      smoothing_samples_per_corner: 6,
      path_lookahead_mm: 450.0,
      replan_interval_seconds: 0.2,
      global_prediction_horizon_seconds: 0.35,

      neighbor_distance_mm: 2_500.0,
      robot_time_horizon_seconds: 0.8,
      ball_time_horizon_seconds: 0.5,
      teammate_avoidance_responsibility: 0.5,
      opponent_avoidance_responsibility: 1.0,
      ball_escape_speed_mm_s: 600.0,
    }
  }
}

impl CrashfinderConfig {
  pub fn validate(&self) -> Result<(), ConfigError> {
    positive(self.max_linear_speed_mm_s, "max_linear_speed_mm_s")?;
    non_negative(self.min_linear_speed_mm_s, "min_linear_speed_mm_s")?;
    if self.min_linear_speed_mm_s > self.max_linear_speed_mm_s {
      return Err(ConfigError::new("min_linear_speed_mm_s"));
    }
    positive(
      self.max_linear_acceleration_mm_s2,
      "max_linear_acceleration_mm_s2",
    )?;
    positive(
      self.max_linear_deceleration_mm_s2,
      "max_linear_deceleration_mm_s2",
    )?;
    positive(
      self.max_lateral_acceleration_mm_s2,
      "max_lateral_acceleration_mm_s2",
    )?;
    positive(self.max_angular_speed_mrad_s, "max_angular_speed_mrad_s")?;
    positive(
      self.max_angular_acceleration_mrad_s2,
      "max_angular_acceleration_mrad_s2",
    )?;
    positive(
      self.max_angular_deceleration_mrad_s2,
      "max_angular_deceleration_mrad_s2",
    )?;
    positive(self.robot_radius_mm, "robot_radius_mm")?;
    non_negative(self.robot_clearance_mm, "robot_clearance_mm")?;
    non_negative(self.static_clearance_mm, "static_clearance_mm")?;
    non_negative(self.position_tolerance_mm, "position_tolerance_mm")?;
    non_negative(self.orientation_tolerance_rad, "orientation_tolerance_rad")?;
    if self.orientation_tolerance_rad > std::f32::consts::PI {
      return Err(ConfigError::new("orientation_tolerance_rad"));
    }
    positive(self.grid_resolution_mm, "grid_resolution_mm")?;
    if self.max_search_nodes == 0 {
      return Err(ConfigError::new("max_search_nodes"));
    }
    non_negative(self.corner_radius_mm, "corner_radius_mm")?;
    if self.smoothing_samples_per_corner == 0 {
      return Err(ConfigError::new("smoothing_samples_per_corner"));
    }
    positive(self.path_lookahead_mm, "path_lookahead_mm")?;
    non_negative(self.replan_interval_seconds, "replan_interval_seconds")?;
    non_negative(
      self.global_prediction_horizon_seconds,
      "global_prediction_horizon_seconds",
    )?;
    positive(self.neighbor_distance_mm, "neighbor_distance_mm")?;
    positive(
      self.robot_time_horizon_seconds,
      "robot_time_horizon_seconds",
    )?;
    positive(self.ball_time_horizon_seconds, "ball_time_horizon_seconds")?;
    unit_interval(
      self.teammate_avoidance_responsibility,
      "teammate_avoidance_responsibility",
    )?;
    unit_interval(
      self.opponent_avoidance_responsibility,
      "opponent_avoidance_responsibility",
    )?;
    positive(self.ball_escape_speed_mm_s, "ball_escape_speed_mm_s")?;
    if self.ball_escape_speed_mm_s > self.max_linear_speed_mm_s {
      return Err(ConfigError::new("ball_escape_speed_mm_s"));
    }
    Ok(())
  }
}

fn positive(value: f32, field: &'static str) -> Result<(), ConfigError> {
  if value.is_finite() && value > 0.0 {
    Ok(())
  } else {
    Err(ConfigError::new(field))
  }
}

fn non_negative(value: f32, field: &'static str) -> Result<(), ConfigError> {
  if value.is_finite() && value >= 0.0 {
    Ok(())
  } else {
    Err(ConfigError::new(field))
  }
}

fn unit_interval(value: f32, field: &'static str) -> Result<(), ConfigError> {
  if value.is_finite() && (0.0..=1.0).contains(&value) {
    Ok(())
  } else {
    Err(ConfigError::new(field))
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigError {
  pub field: &'static str,
}

impl ConfigError {
  const fn new(field: &'static str) -> Self {
    Self { field }
  }
}

impl Display for ConfigError {
  fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
    write!(
      formatter,
      "invalid crashfinder configuration field: {}",
      self.field
    )
  }
}

impl Error for ConfigError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlannerStatus {
  FollowingPath,
  AtTarget,
  EscapingBall,
  NoPath,
  RobotNotFound,
  #[default]
  InvalidInput,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DebugObstacleShape {
  Circle {
    center: Vec2<f32>,
    radius_mm: f32,
  },
  Capsule {
    start: Vec2<f32>,
    end: Vec2<f32>,
    radius_mm: f32,
  },
  Rectangle {
    min: Vec2<f32>,
    max: Vec2<f32>,
  },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugObstacleKind {
  Robot { team: u8, id: u8 },
  Ball,
  PenaltyArea,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DebugObstacle {
  pub kind: DebugObstacleKind,
  pub shape: DebugObstacleShape,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DebugGraphNode {
  pub position: Vec2<f32>,
  pub cost_from_start: f32,
  pub estimated_total_cost: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DebugGraphEdge {
  pub from: Vec2<f32>,
  pub to: Vec2<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AvoidanceSource {
  Robot { team: u8, id: u8 },
  Ball,
}

/// A local ORCA half-plane. `direction` is the line tangent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DebugAvoidanceLine {
  pub point: Vec2<f32>,
  pub direction: Vec2<f32>,
  pub source: AvoidanceSource,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CrashfinderDebug {
  pub status: PlannerStatus,
  pub replanned: bool,
  pub usable_min: Vec2<f32>,
  pub usable_max: Vec2<f32>,
  pub effective_target: Vec2<f32>,
  pub local_target: Vec2<f32>,
  pub preferred_velocity: Vec2<f32>,
  pub avoidance_velocity: Vec2<f32>,
  pub commanded_velocity: Vec2<f32>,
  pub obstacles: Vec<DebugObstacle>,
  pub search_nodes: Vec<DebugGraphNode>,
  pub search_edges: Vec<DebugGraphEdge>,
  pub raw_path: Vec<Vec2<f32>>,
  pub smoothed_path: Vec<Vec2<f32>>,
  pub avoidance_lines: Vec<DebugAvoidanceLine>,
}

impl Default for CrashfinderDebug {
  fn default() -> Self {
    Self {
      status: PlannerStatus::InvalidInput,
      replanned: false,
      usable_min: Vec2::zero(),
      usable_max: Vec2::zero(),
      effective_target: Vec2::zero(),
      local_target: Vec2::zero(),
      preferred_velocity: Vec2::zero(),
      avoidance_velocity: Vec2::zero(),
      commanded_velocity: Vec2::zero(),
      obstacles: Vec::new(),
      search_nodes: Vec::new(),
      search_edges: Vec::new(),
      raw_path: Vec::new(),
      smoothed_path: Vec::new(),
      avoidance_lines: Vec::new(),
    }
  }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CrashfinderPath {
  /// Planned world-frame velocity in mm/s.
  pub robot_vel: Vec2<f32>,

  /// Planned angular velocity in mrad/s.
  pub angular_vel: f32,

  pub debug: CrashfinderDebug,
}

impl CrashfinderPath {
  pub(crate) fn stopped(mut debug: CrashfinderDebug) -> Self {
    debug.commanded_velocity = Vec2::zero();
    Self {
      robot_vel: Vec2::zero(),
      angular_vel: 0.0,
      debug,
    }
  }
}
