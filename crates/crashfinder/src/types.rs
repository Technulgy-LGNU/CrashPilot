use core_dump::vec::types::Vec2;

/// Everything for the pathfinder
pub struct CrashfinderRequest {
  /// ID of the robot
  pub robot_id: u8,

  /// All robots on the field
  pub robots: Vec<core_dump::types::cp_types::Robot>,

  /// Ball data, to avoid it if necessary
  pub ball: Option<core_dump::types::cp_types::Ball>,

  /// End Position
  /// Is directly in mm, f32 is easier for later calculations
  pub end_position: Vec2<f32>,

  /// Flags
  ///   Bit 0: Avoid Ball
  ///   Bit 1: Don't Avoid Penalty Area
  ///   Bit 2:
  ///   Bit 3:
  ///   Bit 4;
  ///   Bit 5:
  ///   Bit 6:
  ///   Bit 7:
  pub flags: u8,

  /// Avoidance distance from ball
  pub avoidance_zone: f32,
}

impl CrashfinderRequest {
  #[inline]
  pub fn avoid_ball(&self) -> bool {
    self.flags & (1 << 0) != 0
  }

  #[inline]
  pub fn avoid_penalty(&self) -> bool {
    // Default is yes
    !(self.flags & (1 << 1) != 0)
  }
}

pub struct CrashfinderPath {
  /// Velocity (Planned velocity for the robot) in mm/s
  pub robot_vel: Vec2<f32>,

  /// Angular Velocity in mrads/s
  pub angular_vel: f32,

  pub debug: CrashfinderDebug,
}

impl CrashfinderPath {

}

pub struct CrashfinderDebug {

}
