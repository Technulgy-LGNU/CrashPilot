pub mod ball_helper;
pub mod best_angle_to_goal;
pub mod robot_data;

use core_dump::proto::{CpVector2, Vector2};
use core_dump::vec::types::Vec2;

#[inline]
pub fn as_cp_vec2(v2: Vector2) -> CpVector2 {
  CpVector2 {
    x: (v2.x * 1000.0) as i32,
    y: (v2.y * 1000.0) as i32,
  }
}

#[inline]
pub fn compensated_kick_direction(
  desired_ball_direction: Vec2<f32>,
  shooter_velocity_mm_s: Vec2<f32>,
  shooter_angular_velocity_deg_s: f32,
  kick_power: u32,
) -> Vec2<f32> {
  const MAX_FLAT_KICK_SPEED_MM_S: f32 = 10_000.0;
  const KICKER_CONTACT_RADIUS_MM: f32 = 80.0;
  const MAX_COMPENSATION_ANGLE_DEG: f32 = 10.0;
  const MAX_COMPENSATION_TARGET_OFFSET_MM: f32 = 250.0;

  let desired_len = desired_ball_direction.length();
  let kick_speed_mm_s = kick_power as f32 / u8::MAX as f32 * MAX_FLAT_KICK_SPEED_MM_S;
  if desired_len <= 1.0 || kick_speed_mm_s <= 1.0 {
    return desired_ball_direction;
  }

  let desired = desired_ball_direction / desired_len;
  let lateral = Vec2::new(-desired.y, desired.x);
  let contact_velocity_mm_s = shooter_velocity_mm_s
    + lateral * shooter_angular_velocity_deg_s.to_radians() * KICKER_CONTACT_RADIUS_MM;
  let lateral_contact_speed = contact_velocity_mm_s.dot(&lateral);
  let max_angle = MAX_COMPENSATION_ANGLE_DEG
    .to_radians()
    .min((MAX_COMPENSATION_TARGET_OFFSET_MM / desired_len).atan());
  let max_compensation_sin = max_angle.sin();
  let lateral_kick_component =
    (-lateral_contact_speed / kick_speed_mm_s).clamp(-max_compensation_sin, max_compensation_sin);
  let forward_kick_component = (1.0 - lateral_kick_component * lateral_kick_component).sqrt();

  (desired * forward_kick_component + lateral * lateral_kick_component) * desired_len
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn compensated_kick_direction_aims_against_lateral_shooter_velocity() {
    let direction =
      compensated_kick_direction(Vec2::new(1_000.0, 0.0), Vec2::new(0.0, 1_000.0), 0.0, 100);

    assert!(direction.x > 0.0);
    assert!(direction.y < 0.0);
  }

  #[test]
  fn compensated_kick_direction_aims_against_rotating_kicker_mouth() {
    let direction = compensated_kick_direction(Vec2::new(1_000.0, 0.0), Vec2::zero(), 360.0, 100);

    assert!(direction.x > 0.0);
    assert!(direction.y < 0.0);
  }

  #[test]
  fn compensated_kick_direction_limits_long_kick_lane_offset() {
    let direction =
      compensated_kick_direction(Vec2::new(9_000.0, 0.0), Vec2::new(0.0, 5_000.0), 0.0, 90);

    assert!(direction.x > 0.0);
    assert!(direction.y < 0.0);
    assert!(direction.y.abs() <= 250.1);
  }
}
