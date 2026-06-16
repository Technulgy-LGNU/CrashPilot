pub mod ball_helper;
pub mod best_angle_to_goal;
pub mod robot_data;

use core_dump::proto::{CpVector2, Vector2};

#[inline]
pub fn as_cp_vec2(v2: Vector2) -> CpVector2 {
  CpVector2 {
    x: (v2.x * 1000.0) as i32,
    y: (v2.y * 1000.0) as i32,
  }
}
