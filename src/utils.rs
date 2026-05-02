use crate::proto::{CpVector2, Vector2};

pub fn as_cp_vec2(v2: Vector2) -> CpVector2 {
  CpVector2 {
    x: v2.x as i32,
    y: v2.y as i32,
  }
}
