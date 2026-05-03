use crate::proto::{CpVector2, Vector2};

pub fn as_cp_vec2(v2: Vector2) -> CpVector2 {
  dbg!(v2);
  CpVector2 {
    x: (v2.x * 1000.0) as i32,
    y: (v2.y * 1000.0) as i32,
  }
}
