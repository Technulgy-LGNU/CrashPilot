use num_traits::Zero;
use crate::proto::{CpVector2, Vector2};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T> {
  pub x: T,
  pub y: T,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
  X,
  Y,
}

impl<T> Vec2<T> {
  #[inline]
  pub fn new(x: T, y: T) -> Self {
    Self { x, y }
  }

  #[inline]
  fn from_tuple(tuple: (T, T)) -> Self {
    Self {
      x: tuple.0,
      y: tuple.1,
    }
  }

  #[inline]
  pub fn new_from_cp(v: Vector2) -> Self
  where
    T: From<f32>,
  {
    Self::new(T::from(v.x), T::from(v.y))
  }

  #[inline]
  pub fn tp_cp_vec2(self) -> CpVector2 where
    T: Into<i32>,
  {
    CpVector2 {
      x: self.x.into(),
      y: self.y.into(),
    }
  }
}

impl<T: Default> Default for Vec2<T> {
  fn default() -> Self {
    Self {
      x: T::default(),
      y: T::default(),
    }
  }
}


impl<T: Zero> Vec2<T> {
  pub fn zero() -> Self {
    Self {
      x: T::zero(),
      y: T::zero(),
    }
  }
}