/*
use std::ops::{Add, Div, Mul, Sub};
use crate::proto::CpVector2;

pub struct Vec2f {
  pub x: f32,
  pub y: f32,
}

impl Vec2f {
  #[inline]
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  #[inline]
  pub fn new_from_cp(v: CpVector2) -> Self {
    Vec2f::new(v.x as f32, v.y as f32)
  }

  #[inline]
  pub fn norm(&self) -> f32 {
    self.norm_squared().sqrt()
  }

  #[inline]
  pub fn norm_squared(&self) -> f32 {
    self.x * self.x + self.y * self.y
  }

  #[inline]
  pub(crate) fn normalized(self) -> Vec2f {
    let n = self.norm();
    if n <= 1e-6 {
      Self::new(0f32, 0f32)
    } else {
      self.scale(1f32 / n)
    }
  }

  #[inline]
  pub(crate) fn scale(self, s: f32) -> Vec2f {
    Self::new(self.x * s, self.y * s)
  }

  /// Scalar Product
  #[inline]
  pub(crate) fn dot(self, other: Vec2f) -> f32 {
    self.x * other.x + self.y * other.y
  }
}

impl Add for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn add(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl Sub for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn sub(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl Mul for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn mul(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x * rhs.x, self.y * rhs.y)
  }
}

impl Mul<f32> for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn mul(self, rhs: f32) -> Self::Output {
    Vec2f::new(self.x * rhs, self.y * rhs)
  }
}

impl Div for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn div(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x / rhs.x, self.y / rhs.y)
  }
}

impl Div<f32> for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn div(self, rhs: f32) -> Self::Output {
    Vec2f::new(self.x / rhs, self.y / rhs)
  }
}
*/
