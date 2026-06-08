use num_traits::{Float, Num};
use num_traits::real::Real;
use crate::vec::types::{Axis, Vec2, Vec3};


impl<T: Num + Real + Copy> Vec2<T> {
  pub fn get(&self, axis: Axis) -> T {
    match axis {
      Axis::X => self.x,
      Axis::Y => self.y,
    }
  }
  pub fn dot(&self, other: &Self) -> T {
    self.x * other.x + self.y * other.y
  }

  pub fn length(&self) -> T {
    self.dot(self).sqrt()
  }

  pub fn scale_to(&self, new_length: T) -> Self {
    let current_length = self.length();

    if current_length.is_zero() {
      return Self::zero();
    }

    let scale_factor = new_length / current_length;

    Self {
      x: self.x * scale_factor,
      y: self.y * scale_factor,
    }
  }

  pub fn powf(&self, n: T) -> Self {
    Self {
      x: self.x.powf(n),
      y: self.y.powf(n),
    }
  }
}

/// Float Specifics
impl<T: Float> Vec2<T> {
  #[inline]
  pub fn norm_squared(self) -> T {
    self.x * self.x + self.y * self.y
  }

  #[inline]
  pub fn norm(self) -> T {
    self.norm_squared().sqrt()
  }

  #[inline]
  pub fn normalized(self) -> Self {
    let n = self.norm();

    if n <= T::epsilon() {
      Self::new(T::zero(), T::zero())
    } else {
      self / n
    }
  }

  /// Euclidean distance between two points.
  #[inline]
  pub fn distance(self, other: Self) -> T {
    (self - other).norm()
  }

  /// Shortest distance from this point to the segment `a`–`b`.
  pub fn distance_to_segment(self, a: Self, b: Self) -> T {
    let ab = b - a;
    let denom = ab.norm_squared();
    if denom <= T::epsilon() {
      return self.distance(a);
    }
    let t = ((self - a).dot(&ab) / denom).max(T::zero()).min(T::one());
    let projection = a + ab * t;
    self.distance(projection)
  }
}

/// Float Specifics
impl<T: Float> Vec3<T> {
  #[inline]
  pub fn norm_squared(self) -> T {
    self.x * self.x + self.y * self.y + self.z * self.z
  }

  #[inline]
  pub fn norm(self) -> T {
    self.norm_squared().sqrt()
  }
}
