use std::ops::{Add, Div, Mul, Sub};
use crate::vec::types::{Vec2, Vec3};

impl<T> Add for Vec2<T>
where
  T: Add<Output = T>,
{
  type Output = Vec2<T>;

  fn add(self, rhs: Self) -> Self::Output {
    Vec2::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl<T> Sub for Vec2<T>
where
  T: Sub<Output = T>,
{
  type Output = Vec2<T>;

  fn sub(self, rhs: Self) -> Self::Output {
    Vec2::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl<T> Mul for Vec2<T>
where
  T: Mul<Output = T>,
{
  type Output = Vec2<T>;

  fn mul(self, rhs: Self) -> Self::Output {
    Vec2::new(self.x * rhs.x, self.y * rhs.y)
  }
}

impl<T> Mul<T> for Vec2<T>
where
  T: Copy + Mul<Output = T>,
{
  type Output = Vec2<T>;

  fn mul(self, rhs: T) -> Self::Output {
    Vec2::new(self.x * rhs, self.y * rhs)
  }
}

impl<T> Div for Vec2<T>
where
  T: Div<Output = T>,
{
  type Output = Vec2<T>;

  fn div(self, rhs: Self) -> Self::Output {
    Vec2::new(self.x / rhs.x, self.y / rhs.y)
  }
}

impl<T> Div<T> for Vec2<T>
where
  T: Copy + Div<Output = T>,
{
  type Output = Vec2<T>;

  fn div(self, rhs: T) -> Self::Output {
    Vec2::new(self.x / rhs, self.y / rhs)
  }
}

impl<T> Add for Vec3<T>
where
  T: Add<Output = T>,
{
  type Output = Vec3<T>;

  fn add(self, rhs: Self) -> Self::Output {
    Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
  }
}

impl<T> Sub for Vec3<T>
where
  T: Sub<Output = T>,
{
  type Output = Vec3<T>;

  fn sub(self, rhs: Self) -> Self::Output {
    Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
  }
}

impl<T> Mul for Vec3<T>
where
  T: Mul<Output = T>,
{
  type Output = Vec3<T>;

  fn mul(self, rhs: Self) -> Self::Output {
    Vec3::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
  }
}

impl<T> Mul<T> for Vec3<T>
where
  T: Copy + Mul<Output = T>,
{
  type Output = Vec3<T>;

  fn mul(self, rhs: T) -> Self::Output {
    Vec3::new(self.x * rhs, self.y * rhs, self.z * rhs)
  }
}

impl<T> Div<T> for Vec3<T>
where
  T: Copy + Div<Output = T>,
{
  type Output = Vec3<T>;

  fn div(self, rhs: T) -> Self::Output {
    Vec3::new(self.x / rhs, self.y / rhs, self.z / rhs)
  }
}
