use std::ops::{Add, Mul, Sub};
use num_traits::{Num, Zero};
use num_traits::real::Real;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vec2<T = i16> {
    pub x: T,
    pub y: T,
}

impl<T> Vec2<T> {
    fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    fn from_tuple(tuple: (T, T)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }

}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
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



impl<T: Add<Output = T>> Add for Vec2<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Sub<Output = T>> Sub for Vec2<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Mul<Output = T> + Clone> Mul<T> for Vec2<T> {
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs,
        }
    }
}