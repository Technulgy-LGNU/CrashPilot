pub enum QuadraticResult {
  None,
  One(f32),
  Two(f32, f32),
}

impl QuadraticResult {
  pub fn from_coefficients(a: f32, b: f32, c: f32) -> Self {
    let discriminant = b * b - 4.0 * a * c;

    if discriminant < 0.0 {
      Self::None
    } else if discriminant == 0.0 {
      let root = -b / (2.0 * a);
      Self::One(root)
    } else {
      let sqrt_disc = discriminant.sqrt();
      let root1 = (-b + sqrt_disc) / (2.0 * a);
      let root2 = (-b - sqrt_disc) / (2.0 * a);
      Self::Two(root1, root2)
    }
  }
}

impl Iterator for QuadraticResult {
  type Item = f32;

  fn next(&mut self) -> Option<Self::Item> {
    match self {
      Self::None => None,
      Self::One(root) => {
        let result = *root;
        *self = Self::None; // Mark as consumed
        Some(result)
      }
      Self::Two(root1, root2) => {
        let result = *root1;
        *self = Self::One(*root2); // Move the second root to One
        Some(result)
      }
    }
  }
}
