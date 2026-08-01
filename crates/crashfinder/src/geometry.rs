use core_dump::vec::types::Vec2;

pub(crate) const EPSILON: f32 = 1.0e-5;

#[inline]
pub(crate) fn is_finite(point: Vec2<f32>) -> bool {
  point.x.is_finite() && point.y.is_finite()
}

#[inline]
pub(crate) fn det(a: Vec2<f32>, b: Vec2<f32>) -> f32 {
  a.x * b.y - a.y * b.x
}

#[inline]
pub(crate) fn perpendicular_left(vector: Vec2<f32>) -> Vec2<f32> {
  Vec2::new(-vector.y, vector.x)
}

#[inline]
pub(crate) fn perpendicular_right(vector: Vec2<f32>) -> Vec2<f32> {
  Vec2::new(vector.y, -vector.x)
}

#[inline]
pub(crate) fn lerp(a: Vec2<f32>, b: Vec2<f32>, t: f32) -> Vec2<f32> {
  a + (b - a) * t
}

#[inline]
pub(crate) fn clamp_magnitude(vector: Vec2<f32>, maximum: f32) -> Vec2<f32> {
  let length_squared = vector.norm_squared();
  if length_squared <= maximum * maximum {
    vector
  } else if length_squared <= EPSILON * EPSILON {
    Vec2::zero()
  } else {
    vector * (maximum / length_squared.sqrt())
  }
}

#[inline]
pub(crate) fn closest_point_on_segment(
  point: Vec2<f32>,
  start: Vec2<f32>,
  end: Vec2<f32>,
) -> Vec2<f32> {
  let segment = end - start;
  let length_squared = segment.norm_squared();
  if length_squared <= EPSILON * EPSILON {
    return start;
  }
  let t = ((point - start).dot(&segment) / length_squared).clamp(0.0, 1.0);
  start + segment * t
}

pub(crate) fn segment_distance(
  a_start: Vec2<f32>,
  a_end: Vec2<f32>,
  b_start: Vec2<f32>,
  b_end: Vec2<f32>,
) -> f32 {
  if segments_intersect(a_start, a_end, b_start, b_end) {
    return 0.0;
  }

  a_start
    .distance_to_segment(b_start, b_end)
    .min(a_end.distance_to_segment(b_start, b_end))
    .min(b_start.distance_to_segment(a_start, a_end))
    .min(b_end.distance_to_segment(a_start, a_end))
}

fn segments_intersect(
  a_start: Vec2<f32>,
  a_end: Vec2<f32>,
  b_start: Vec2<f32>,
  b_end: Vec2<f32>,
) -> bool {
  let a = a_end - a_start;
  let b = b_end - b_start;
  let denominator = det(a, b);
  let offset = b_start - a_start;

  if denominator.abs() <= EPSILON {
    if det(offset, a).abs() > EPSILON {
      return false;
    }
    let a_length_squared = a.norm_squared();
    if a_length_squared <= EPSILON * EPSILON {
      return a_start.distance_to_segment(b_start, b_end) <= EPSILON;
    }
    let t0 = offset.dot(&a) / a_length_squared;
    let t1 = (b_end - a_start).dot(&a) / a_length_squared;
    return t0.min(t1) <= 1.0 && t0.max(t1) >= 0.0;
  }

  let t = det(offset, b) / denominator;
  let u = det(offset, a) / denominator;
  (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

#[inline]
pub(crate) fn point_in_rectangle(point: Vec2<f32>, min: Vec2<f32>, max: Vec2<f32>) -> bool {
  point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y
}

/// Liang-Barsky intersection against a closed axis-aligned rectangle.
pub(crate) fn segment_intersects_rectangle(
  start: Vec2<f32>,
  end: Vec2<f32>,
  min: Vec2<f32>,
  max: Vec2<f32>,
) -> bool {
  if point_in_rectangle(start, min, max) || point_in_rectangle(end, min, max) {
    return true;
  }

  let delta = end - start;
  let mut lower: f32 = 0.0;
  let mut upper: f32 = 1.0;
  let constraints = [
    (-delta.x, start.x - min.x),
    (delta.x, max.x - start.x),
    (-delta.y, start.y - min.y),
    (delta.y, max.y - start.y),
  ];

  for (p, q) in constraints {
    if p.abs() <= EPSILON {
      if q < 0.0 {
        return false;
      }
      continue;
    }

    let ratio = q / p;
    if p < 0.0 {
      if ratio > upper {
        return false;
      }
      lower = lower.max(ratio);
    } else {
      if ratio < lower {
        return false;
      }
      upper = upper.min(ratio);
    }
  }

  lower <= upper
}

pub(crate) fn nearest_point_outside_rectangle(
  point: Vec2<f32>,
  min: Vec2<f32>,
  max: Vec2<f32>,
) -> Vec2<f32> {
  if !point_in_rectangle(point, min, max) {
    return point;
  }

  let distances = [
    (point.x - min.x, Vec2::new(min.x, point.y)),
    (max.x - point.x, Vec2::new(max.x, point.y)),
    (point.y - min.y, Vec2::new(point.x, min.y)),
    (max.y - point.y, Vec2::new(point.x, max.y)),
  ];
  distances
    .into_iter()
    .min_by(|left, right| left.0.total_cmp(&right.0))
    .map(|(_, projected)| projected)
    .unwrap_or(point)
}

#[inline]
pub(crate) fn normalize_angle(mut angle: f32) -> f32 {
  while angle > std::f32::consts::PI {
    angle -= std::f32::consts::TAU;
  }
  while angle < -std::f32::consts::PI {
    angle += std::f32::consts::TAU;
  }
  angle
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn segment_distance_detects_crossing_segments() {
    let distance = segment_distance(
      Vec2::new(-1.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, -1.0),
      Vec2::new(0.0, 1.0),
    );
    assert_eq!(distance, 0.0);
  }

  #[test]
  fn rectangle_intersection_handles_parallel_segments() {
    let min = Vec2::new(-1.0, -1.0);
    let max = Vec2::new(1.0, 1.0);
    assert!(!segment_intersects_rectangle(
      Vec2::new(-2.0, 2.0),
      Vec2::new(2.0, 2.0),
      min,
      max,
    ));
    assert!(segment_intersects_rectangle(
      Vec2::new(-2.0, 0.0),
      Vec2::new(2.0, 0.0),
      min,
      max,
    ));
  }

  #[test]
  fn angles_wrap_to_shortest_rotation() {
    assert!(
      (normalize_angle(1.5 * std::f32::consts::PI) + 0.5 * std::f32::consts::PI).abs() < 1.0e-5
    );
  }
}
