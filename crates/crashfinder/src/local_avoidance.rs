use core_dump::types::cp_types::Robot;
use core_dump::vec::types::Vec2;

use crate::geometry::{
  EPSILON, clamp_magnitude, det, is_finite, perpendicular_left, perpendicular_right,
};
use crate::types::{AvoidanceSource, CrashfinderConfig, CrashfinderRequest, DebugAvoidanceLine};

#[derive(Debug, Clone)]
pub(crate) struct AvoidanceResult {
  pub velocity: Vec2<f32>,
  pub lines: Vec<DebugAvoidanceLine>,
}

#[derive(Debug, Clone, Copy)]
struct Neighbor {
  position: Vec2<f32>,
  velocity: Vec2<f32>,
  combined_radius: f32,
  time_horizon: f32,
  responsibility: f32,
  source: AvoidanceSource,
}

#[derive(Debug, Clone, Copy)]
struct OrcaLine {
  point: Vec2<f32>,
  direction: Vec2<f32>,
  source: AvoidanceSource,
}

pub(crate) fn avoid_velocity(
  request: &CrashfinderRequest,
  config: &CrashfinderConfig,
  controlled_robot: &Robot,
  current_velocity: Vec2<f32>,
  preferred_velocity: Vec2<f32>,
  time_step: f32,
  include_robots: bool,
) -> AvoidanceResult {
  let mut neighbors = collect_neighbors(request, config, controlled_robot, include_robots);
  neighbors.sort_by(|left, right| {
    controlled_robot
      .pos
      .distance(left.position)
      .total_cmp(&controlled_robot.pos.distance(right.position))
  });

  let mut lines = Vec::with_capacity(neighbors.len());
  for neighbor in neighbors {
    lines.push(make_orca_line(
      controlled_robot.pos,
      current_velocity,
      preferred_velocity,
      neighbor,
      time_step,
    ));
  }

  let internal_lines: Vec<Line> = lines
    .iter()
    .map(|line| Line {
      point: line.point,
      direction: line.direction,
    })
    .collect();
  let preferred = clamp_magnitude(preferred_velocity, config.max_linear_speed_mm_s);
  let mut result = preferred;
  let failed_line = linear_program_2(
    &internal_lines,
    config.max_linear_speed_mm_s,
    preferred,
    false,
    &mut result,
  );
  if failed_line < internal_lines.len() {
    linear_program_3(
      &internal_lines,
      failed_line,
      config.max_linear_speed_mm_s,
      &mut result,
    );
  }

  AvoidanceResult {
    velocity: clamp_magnitude(result, config.max_linear_speed_mm_s),
    lines: lines
      .into_iter()
      .map(|line| DebugAvoidanceLine {
        point: line.point,
        direction: line.direction,
        source: line.source,
      })
      .collect(),
  }
}

fn collect_neighbors(
  request: &CrashfinderRequest,
  config: &CrashfinderConfig,
  controlled_robot: &Robot,
  include_robots: bool,
) -> Vec<Neighbor> {
  let mut neighbors = Vec::with_capacity(request.robots.len());
  let robot_radius = config.robot_radius_mm * 2.0 + config.robot_clearance_mm;
  if include_robots {
    for robot in &request.robots {
      if robot.robot_id == request.robot_id && robot.team == request.robot_team {
        continue;
      }
      if !is_finite(robot.pos)
        || controlled_robot.pos.distance(robot.pos) > config.neighbor_distance_mm + robot_radius
      {
        continue;
      }

      let responsibility = if robot.team == request.robot_team {
        config.teammate_avoidance_responsibility
      } else {
        config.opponent_avoidance_responsibility
      };
      neighbors.push(Neighbor {
        position: robot.pos,
        velocity: robot
          .vel
          .filter(|velocity| is_finite(*velocity))
          .unwrap_or_default(),
        combined_radius: robot_radius,
        time_horizon: config.robot_time_horizon_seconds,
        responsibility,
        source: AvoidanceSource::Robot {
          team: robot.team,
          id: robot.robot_id,
        },
      });
    }
  }

  if request.avoid_ball()
    && request.avoidance_zone > 0.0
    && let Some(ball) = request.ball
    && is_finite(ball.pos)
    && controlled_robot.pos.distance(ball.pos)
      <= config.neighbor_distance_mm + request.avoidance_zone
  {
    neighbors.push(Neighbor {
      position: ball.pos,
      velocity: ball
        .vel
        .filter(|velocity| is_finite(*velocity))
        .unwrap_or_default(),
      combined_radius: request.avoidance_zone,
      time_horizon: config.ball_time_horizon_seconds,
      responsibility: 1.0,
      source: AvoidanceSource::Ball,
    });
  }
  neighbors
}

fn make_orca_line(
  position: Vec2<f32>,
  velocity: Vec2<f32>,
  preferred_velocity: Vec2<f32>,
  neighbor: Neighbor,
  time_step: f32,
) -> OrcaLine {
  let relative_position = neighbor.position - position;
  let relative_velocity = velocity - neighbor.velocity;
  let distance_squared = relative_position.norm_squared();
  let radius_squared = neighbor.combined_radius * neighbor.combined_radius;

  let (direction, correction) = if distance_squared > radius_squared {
    let inverse_horizon = 1.0 / neighbor.time_horizon;
    let w = relative_velocity - relative_position * inverse_horizon;
    let w_length_squared = w.norm_squared();
    let projection = w.dot(&relative_position);

    if projection < 0.0 && projection * projection > radius_squared * w_length_squared {
      let unit_w = stable_unit(w, relative_position, preferred_velocity);
      let w_length = w_length_squared.sqrt();
      (
        perpendicular_right(unit_w),
        unit_w * (neighbor.combined_radius * inverse_horizon - w_length),
      )
    } else {
      let leg = (distance_squared - radius_squared).max(0.0).sqrt();
      let direction = if det(relative_position, w) > 0.0 {
        Vec2::new(
          relative_position.x * leg - relative_position.y * neighbor.combined_radius,
          relative_position.x * neighbor.combined_radius + relative_position.y * leg,
        ) / distance_squared
      } else {
        Vec2::new(
          -(relative_position.x * leg + relative_position.y * neighbor.combined_radius),
          relative_position.x * neighbor.combined_radius - relative_position.y * leg,
        ) / distance_squared
      };
      let projection_on_leg = relative_velocity.dot(&direction);
      (direction, direction * projection_on_leg - relative_velocity)
    }
  } else {
    let inverse_step = 1.0 / time_step.max(1.0e-3);
    let w = relative_velocity - relative_position * inverse_step;
    let unit_w = stable_unit(w, relative_position, preferred_velocity);
    (
      perpendicular_right(unit_w),
      unit_w * (neighbor.combined_radius * inverse_step - w.norm()),
    )
  };

  OrcaLine {
    point: velocity + correction * neighbor.responsibility,
    direction: direction.normalized(),
    source: neighbor.source,
  }
}

fn stable_unit(
  vector: Vec2<f32>,
  relative_position: Vec2<f32>,
  preferred_velocity: Vec2<f32>,
) -> Vec2<f32> {
  if vector.norm_squared() > EPSILON * EPSILON {
    return vector.normalized();
  }
  if relative_position.norm_squared() > EPSILON * EPSILON {
    return relative_position.normalized() * -1.0;
  }
  if preferred_velocity.norm_squared() > EPSILON * EPSILON {
    return preferred_velocity.normalized();
  }
  Vec2::new(1.0, 0.0)
}

#[derive(Debug, Clone, Copy)]
struct Line {
  point: Vec2<f32>,
  direction: Vec2<f32>,
}

fn linear_program_1(
  lines: &[Line],
  line_index: usize,
  radius: f32,
  preferred_velocity: Vec2<f32>,
  direction_only: bool,
  result: &mut Vec2<f32>,
) -> bool {
  let line = lines[line_index];
  let point_projection = line.point.dot(&line.direction);
  let discriminant =
    point_projection * point_projection + radius * radius - line.point.norm_squared();
  if discriminant < 0.0 {
    return false;
  }

  let root = discriminant.sqrt();
  let mut left = -point_projection - root;
  let mut right = -point_projection + root;
  for previous in &lines[..line_index] {
    let denominator = det(line.direction, previous.direction);
    let numerator = det(previous.direction, line.point - previous.point);
    if denominator.abs() <= EPSILON {
      if numerator < 0.0 {
        return false;
      }
      continue;
    }

    let intersection = numerator / denominator;
    if denominator >= 0.0 {
      right = right.min(intersection);
    } else {
      left = left.max(intersection);
    }
    if left > right {
      return false;
    }
  }

  let parameter = if direction_only {
    if preferred_velocity.dot(&line.direction) > 0.0 {
      right
    } else {
      left
    }
  } else {
    (-((line.point - preferred_velocity).dot(&line.direction))).clamp(left, right)
  };
  *result = line.point + line.direction * parameter;
  true
}

fn linear_program_2(
  lines: &[Line],
  radius: f32,
  preferred_velocity: Vec2<f32>,
  direction_only: bool,
  result: &mut Vec2<f32>,
) -> usize {
  *result = if direction_only {
    preferred_velocity * radius
  } else {
    clamp_magnitude(preferred_velocity, radius)
  };

  for (index, line) in lines.iter().enumerate() {
    if det(line.direction, line.point - *result) > 0.0 {
      let previous_result = *result;
      if !linear_program_1(
        lines,
        index,
        radius,
        preferred_velocity,
        direction_only,
        result,
      ) {
        *result = previous_result;
        return index;
      }
    }
  }
  lines.len()
}

fn linear_program_3(lines: &[Line], begin_line: usize, radius: f32, result: &mut Vec2<f32>) {
  let mut violation = 0.0;
  for index in begin_line..lines.len() {
    let current_violation = det(lines[index].direction, lines[index].point - *result);
    if current_violation <= violation {
      continue;
    }

    let mut projected_lines = Vec::with_capacity(index);
    for previous_index in 0..index {
      let current = lines[index];
      let previous = lines[previous_index];
      let determinant = det(current.direction, previous.direction);
      let point = if determinant.abs() <= EPSILON {
        if current.direction.dot(&previous.direction) > 0.0 {
          continue;
        }
        (current.point + previous.point) * 0.5
      } else {
        current.point
          + current.direction
            * (det(previous.direction, current.point - previous.point) / determinant)
      };
      projected_lines.push(Line {
        point,
        direction: (previous.direction - current.direction).normalized(),
      });
    }

    let previous_result = *result;
    if linear_program_2(
      &projected_lines,
      radius,
      perpendicular_left(lines[index].direction),
      true,
      result,
    ) < projected_lines.len()
    {
      *result = previous_result;
    }
    violation = det(lines[index].direction, lines[index].point - *result);
  }
}

#[cfg(test)]
mod tests {
  use core_dump::types::cp_types::{Ball, Robot};

  use super::*;
  use crate::types::FieldData;

  fn robot(id: u8, team: u8, x: f32, y: f32) -> Robot {
    Robot {
      robot_id: id,
      team,
      pos: Vec2::new(x, y),
      ..Robot::default()
    }
  }

  fn request(robots: Vec<Robot>) -> CrashfinderRequest {
    CrashfinderRequest {
      robot_id: 0,
      robot_team: 1,
      robots,
      ball: None,
      field_data: FieldData {
        height: 6_000.0,
        width: 9_000.0,
        runoff_area: 300.0,
        goal_width: 1_000.0,
        penalty_area_width: 1_000.0,
        penalty_area_height: 2_000.0,
      },
      end_position: Vec2::new(1_000.0, 0.0),
      target_orientation: 0.0,
      flags: 1 << 1,
      avoidance_zone: 500.0,
    }
  }

  #[test]
  fn head_on_obstacle_changes_the_preferred_velocity() {
    let own = robot(0, 1, 0.0, 0.0);
    let other = robot(1, 2, 500.0, 0.0);
    let request = request(vec![own, other]);
    let result = avoid_velocity(
      &request,
      &CrashfinderConfig::default(),
      &own,
      Vec2::zero(),
      Vec2::new(1_000.0, 0.0),
      0.02,
      true,
    );
    assert_eq!(result.lines.len(), 1);
    assert!(result.velocity.x < 1_000.0);
    assert!(
      result
        .lines
        .iter()
        .all(|line| { det(line.direction, line.point - result.velocity) <= 1.0e-3 })
    );
  }

  #[test]
  fn overlapping_ball_produces_an_outward_velocity_constraint() {
    let own = robot(0, 1, 0.0, 0.0);
    let mut request = request(vec![own]);
    request.flags |= 1;
    request.ball = Some(Ball {
      pos: Vec2::new(100.0, 0.0),
      vel: None,
    });
    let result = avoid_velocity(
      &request,
      &CrashfinderConfig::default(),
      &own,
      Vec2::zero(),
      Vec2::new(-600.0, 0.0),
      0.02,
      true,
    );
    assert!(result.velocity.x < 0.0);
    assert!(matches!(result.lines[0].source, AvoidanceSource::Ball));
  }
}
