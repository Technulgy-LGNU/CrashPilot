use crate::RobotData;
use crate::game_logic::types::{Robot, WorldState};
use crate::helpers::compensated_kick_direction;
use crate::utils::FieldSetup;
use core_dump::proto::CpTask::TaskKick;
use core_dump::vec::types::Vec2;

#[inline]
pub fn shoot_to_goal(
  robot: &mut RobotData,
  robot_self: &Robot,
  all_robots: &[Robot],
  state: &WorldState,
  field_setup: &FieldSetup,
) {
  let robot_pos = robot_self.pos.unwrap_or_default();
  let shooter = shot_origin(robot_pos, state.ball.ball.pos);
  let opponent_goal_side = opponent_goal_side(state);
  let goal_center = Vec2::new(field_setup.width as f32 * 0.5 * opponent_goal_side, 0f32);
  let opponents: Vec<Robot> = all_robots
    .iter()
    .filter(|r| r.team != robot_self.team)
    .cloned()
    .collect();

  match best_shot_angle(
    shooter,
    &opponents,
    Vec2::new(
      field_setup.width as f32 * 0.5 * opponent_goal_side,
      shot_goal_half_width(field_setup),
    ),
    Vec2::new(
      field_setup.width as f32 * 0.5 * opponent_goal_side,
      -shot_goal_half_width(field_setup),
    ),
  ) {
    None => {
      // Try to shoot to the center
      let kick_direction = compensated_kick_direction(
        goal_center - shooter,
        robot_self.vel.unwrap_or_default(),
        robot_self.angular_vel,
        GOAL_KICK_POWER,
      );

      robot.msg.cmd.task = TaskKick as i32;
      robot.msg.cmd.kick_orient = Option::from(kick_direction.angle_in_u16() as u32);
      robot.msg.cmd.kick_speed = Option::from(GOAL_KICK_POWER);
    }
    Some(angle) => {
      let selected_direction = direction_from_command_deg(angle);
      let kick_direction = compensated_kick_direction(
        selected_direction,
        robot_self.vel.unwrap_or_default(),
        robot_self.angular_vel,
        GOAL_KICK_POWER,
      );

      robot.msg.cmd.task = TaskKick as i32;
      robot.msg.cmd.kick_orient = Option::from(kick_direction.angle_in_u16() as u32);
      robot.msg.cmd.kick_speed = Option::from(GOAL_KICK_POWER);
    }
  };
}

const GOAL_KICK_POWER: u32 = 200;

#[inline]
fn shot_origin(robot_pos: Vec2<f32>, ball_pos: Vec2<f32>) -> Vec2<f32> {
  const MAX_CAPTURED_BALL_DIST_MM: f32 = 260.0;

  if (ball_pos - robot_pos).length() <= MAX_CAPTURED_BALL_DIST_MM {
    ball_pos
  } else {
    robot_pos
  }
}

#[inline]
fn shot_goal_half_width(field_setup: &FieldSetup) -> f32 {
  const GOAL_POST_CLEARANCE_MM: f32 = 120.0;

  (field_setup.goal_width as f32 * 0.5 - GOAL_POST_CLEARANCE_MM).max(50.0)
}

#[inline]
fn opponent_goal_side(state: &WorldState) -> f32 {
  if state.site.abs() > f32::EPSILON {
    -state.site.signum()
  } else {
    1.0
  }
}

#[inline]
fn direction_from_command_deg(angle: u32) -> Vec2<f32> {
  let radians = (angle as f32).to_radians();
  Vec2::new(radians.cos(), radians.sin())
}

#[inline]
fn best_shot_angle(
  shooter: Vec2<f32>,
  opponents: &[Robot],
  goal_left: Vec2<f32>,
  goal_right: Vec2<f32>,
) -> Option<u32> {
  let angle_left = angle_to_point(shooter, goal_left);
  let angle_right = angle_to_point(shooter, goal_right);
  let (start_angle, span) = goal_arc_start_and_span(angle_left, angle_right);

  const ROBOT_RADIUS_MM: f32 = 90.0;
  const BALL_RADIUS_MM: f32 = 21.5;
  const SAFETY_MARGIN_MM: f32 = 10.0;

  let block_radius = ROBOT_RADIUS_MM + BALL_RADIUS_MM + SAFETY_MARGIN_MM;

  let mut best_range = None;
  let mut current_range = None;
  let sample_count = span.ceil() as usize;

  for sample in 0..=sample_count {
    let offset = (sample as f32).min(span);
    let angle_deg = start_angle + offset;
    let angle = angle_deg.to_radians();

    let dir = Vec2::new(angle.cos(), angle.sin());
    let goal_forward = if dir.x.abs() <= f32::EPSILON {
      f32::INFINITY
    } else {
      (goal_left.x - shooter.x) / dir.x
    };

    if goal_forward <= 0.0 {
      finish_range(&mut current_range, &mut best_range);
      continue;
    }

    let mut blocked = false;

    for robot in opponents {
      let Some(pos) = robot.pos else {
        continue;
      };

      let rel = pos - shooter;

      let forward = rel.dot(&dir);

      if forward <= 0.0 || forward - block_radius > goal_forward {
        continue;
      }

      let closest = rel - dir * forward;

      if closest.length() < block_radius {
        blocked = true;
        break;
      }
    }

    if !blocked {
      match current_range {
        Some((start, _)) => current_range = Some((start, sample)),
        None => current_range = Some((sample, sample)),
      }
    } else {
      finish_range(&mut current_range, &mut best_range);
    }
  }

  finish_range(&mut current_range, &mut best_range);

  let (best_start, best_end) = best_range?;
  let best_start = (best_start as f32).min(span);
  let best_end = (best_end as f32).min(span);
  let best_angle = start_angle + (best_start + best_end) * 0.5;

  Some(angle_to_command_deg(best_angle))
}

#[inline]
fn angle_to_point(from: Vec2<f32>, to: Vec2<f32>) -> f32 {
  (to.y - from.y).atan2(to.x - from.x).to_degrees()
}

#[inline]
fn goal_arc_start_and_span(angle_a: f32, angle_b: f32) -> (f32, f32) {
  let angle_a = normalize_angle_deg(angle_a);
  let angle_b = normalize_angle_deg(angle_b);
  let ccw_span = normalize_angle_deg(angle_b - angle_a);

  if ccw_span <= 180.0 {
    (angle_a, ccw_span)
  } else {
    (angle_b, 360.0 - ccw_span)
  }
}

#[inline]
fn finish_range(
  current_range: &mut Option<(usize, usize)>,
  best_range: &mut Option<(usize, usize)>,
) {
  let Some(current) = current_range.take() else {
    return;
  };

  if best_range
    .map(|best| current.1 - current.0 > best.1 - best.0)
    .unwrap_or(true)
  {
    *best_range = Some(current);
  }
}

#[inline]
fn angle_to_command_deg(angle: f32) -> u32 {
  normalize_angle_deg(normalize_angle_deg(angle).round()) as u32
}

#[inline]
fn normalize_angle_deg(angle: f32) -> f32 {
  angle.rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::game_logic::types::Team;

  fn robot_at(x: f32, y: f32) -> Robot {
    Robot {
      robot_id: 0,
      pos: Some(Vec2::new(x, y)),
      vel: None,
      orientation: 0.0,
      angular_vel: 0.0,
      team: Team::Yellow,
      distance_team: Default::default(),
      _distance_opponent: Default::default(),
      distance_ball: None,
      _distance_goal: None,
      _distance_wall: None,
    }
  }

  #[test]
  fn normalizes_negative_best_angle_instead_of_clamping_to_zero() {
    let angle = best_shot_angle(
      Vec2::new(0.0, 500.0),
      &[],
      Vec2::new(4_500.0, 500.0),
      Vec2::new(4_500.0, -500.0),
    );

    assert_eq!(angle, Some(354));
  }

  #[test]
  fn chooses_non_zero_angle_when_center_is_blocked() {
    let angle = best_shot_angle(
      Vec2::new(0.0, 0.0),
      &[robot_at(2_000.0, 0.0)],
      Vec2::new(4_500.0, 500.0),
      Vec2::new(4_500.0, -500.0),
    )
    .unwrap();

    assert_ne!(angle, 0);
  }

  #[test]
  fn handles_goal_arc_crossing_angle_wrap() {
    let angle = best_shot_angle(
      Vec2::new(0.0, 0.0),
      &[],
      Vec2::new(-4_500.0, 500.0),
      Vec2::new(-4_500.0, -500.0),
    );

    assert_eq!(angle, Some(180));
  }

  #[test]
  fn fallback_direction_points_from_shooter_to_goal_center() {
    let angle = angle_to_command_deg(angle_to_point(
      Vec2::new(0.0, 500.0),
      Vec2::new(4_500.0, 0.0),
    ));

    assert_eq!(angle, 354);
  }

  #[test]
  fn kick_goal_targets_negative_x_when_own_goal_is_positive_x() {
    let mut robot = RobotData::default();
    let robot_self = robot_at(0.0, 0.0);
    let state = WorldState {
      site: 1.0,
      ..WorldState::default()
    };

    shoot_to_goal(&mut robot, &robot_self, &[], &state, &FieldSetup::default());

    assert_eq!(robot.msg.cmd.task, TaskKick as i32);
    assert_eq!(robot.msg.cmd.kick_orient, Some(180));
  }

  #[test]
  fn kick_goal_targets_positive_x_when_own_goal_is_negative_x() {
    let mut robot = RobotData::default();
    let robot_self = robot_at(0.0, 0.0);
    let state = WorldState {
      site: -1.0,
      ..WorldState::default()
    };

    shoot_to_goal(&mut robot, &robot_self, &[], &state, &FieldSetup::default());

    assert_eq!(robot.msg.cmd.task, TaskKick as i32);
    assert_eq!(robot.msg.cmd.kick_orient, Some(0));
  }

  #[test]
  fn kick_goal_compensates_sideways_shooter_velocity() {
    let mut robot = RobotData::default();
    let mut stationary_robot = RobotData::default();
    let stationary_self = robot_at(0.0, 0.0);
    let mut robot_self = robot_at(0.0, 0.0);
    robot_self.vel = Some(Vec2::new(0.0, 1_000.0));
    let mut blocker = robot_at(1_000.0, 0.0);
    blocker.team = Team::Blue;
    let state = WorldState {
      site: -1.0,
      ..WorldState::default()
    };
    let all_robots = [blocker];

    shoot_to_goal(
      &mut stationary_robot,
      &stationary_self,
      &all_robots,
      &state,
      &FieldSetup::default(),
    );
    shoot_to_goal(
      &mut robot,
      &robot_self,
      &all_robots,
      &state,
      &FieldSetup::default(),
    );

    assert_eq!(robot.msg.cmd.task, TaskKick as i32);
    let stationary_angle = stationary_robot.msg.cmd.kick_orient.unwrap();
    let moving_angle = robot.msg.cmd.kick_orient.unwrap();
    assert_eq!(stationary_angle, 0);
    assert!(
      moving_angle > 330,
      "expected wrapped negative compensation, got {moving_angle}"
    );
  }

  #[test]
  fn uses_ball_position_as_shot_origin_when_captured() {
    let mut robot = RobotData::default();
    let robot_self = robot_at(0.0, 0.0);
    let state = WorldState {
      site: -1.0,
      ball: crate::game_logic::types::BallData {
        ball: crate::game_logic::types::Ball {
          pos: Vec2::new(180.0, 180.0),
          ..Default::default()
        },
        ..Default::default()
      },
      ..WorldState::default()
    };

    shoot_to_goal(&mut robot, &robot_self, &[], &state, &FieldSetup::default());

    assert_eq!(robot.msg.cmd.task, TaskKick as i32);
    assert_eq!(robot.msg.cmd.kick_orient, Some(358));
  }

  #[test]
  fn keeps_selected_shot_inside_goal_posts() {
    let field = FieldSetup {
      goal_width: 1000,
      ..FieldSetup::default()
    };
    let half_width = shot_goal_half_width(&field);

    assert_eq!(half_width, 380.0);
  }
}
