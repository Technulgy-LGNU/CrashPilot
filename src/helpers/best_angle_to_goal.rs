use crate::game_logic::types::{Robot, WorldState};
use crate::utils::FieldSetup;
use crate::RobotData;
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
  match best_shot_angle(
    robot_self.pos.unwrap_or_default(),
    all_robots,
    Vec2::new(
      field_setup.width as f32 * 0.5 * state.site,
      field_setup.goal_width as f32 * 0.5 * state.site,
    ),
    Vec2::new(
      field_setup.width as f32 * 0.5 * state.site,
      -(field_setup.goal_width as f32) * 0.5 * state.site,
    ),
  ) {
    None => {
      // Try to shoot to the center
      let angle = (robot_self.pos.unwrap_or_default()
        + Vec2::new(field_setup.width as f32 * state.site, 0f32))
      .angle_in_u16();

      robot.msg.cmd.task = TaskKick as i32;
      robot.msg.cmd.kick_orient = Option::from(angle as u32);
      robot.msg.cmd.kick_speed = Option::from(200);
    }
    Some(angle) => {
      robot.msg.cmd.task = TaskKick as i32;
      robot.msg.cmd.kick_orient = Option::from(angle);
      robot.msg.cmd.kick_speed = Option::from(200);
    }
  };
}

#[inline]
fn best_shot_angle(
  shooter: Vec2<f32>,
  opponents: &[Robot],
  goal_left: Vec2<f32>,
  goal_right: Vec2<f32>,
) -> Option<u32> {
  let angle_left = (goal_left.y - shooter.y)
    .atan2(goal_left.x - shooter.x)
    .to_degrees();

  let angle_right = (goal_right.y - shooter.y)
    .atan2(goal_right.x - shooter.x)
    .to_degrees();

  let min_angle = angle_left.min(angle_right).floor() as i32;
  let max_angle = angle_left.max(angle_right).ceil() as i32;

  const ROBOT_RADIUS_MM: f32 = 90.0;
  const BALL_RADIUS_MM: f32 = 21.5;
  const SAFETY_MARGIN_MM: f32 = 10.0;

  let block_radius = ROBOT_RADIUS_MM + BALL_RADIUS_MM + SAFETY_MARGIN_MM;

  let mut free_angles = Vec::new();

  for angle_deg in min_angle..=max_angle {
    let angle = (angle_deg as f32).to_radians();

    let dir = Vec2::new(angle.cos(), angle.sin());

    let mut blocked = false;

    for robot in opponents {
      let rel = robot.pos.unwrap_or_default() - shooter;

      let forward = rel.dot(&dir);

      if forward <= 0.0 {
        continue;
      }

      let closest = rel - dir * forward;

      if closest.length() < block_radius {
        blocked = true;
        break;
      }
    }

    if !blocked {
      free_angles.push(angle_deg);
    }
  }

  if free_angles.is_empty() {
    return None;
  }

  let mut best_start = free_angles[0];
  let mut best_end = free_angles[0];

  let mut current_start = free_angles[0];
  let mut current_end = free_angles[0];

  for &angle in free_angles.iter().skip(1) {
    if angle == current_end + 1 {
      current_end = angle;
    } else {
      if current_end - current_start > best_end - best_start {
        best_start = current_start;
        best_end = current_end;
      }

      current_start = angle;
      current_end = angle;
    }
  }

  if current_end - current_start > best_end - best_start {
    best_start = current_start;
    best_end = current_end;
  }

  Some(((best_start + best_end) as f32 * 0.5) as u32)
}
