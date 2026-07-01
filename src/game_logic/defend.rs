use crate::{Communication, CrashPilot};
use core_dump::proto::CpState::StateFree;
use core_dump::proto::CpTask::TaskPos;
use core_dump::types::Ai;
use core_dump::vec::types::Vec2;

const WALL_DEPTH_MM: f32 = 700.0;
const WALL_SPACING_MM: f32 = 360.0;
const FIELD_MARGIN_MM: f32 = 150.0;
const WALL_SPEED_MM_S: u32 = 3000;

// Places AI-selected defenders in a compact wall between the ball and our goal.
pub fn goalie_wall<C: Communication, A: Ai + Send>(cp: &mut CrashPilot<C, A>) {
  let mut defender_ids = std::mem::take(&mut cp.state.defenders);
  if defender_ids.is_empty() {
    return;
  }

  defender_ids.sort_unstable();
  defender_ids.dedup();

  if let Some(goalie) = cp.state.goalie {
    defender_ids.retain(|id| *id != goalie);
  }

  defender_ids.retain(|id| cp.robots.contains_key(&(*id as u32)));
  if defender_ids.is_empty() {
    return;
  }

  let side = if cp.state.site.abs() > f32::EPSILON {
    cp.state.site.signum()
  } else {
    1f32
  };

  let half_width = cp.field_setup.width as f32 * 0.5;
  let half_height = cp.field_setup.height as f32 * 0.5;
  let own_goal = Vec2::new(-side * half_width, 0.0);
  let ball = cp.state.ball.ball.pos;

  let mut goal_to_ball = ball - own_goal;
  if goal_to_ball.norm_squared() <= f32::EPSILON {
    goal_to_ball = Vec2::new(side, 0.0);
  }

  let direction = goal_to_ball.normalized();
  let wall_depth = WALL_DEPTH_MM.min((half_width - FIELD_MARGIN_MM).max(FIELD_MARGIN_MM));
  let anchor = own_goal + direction * wall_depth;
  let tangent = Vec2::new(-direction.y, direction.x);

  let mut defenders: Vec<(u8, f32)> = defender_ids
    .into_iter()
    .map(|id| {
      let projection = cp
        .state
        .robots_self
        .iter()
        .find(|robot| robot.robot_id == id)
        .and_then(|robot| robot.pos)
        .map(|pos| (pos - anchor).dot(&tangent))
        .unwrap_or_default();

      (id, projection)
    })
    .collect();

  defenders.sort_by(|a, b| a.1.total_cmp(&b.1));

  let center_index = defenders.len().saturating_sub(1) as f32 * 0.5;

  for (idx, (robot_id, _)) in defenders.into_iter().enumerate() {
    let offset = (idx as f32 - center_index) * WALL_SPACING_MM;
    let target = clamp_to_field(anchor + tangent * offset, half_width, half_height);
    let face_ball = ball - target;

    if let Some(robot) = cp.robots.get_mut(&(robot_id as u32)) {
      robot.msg.cmd.state = StateFree as i32;
      robot.msg.cmd.task = TaskPos as i32;
      robot.msg.cmd.pos = Some(target.to_cp_vec2());
      robot.msg.cmd.speed = Some(WALL_SPEED_MM_S);
      robot.msg.cmd.orientation = Some(face_ball.angle_in_u16() as u32);
    }
  }
}

fn clamp_to_field(pos: Vec2<f32>, half_width: f32, half_height: f32) -> Vec2<f32> {
  Vec2::new(
    pos
      .x
      .clamp(-half_width + FIELD_MARGIN_MM, half_width - FIELD_MARGIN_MM),
    pos.y.clamp(
      -half_height + FIELD_MARGIN_MM,
      half_height - FIELD_MARGIN_MM,
    ),
  )
}
