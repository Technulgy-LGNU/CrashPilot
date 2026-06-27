use crate::RobotData;
use crate::game_logic::WorldState;
use crate::game_logic::types::Robot;
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::utils::FieldSetup;
use core_dump::proto::CpState::{StateFree, StateGoalie, StateHalt};
use core_dump::proto::CpTask::{TaskDribble, TaskKick, TaskPos, TaskPosBall, TaskRecKick};
use core_dump::proto::{CpCommand, CpTests, CpVector2};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

const TEST_SPEED: u32 = 1500;
const DRIBBLER_SPEED: u32 = 200;
const KICK_SPEED: u32 = 200;
const TARGET_CHANGE_PERIOD_MS: u64 = 8_000;

#[inline]
pub fn mode_test(
  robot_data: &mut HashMap<u32, RobotData>,
  state: &mut WorldState,
  field_setup: &FieldSetup,
) {
  clear_commands(robot_data);

  let all_robots: Vec<Robot> = state
    .robots_self
    .iter()
    .chain(state.robots_opp.iter())
    .cloned()
    .collect();

  match CpTests::try_from(state.iface_cmd.test.test).unwrap_or_default() {
    CpTests::TestNone => {
      for robot in robot_data.values_mut() {
        robot.msg.cmd.state = StateHalt as i32;
        robot.msg.cmd.speed = Some(0);
      }
    }
    CpTests::TestBallControl => {
      for_each_selected_robot(robot_data, state, |robot, _, _| {
        robot.msg.cmd = free_command(TaskDribble as i32, None, Some(DRIBBLER_SPEED));
      });
    }
    CpTests::TestDribbler => {
      for_each_selected_robot(robot_data, state, |robot, idx, _| {
        let target = random_target(state, idx, 0);
        robot.msg.cmd = free_command(TaskPosBall as i32, Some(target), Some(TEST_SPEED));
      });
    }
    CpTests::TestKicker => {
      let selected = selected_robot_ids(robot_data, state);

      match selected.as_slice() {
        [] => {}
        [robot_id] => {
          if let Some(robot) = robot_data.get_mut(robot_id) {
            let target = random_target(state, 0, 1);
            robot.msg.cmd = free_command(TaskPosBall as i32, Some(target), Some(TEST_SPEED));
          }
        }
        [first_id, second_id, ..] => {
          let first_target = random_target(state, 0, 2);
          let second_target = random_target(state, 1, 3);

          let first_pos = robot_position(state, *first_id).unwrap_or(first_target);
          let second_pos = robot_position(state, *second_id).unwrap_or(second_target);
          let first_has_ball = robot_data
            .get(first_id)
            .map(|robot| robot.feedback.has_ball)
            .unwrap_or_default();
          let second_has_ball = robot_data
            .get(second_id)
            .map(|robot| robot.feedback.has_ball)
            .unwrap_or_default();

          if let Some(first) = robot_data.get_mut(first_id) {
            first.msg.cmd = if first_has_ball {
              kick_command(first_pos, second_pos)
            } else {
              free_command(TaskPosBall as i32, Some(first_target), Some(TEST_SPEED))
            };
          }

          if let Some(second) = robot_data.get_mut(second_id) {
            second.msg.cmd = if first_has_ball {
              free_command(TaskRecKick as i32, Some(second_target), Some(TEST_SPEED))
            } else if second_has_ball {
              kick_command(second_pos, first_pos)
            } else {
              free_command(TaskPos as i32, Some(second_target), Some(TEST_SPEED))
            };
          }
        }
      }
    }
    CpTests::ModeGoalShoot => {
      // The selected robot should shoot towards the goal
      if let Some(&robot_id) = state.iface_cmd.test.robot_ids.first() {
        let robot_self = state
          .robots_self
          .iter()
          .find(|r| r.robot_id == robot_id as u8);
        if let (Some(robot_self), Some(robot_data)) = (robot_self, robot_data.get_mut(&robot_id)) {
          shoot_to_goal(robot_data, robot_self, &all_robots, state, field_setup);
          robot_data.msg.cmd.state = StateFree as i32;
          robot_data.msg.cmd.speed = Some(400);

          dbg!(robot_data.msg.cmd.kick_orient);
          dbg!(state.site);
        }
      }
    }
    CpTests::ModeGoalie => {
      // Put the selected robot into goalie mode
      if let Some(&robot_id) = state.iface_cmd.test.robot_ids.first()
        && let Some(robot_data) = robot_data.get_mut(&robot_id)
      {
        robot_data.msg.cmd.state = StateGoalie as i32;
      }
    }
    CpTests::ModeGoalieAndShoot => {
      // One robot gets goalie, the other shoots, into the goal, if it gets the ball
      // Put the selected robot into goalie mode
      if state.iface_cmd.test.robot_ids.len() >= 2 {
        let goalie_id = state.iface_cmd.test.robot_ids[0];
        let shooter_id = state.iface_cmd.test.robot_ids[1];

        if let Some(robot_goalie_data) = robot_data.get_mut(&goalie_id) {
          robot_goalie_data.msg.cmd.state = StateGoalie as i32;
        }

        let robot_self = state
          .robots_self
          .iter()
          .find(|r| r.robot_id == shooter_id as u8);
        if let (Some(robot_self), Some(robot_shooter_data)) =
          (robot_self, robot_data.get_mut(&shooter_id))
        {
          shoot_to_goal(
            robot_shooter_data,
            robot_self,
            &all_robots,
            state,
            &FieldSetup::default(),
          );
          robot_shooter_data.msg.cmd.state = StateFree as i32;
        }
      }
    }
  }
}

fn clear_commands(robot_data: &mut HashMap<u32, RobotData>) {
  for robot in robot_data.values_mut() {
    robot.msg.cmd = CpCommand::default();
    robot.msg.cmd.state = StateFree as i32;
  }
}

fn for_each_selected_robot(
  robot_data: &mut HashMap<u32, RobotData>,
  state: &WorldState,
  mut apply: impl FnMut(&mut RobotData, usize, u32),
) {
  for (idx, robot_id) in selected_robot_ids(robot_data, state)
    .into_iter()
    .enumerate()
  {
    if let Some(robot) = robot_data.get_mut(&robot_id) {
      apply(robot, idx, robot_id);
    }
  }
}

fn selected_robot_ids(robot_data: &HashMap<u32, RobotData>, state: &WorldState) -> Vec<u32> {
  let mut ids: Vec<u32> = state
    .iface_cmd
    .test
    .robot_ids
    .iter()
    .copied()
    .filter(|id| robot_data.contains_key(id))
    .collect();

  if ids.is_empty() {
    ids = robot_data.keys().copied().collect();
    ids.sort_unstable();
  }

  ids
}

fn free_command(task: i32, pos: Option<CpVector2>, speed: Option<u32>) -> CpCommand {
  CpCommand {
    state: StateFree as i32,
    task,
    pos,
    speed,
    ..Default::default()
  }
}

fn kick_command(from: CpVector2, to: CpVector2) -> CpCommand {
  let dx = (to.x - from.x) as f32;
  let dy = (to.y - from.y) as f32;

  CpCommand {
    state: StateFree as i32,
    task: TaskKick as i32,
    kick_orient: Some(angle_to_u16(dx, dy) as u32),
    kick_speed: Some(KICK_SPEED),
    ..Default::default()
  }
}

fn robot_position(state: &WorldState, robot_id: u32) -> Option<CpVector2> {
  state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == robot_id as u8)
    .and_then(|robot| robot.pos)
    .map(|pos| CpVector2 {
      x: pos.x as i32,
      y: pos.y as i32,
    })
}

fn random_target(state: &WorldState, robot_idx: usize, salt: u64) -> CpVector2 {
  let half_width = field_half_width(state);
  let half_height = field_half_height(state);
  let bucket = now_ms() / TARGET_CHANGE_PERIOD_MS;
  let seed = splitmix64(bucket ^ ((robot_idx as u64) << 32) ^ salt);
  let x = scale_to_range(seed, -half_width, half_width);
  let y = scale_to_range(splitmix64(seed), -half_height, half_height);

  CpVector2 { x, y }
}

fn field_half_width(state: &WorldState) -> i32 {
  state
    .robots_self
    .iter()
    .chain(state.robots_opp.iter())
    .filter_map(|robot| robot.pos)
    .map(|pos| pos.x.abs() as i32 + 1_000)
    .max()
    .unwrap_or(4_000)
    .clamp(1_000, 4_500)
}

fn field_half_height(state: &WorldState) -> i32 {
  state
    .robots_self
    .iter()
    .chain(state.robots_opp.iter())
    .filter_map(|robot| robot.pos)
    .map(|pos| pos.y.abs() as i32 + 1_000)
    .max()
    .unwrap_or(2_500)
    .clamp(1_000, 3_000)
}

#[inline]
fn now_ms() -> u64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|duration| duration.as_millis() as u64)
    .unwrap_or_default()
}

fn splitmix64(mut value: u64) -> u64 {
  value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
  value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
  value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
  value ^ (value >> 31)
}

fn scale_to_range(value: u64, min: i32, max: i32) -> i32 {
  let span = (max - min).max(1) as u64;
  min + (value % span) as i32
}

fn angle_to_u16(dx: f32, dy: f32) -> u16 {
  let mut angle = dy.atan2(dx) / std::f32::consts::TAU;
  if angle < 0.0 {
    angle += 1.0;
  }

  (angle * u16::MAX as f32) as u16
}
