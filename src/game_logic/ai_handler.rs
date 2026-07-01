use crate::CrashPilot;
use crate::game_logic::types::Robot;
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::helpers::compensated_kick_direction;
use crate::utils::FieldSetup;
use core_dump::proto::CpCommand;
use core_dump::proto::CpState::{StateFree, StateGoalie};
use core_dump::proto::CpTask::{
  TaskBlock, TaskDribble, TaskKick, TaskPos, TaskPosBall, TaskRecKick, TaskSteal,
};
use core_dump::types::{Ai, RobotCommand};
use core_dump::vec::types::Vec2;

#[inline]
pub fn ai_handler<C, A: Ai>(all_robots: &[Robot], cp: &mut CrashPilot<C, A>) {
  // AI does it thing
  let commands = cp.ai.predict(&cp.ai_data, cp.logic_dt());
  let planned_receive_kicks =
    planned_receive_kicks(&commands, &cp.state.robots_self, cp.state.ball.ball.pos);
  #[cfg(feature = "viewer-debug")]
  {
    cp.last_ai_commands = commands;
  }
  // Convert the commands to robot commands
  if !commands.is_empty() {
    for (id, command) in commands.iter().enumerate() {
      if let Some(command) = command
        && let Some(goalie) = cp.state.goalie
        && (goalie != id as u8 || goalie_ai_command_allowed(*command))
      {
        // Get the robot
        let Some(robot) = cp.robots.get_mut(&(id as u32)) else {
          continue;
        };
        let robot_self = match cp.state.robots_self.iter().find(|r| r.robot_id == id as u8) {
          None => {
            return;
          }
          Some(robot) => robot,
        };

        if *robot != crate::RobotData::default() {
          robot.msg.cmd.state = StateFree as i32;
          match command {
            RobotCommand::Pos(pos) => {
              set_pos_command(&mut robot.msg.cmd, *pos, None, None, cp.field_setup);
            }
            RobotCommand::PosSpeed(pos, speed) => {
              set_pos_command(&mut robot.msg.cmd, *pos, Some(*speed), None, cp.field_setup);
            }
            RobotCommand::PosFace(pos, orient) => {
              set_pos_command(
                &mut robot.msg.cmd,
                *pos,
                None,
                Some(*orient),
                cp.field_setup,
              );
            }
            RobotCommand::PosFaceSpeed(pos, orient, speed) => {
              set_pos_command(
                &mut robot.msg.cmd,
                *pos,
                Some(*speed),
                Some(*orient),
                cp.field_setup,
              );
            }

            RobotCommand::Kick(orient) => {
              robot.msg.cmd.task = TaskKick as i32;
              robot.msg.cmd.kick_orient = Option::from(*orient as u32);
              robot.msg.cmd.kick_speed = Option::from(255);
            }
            RobotCommand::Chip(orient) => {
              robot.msg.cmd.task = TaskKick as i32;
              robot.msg.cmd.kick_orient = Option::from(*orient as u32);
              robot.msg.cmd.kick_speed = Option::from(200);
            }
            RobotCommand::RecKick(_) => {
              robot.msg.cmd.task = TaskRecKick as i32;
            }
            RobotCommand::Steal => {
              robot.msg.cmd.task = TaskSteal as i32;
              robot.msg.cmd.speed = Option::from(4000);
              let from = robot_self.pos.unwrap_or_default();
              let ball = cp.state.ball.ball.pos;
              robot.msg.cmd.orientation = Option::from((ball - from).angle_in_u16() as u32);
            }
            RobotCommand::Dribble(pos) => {
              robot.msg.cmd.task = TaskDribble as i32;
              let target =
                *pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32);
              robot.msg.cmd.pos = Option::from(target.to_cp_vec2());
              robot.msg.cmd.speed = Option::from(2000);
              // The firmware uses cmd.orientation as the direction to push the
              // ball; aim it from the ball toward the dribble target.
              robot.msg.cmd.orientation =
                Option::from((target - cp.state.ball.ball.pos).angle_in_u16() as u32);
            }
            RobotCommand::PosBall(pos) => {
              robot.msg.cmd.task = TaskPosBall as i32;
              let target =
                *pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32);
              robot.msg.cmd.pos = Option::from(target.to_cp_vec2());
              robot.msg.cmd.speed = Option::from(2000);
              robot.msg.cmd.orientation =
                Option::from((target - cp.state.ball.ball.pos).angle_in_u16() as u32);
            }
            RobotCommand::Kickoff(_) => {}
            RobotCommand::FreeKick(_) => {}
            RobotCommand::KickGoal => {
              // Calculate the angle to the goal with no opponents in the way and the minimum distance from all robots
              shoot_to_goal(robot, robot_self, all_robots, &cp.state, &cp.field_setup)
            }
            RobotCommand::PassTo(r_id) => {
              robot.msg.cmd.task = TaskKick as i32;

              if let Some(to_robot) = cp.state.robots_self.iter().find(|r| r.robot_id == *r_id) {
                if let Some(plan) =
                  planned_pass_from_robots(robot_self, to_robot, cp.state.ball.ball.pos)
                {
                  robot.msg.cmd.kick_orient = Option::from(plan.kick_orient);
                  robot.msg.cmd.kick_speed = Option::from(plan.kick_speed);
                }
              } else {
                // Shoot to goal
                shoot_to_goal(robot, robot_self, all_robots, &cp.state, &cp.field_setup);
              }
            }
            RobotCommand::RecPass => {
              robot.msg.cmd.task = TaskRecKick as i32;
              robot.msg.cmd.kick_orient = None;
              robot.msg.cmd.kick_speed = None;
              if let Some(plan) = planned_receive_kick(&planned_receive_kicks, id as u8) {
                robot.msg.cmd.kick_orient = Option::from(plan.kick_orient);
                robot.msg.cmd.kick_speed = Option::from(plan.kick_speed);
              }
            }
            RobotCommand::GoalWall => {
              // Add the robot to the defense struct, so the crashpilot can create a wall
              cp.state.defenders.push(robot.msg.robot_id as u8)
            }
            RobotCommand::GoalieGuard => {
              robot.msg.cmd.state = StateGoalie as i32;
              robot.msg.cmd.task = TaskBlock as i32;
              robot.msg.cmd.enemy_id = None;
              robot.msg.cmd.speed = Option::from(4000);
            }
            RobotCommand::Hold => {
              robot.msg.cmd.speed = Option::from(0);
            }
          }
        }
      }
    }
  }
}

fn goalie_ai_command_allowed(command: RobotCommand) -> bool {
  matches!(command, RobotCommand::GoalieGuard | RobotCommand::PassTo(_))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PlannedPass {
  kick_orient: u32,
  kick_speed: u32,
}

fn planned_receive_kicks(
  commands: &[Option<RobotCommand>],
  own_robots: &[Robot],
  ball_pos: Vec2<f32>,
) -> Vec<(u8, PlannedPass)> {
  commands
    .iter()
    .enumerate()
    .filter_map(|(from_id, command)| {
      let Some(RobotCommand::PassTo(receiver_id)) = command else {
        return None;
      };
      let passer = own_robots
        .iter()
        .find(|robot| robot.robot_id == from_id as u8)?;
      let receiver = own_robots
        .iter()
        .find(|robot| robot.robot_id == *receiver_id)?;
      planned_pass_from_robots(passer, receiver, ball_pos).map(|plan| (*receiver_id, plan))
    })
    .collect()
}

fn planned_receive_kick(planned: &[(u8, PlannedPass)], receiver_id: u8) -> Option<PlannedPass> {
  planned
    .iter()
    .find_map(|(id, plan)| (*id == receiver_id).then_some(*plan))
}

fn planned_pass_from_robots(
  passer: &Robot,
  receiver: &Robot,
  ball_pos: Vec2<f32>,
) -> Option<PlannedPass> {
  // Aim the kicker along the direction to the receiver. Scale kick
  // power with pass distance so short passes do not overrun the
  // receiver and long ones still arrive.
  let robot_pos = passer.pos.unwrap_or_default();
  let from = kick_origin(robot_pos, ball_pos);
  let to = receiver.pos.unwrap_or_default();
  let base_dir = to - from;
  let base_dist = (base_dir.x * base_dir.x + base_dir.y * base_dir.y)
    .sqrt()
    .max(1.0);
  let receiver_vel = receiver.vel.unwrap_or_default();
  let lead_s = (base_dist / 4500.0).clamp(0.05, 0.22);
  let to = receiver_intake_target(from, to + receiver_vel * lead_s);
  let dir = to - from;
  let dist = (dir.x * dir.x + dir.y * dir.y).sqrt().max(1.0);
  let power = (dist * 0.06).clamp(90.0, 200.0) as u32;
  let compensated_dir = compensated_kick_direction(
    dir,
    passer.vel.unwrap_or_default(),
    passer.angular_vel,
    power,
  );
  Some(PlannedPass {
    kick_orient: compensated_dir.angle_in_u16() as u32,
    kick_speed: power,
  })
}

fn set_pos_command(
  cmd: &mut CpCommand,
  pos: Vec2<f32>,
  speed: Option<u32>,
  orientation: Option<u32>,
  fs: FieldSetup,
) {
  cmd.task = TaskPos as i32;
  cmd.pos = Some((pos * Vec2::new(fs.width as f32, fs.height as f32)).to_cp_vec2());
  cmd.speed = speed.or(Some(4000));
  cmd.orientation = orientation
}

#[inline]
fn kick_origin(robot_pos: Vec2<f32>, ball_pos: Vec2<f32>) -> Vec2<f32> {
  const MAX_CAPTURED_BALL_DIST_MM: f32 = 260.0;

  if (ball_pos - robot_pos).length() <= MAX_CAPTURED_BALL_DIST_MM {
    ball_pos
  } else {
    robot_pos
  }
}

#[inline]
fn receiver_intake_target(from: Vec2<f32>, receiver_center: Vec2<f32>) -> Vec2<f32> {
  const RECEIVER_CENTER_FROM_INTAKE_MM: f32 = 80.0;

  let pass = receiver_center - from;
  let pass_len = pass.length();
  if pass_len <= 1.0 {
    return receiver_center;
  }

  receiver_center - pass / pass_len * RECEIVER_CENTER_FROM_INTAKE_MM
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::game_logic::types::Team;

  #[test]
  fn receiver_intake_target_aims_before_receiver_center() {
    let from = Vec2::new(0.0, 0.0);
    let receiver = Vec2::new(1000.0, 0.0);

    assert_eq!(
      receiver_intake_target(from, receiver),
      Vec2::new(920.0, 0.0)
    );
  }

  #[test]
  fn planned_receive_kick_matches_pass_to_plan() {
    let robots = vec![
      test_robot(0, Vec2::new(0.0, 0.0), Vec2::new(0.0, 400.0)),
      test_robot(1, Vec2::new(1000.0, 250.0), Vec2::new(150.0, 0.0)),
    ];
    let ball_pos = Vec2::new(40.0, 0.0);
    let commands = vec![Some(RobotCommand::PassTo(1)), Some(RobotCommand::RecPass)];

    let receive_plan =
      planned_receive_kick(&planned_receive_kicks(&commands, &robots, ball_pos), 1)
        .expect("receiver should inherit the pass plan");
    let pass_to_plan = planned_pass_from_robots(&robots[0], &robots[1], ball_pos)
      .expect("passer should have a pass plan");

    assert_eq!(receive_plan, pass_to_plan);
  }

  fn test_robot(id: u8, pos: Vec2<f32>, vel: Vec2<f32>) -> Robot {
    Robot {
      robot_id: id,
      pos: Some(pos),
      vel: Some(vel),
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
}
