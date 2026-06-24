use crate::game_logic::types::Robot;
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::CrashPilot;
use artificial_incompetence::{Ai, RobotCommand};
use core_dump::proto::CpState::StateFree;
use core_dump::proto::CpTask::{
  TaskBlock, TaskDribble, TaskKick, TaskPos, TaskPosBall, TaskRecKick, TaskSteal,
};
use core_dump::vec::types::Vec2;

#[inline]
pub fn ai_handler<C, A: Ai>(all_robots: &Vec<Robot>, cp: &mut CrashPilot<C, A>) {
  // AI does it thing
  let commands = cp.ai.predict(&cp.ai_data, 1f32);
  // Convert the commands to robot commands
  if commands.len() > 0 {
    for (id, command) in commands.iter().enumerate() {
      if command.is_some() {
        // Get the robot
        let mut robot = cp.robots.get(&(id as u32)).cloned().unwrap_or_default();
        let robot_self = match cp.state.robots_self.iter().find(|r| r.robot_id == id as u8) {
          None => {
            return;
          }
          Some(robot) => robot,
        };

        if robot != Default::default() {
          robot.msg.cmd.state = StateFree as i32;
          match command.unwrap_or_default() {
            RobotCommand::Pos(pos) => {
              robot.msg.cmd.task = TaskPos as i32;
              robot.msg.cmd.pos = Option::from(
                (pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32))
                  .to_cp_vec2(),
              );
              robot.msg.cmd.speed = Option::from(4000);
            }
            RobotCommand::Kick(orient) => {
              robot.msg.cmd.task = TaskKick as i32;
              robot.msg.cmd.kick_orient = Option::from((orient * 360f32) as u32);
              robot.msg.cmd.kick_speed = Option::from(200);
            }
            RobotCommand::Chip(orient) => {
              robot.msg.cmd.task = TaskKick as i32;
              robot.msg.cmd.kick_orient = Option::from((orient * 360f32) as u32);
              robot.msg.cmd.kick_speed = Option::from(200);
            }
            RobotCommand::RecKick(_) => {
              robot.msg.cmd.task = TaskRecKick as i32;
            }
            RobotCommand::Steal => {
              robot.msg.cmd.task = TaskSteal as i32;
              robot.msg.cmd.speed = Option::from(4000);
            }
            RobotCommand::Dribble(pos) => {
              robot.msg.cmd.task = TaskDribble as i32;
              robot.msg.cmd.pos = Option::from(
                (pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32))
                  .to_cp_vec2(),
              );
              robot.msg.cmd.speed = Option::from(2000);
            }
            RobotCommand::PosBall(pos) => {
              robot.msg.cmd.task = TaskPosBall as i32;
              robot.msg.cmd.pos = Option::from(
                (pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32))
                  .to_cp_vec2(),
              );
              robot.msg.cmd.speed = Option::from(2000);
            }
            RobotCommand::Kickoff(_) => {}
            RobotCommand::FreeKick(_) => {}
            RobotCommand::KickGoal => {
              // Calculate the angle to the goal with no opponents in the way and the minimum distance from all robots
              shoot_to_goal(&mut robot, robot_self, &all_robots, cp)
            }
            RobotCommand::PassTo(r_id) => {
              robot.msg.cmd.task = TaskKick as i32;

              let to_robot = match cp.state.robots_self.iter().find(|r| r.robot_id == r_id) {
                Some(r) => r,
                None => {
                  // Shoot to goal
                  shoot_to_goal(&mut robot, robot_self, &all_robots, cp);

                  return;
                }
              };
              // Get the direction to that robot
              robot.msg.cmd.kick_orient = Option::from(
                (to_robot.pos.unwrap_or_default() + robot_self.pos.unwrap_or_default())
                  .angle_in_u16() as u32,
              )
            }
            RobotCommand::RecPass => {
              robot.msg.cmd.task = TaskRecKick as i32;
            }
            RobotCommand::GoalWall => {
              // Add the robot to the defense struct, so the crashpilot can create a wall
              cp.state.defenders.push(robot.msg.robot_id as u8)
            }
            RobotCommand::GoalieGuard => {
              robot.msg.cmd.task = TaskBlock as i32;
              robot.msg.cmd.enemy_id = None;
              robot.msg.cmd.speed = Option::from(4000);
            }
            RobotCommand::Hold => {
              robot.msg.cmd.speed = Option::from(0);
            }
          }
          cp.robots.insert(id as u32, robot);
        }
      }
    }
  }
}
