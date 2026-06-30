use core_dump::proto::CpCommand;
use crate::CrashPilot;
use crate::game_logic::types::Robot;
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use core_dump::types::{Ai, RobotCommand};
use core_dump::proto::CpState::{StateFree, StateGoalie};
use core_dump::proto::CpTask::{
  TaskBlock, TaskDribble, TaskKick, TaskPos, TaskPosBall, TaskRecKick, TaskSteal,
};
use core_dump::vec::types::Vec2;
use crate::utils::FieldSetup;

#[inline]
pub fn ai_handler<C, A: Ai>(all_robots: &[Robot], cp: &mut CrashPilot<C, A>) {
  // AI does it thing
  let commands = cp.ai.predict(&cp.ai_data, 1f32);
  // Convert the commands to robot commands
  if !commands.is_empty() {
    for (id, command) in commands.iter().enumerate() {
      if let Some(command) = command
        && let Some(goalie) = cp.state.goalie
        && goalie != id as u8
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
              set_pos_command(&mut robot.msg.cmd, *pos, None, Some(*orient), cp.field_setup);
            }
            RobotCommand::PosFaceSpeed(pos, orient, speed) => {
              set_pos_command(&mut robot.msg.cmd, *pos, Some(*speed), Some(*orient), cp.field_setup);
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
            }
            RobotCommand::Dribble(pos) => {
              robot.msg.cmd.task = TaskDribble as i32;
              let target =
                *pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32);
              robot.msg.cmd.pos = Option::from(target.to_cp_vec2());
              robot.msg.cmd.speed = Option::from(2000);
              // The firmware uses cmd.orientation as the direction to push the
              // ball; aim it from the ball toward the dribble target.
              robot.msg.cmd.orientation = Option::from(
                (target - cp.state.ball.ball.pos * Vec2::new(1000f32, 1000f32)).angle_in_u16()
                  as u32,
              );
            }
            RobotCommand::PosBall(pos) => {
              robot.msg.cmd.task = TaskPosBall as i32;
              let target =
                *pos * Vec2::new(cp.field_setup.width as f32, cp.field_setup.height as f32);
              robot.msg.cmd.pos = Option::from(target.to_cp_vec2());
              robot.msg.cmd.speed = Option::from(2000);
              robot.msg.cmd.orientation = Option::from(
                (target - cp.state.ball.ball.pos * Vec2::new(1000f32, 1000f32)).angle_in_u16()
                  as u32,
              );
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
                // Aim the kicker along the direction to the receiver. Scale kick
                // power with pass distance so short passes do not overrun the
                // receiver and long ones still arrive.
                let from = robot_self.pos.unwrap_or_default();
                let to = to_robot.pos.unwrap_or_default();
                let base_dir = to - from;
                let base_dist =
                  ((base_dir.x * base_dir.x + base_dir.y * base_dir.y).sqrt()).max(1.0);
                let receiver_vel = to_robot.vel.unwrap_or_default();
                let lead_s = (base_dist / 4500.0).clamp(0.05, 0.22);
                let to = to + receiver_vel * lead_s;
                let dir = to - from;
                robot.msg.cmd.kick_orient = Option::from(dir.angle_in_u16() as u32);
                let dist = ((dir.x * dir.x + dir.y * dir.y).sqrt()).max(1.0);
                let power = (dist * 0.06).clamp(70.0, 200.0) as u32;
                robot.msg.cmd.kick_speed = Option::from(power);
              } else {
                // Shoot to goal
                shoot_to_goal(robot, robot_self, all_robots, &cp.state, &cp.field_setup);
              }
            }
            RobotCommand::RecPass => {
              robot.msg.cmd.task = TaskRecKick as i32;
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


fn set_pos_command(cmd: &mut CpCommand, pos: Vec2<f32>, speed: Option<u32>, orientation: Option<u32>, fs: FieldSetup) {
  cmd.task = TaskPos as i32;
  cmd.pos = Some(
    (pos * Vec2::new(fs.width as f32, fs.height as f32))
        .to_cp_vec2(),
  );
  cmd.speed = speed.or(Some(4000));
  cmd.orientation = orientation
}
