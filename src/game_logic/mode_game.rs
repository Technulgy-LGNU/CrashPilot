use crate::CrashPilot;
use crate::game_logic::types::GamePhase;
use crate::helpers::best_angle_to_goal::best_shot_angle;
use artificial_incompetence::types::RobotCommand;
use core_dump::proto::CpState::StateFree;
use core_dump::proto::CpTask::{
  TaskBlock, TaskDribble, TaskKick, TaskPos, TaskPosBall, TaskRecKick, TaskSteal,
};
use core_dump::vec::types::Vec2;

#[inline]
pub fn mode_game(cp: &mut CrashPilot) {
  match cp.state.phase {
    GamePhase::UNKNOWN => {}
    GamePhase::Halted => {}
    GamePhase::Stopped => {}
    GamePhase::OffensiveKickoff => {}
    GamePhase::DefensiveKickoff => {}
    GamePhase::OffensivePenalty => {}
    GamePhase::DefensivePenalty => {}
    GamePhase::OffensiveFreeKick => {}
    GamePhase::DefensiveFreeKick => {}
    GamePhase::Running => {
      // AI does it thing
      let commands = cp.ai.predict(&cp.ai_data, 1f32);
      // Convert the commands to robot commands
      if commands.len() > 0 {
        for (id, command) in commands.iter().enumerate() {
          if command.is_some() {
            // Get the robot
            let mut robot = cp.robots.get(&(id as u32)).unwrap_or_default().clone();
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
                  let side = if cp.packet_buffer.interface_command.game.side {
                    -1f32
                  } else {
                    1f32
                  };

                  let robot_self_pos = cp
                    .state
                    .robots
                    .iter()
                    .find(|r| {
                      r.robot_id == id as u8
                        && r.team
                          == if cp.packet_buffer.interface_command.game.team_color {
                            2
                          } else {
                            1
                          }
                    })
                    .unwrap_or_default()
                    .pos
                    .unwrap_or_default();
                  // Calculate the angle to the goal with no opponents in the way and the minimum distance from all robots
                  match best_shot_angle(
                    robot_self_pos,
                    &*cp.state.robots,
                    Vec2::new(
                      cp.field_setup.width as f32 * 0.5 * side,
                      cp.field_setup.goal_width as f32 * 0.5 * side,
                    ),
                    Vec2::new(
                      cp.field_setup.width as f32 * 0.5 * side,
                      (cp.field_setup.goal_width as f32 * -1f32) * 0.5 * side,
                    ),
                  ) {
                    None => {
                      // Try to shoot to the center
                      let angle = (robot_self_pos
                        + Vec2::new(cp.field_setup.width as f32 * side, 0f32))
                      .angle_in_u16();

                      robot.msg.cmd.task = TaskKick as i32;
                      robot.msg.cmd.kick_orient = Option::from(angle as u32);
                      robot.msg.cmd.kick_speed = Option::from(255);
                    }
                    Some(angle) => {
                      robot.msg.cmd.task = TaskKick as i32;
                      robot.msg.cmd.kick_orient = Option::from(angle);
                      robot.msg.cmd.kick_speed = Option::from(255);
                    }
                  };
                }
                RobotCommand::PassTo(r_id) => {
                  robot.msg.cmd.task = TaskKick as i32;

                  // Get the direction to that robot
                  for r in cp.state.robots {
                    if r.team
                      == if cp.packet_buffer.interface_command.game.team_color {
                        2
                      } else {
                        1
                      }
                      && r.robot_id == r_id
                    {
                      robot.msg.cmd.kick_orient = Option::from(
                        (r.pos.unwrap_or_default()
                          + cp
                            .state
                            .robots
                            .iter()
                            .find(|r| r.robot_id == id as u8)
                            .unwrap_or_default()
                            .pos
                            .unwrap_or_default())
                        .angle_in_u16() as u32,
                      );
                    }
                  }
                }
                RobotCommand::RecPass => {
                  robot.msg.cmd.task = TaskRecKick as i32;
                }
                RobotCommand::GoalWall => {
                  // Select three robots and build a wall between the ball and the  goal
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
  }
}
