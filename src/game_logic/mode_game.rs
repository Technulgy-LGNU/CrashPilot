use crate::game_logic::ai_handler::ai_handler;
use crate::game_logic::types::{GamePhase, Robot};
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::{CrashPilot, RobotData};
use artificial_incompetence::types::Ai;
use core_dump::proto::CpState::{StateHalt, StateStop};
use core_dump::proto::CpTask::{TaskChip, TaskPos, TaskPosBall, TaskRecKick};
use core_dump::proto::CpVector2;

#[inline]
pub fn mode_game<C, A: Ai>(cp: &mut CrashPilot<C, A>) {
  let all_robots: Vec<Robot> = cp
    .state
    .robots_self
    .iter()
    .chain(cp.state.robots_opp.iter())
    .cloned()
    .collect();

  match cp.state.phase {
    GamePhase::UNKNOWN => {
      for robot in cp.robots.values_mut() {
        robot.msg.cmd.state = StateHalt as i32;
      }
    }
    GamePhase::Halted => {
      for robot in cp.robots.values_mut() {
        robot.msg.cmd.state = StateHalt as i32;
      }
    }
    GamePhase::Stopped => {
      for robot in cp.robots.values_mut() {
        robot.msg.cmd.state = StateStop as i32;
      }
    }
    GamePhase::OffensiveKickoff => {}
    GamePhase::DefensiveKickoff => {}
    GamePhase::OffensivePenalty => {}
    GamePhase::DefensivePenalty => {}
    GamePhase::OffensiveFreeKick => {}
    GamePhase::DefensiveFreeKick => {}
    GamePhase::Running => {
      // First check if the goalie has the ball
      if cp.state.goalie.is_some() {
        match cp
          .robots
          .get(&(cp.state.goalie.unwrap_or_default() as u32))
          .cloned()
        {
          None => (),
          Some(mut goalie_robot) => {
            if goalie_robot.feedback.has_ball {
              // Get the state.robot
              let goalie_robot_state: &Robot = match cp
                .state
                .robots_self
                .iter()
                .find(|r| r.robot_id == cp.state.goalie.unwrap_or_default())
              {
                None => return,
                Some(robot) => robot,
              };

              // Chip to robot the furthest away
              if goalie_robot_state.distance_team.len() >= 2 {
                let to_robot_id = goalie_robot_state
                  .distance_team
                  .iter()
                  .max_by(|a, b| a.1.partial_cmp(b.1).unwrap());
                match to_robot_id {
                  Some(to_robot_id) => {
                    // Get the robots
                    let mut to_robot_msg: RobotData =
                      match cp.robots.get(&(*to_robot_id.0 as u32)).cloned() {
                        None => return,
                        Some(robot) => robot,
                      };
                    let to_robot_state: &Robot = match cp
                      .state
                      .robots_self
                      .iter()
                      .find(|r| r.robot_id == *to_robot_id.0)
                    {
                      None => return,
                      Some(robot) => robot,
                    };

                    // Get the direction to that robot
                    goalie_robot.msg.cmd.kick_orient = Option::from(
                      (to_robot_state.pos.unwrap_or_default()
                        + goalie_robot_state.pos.unwrap_or_default())
                      .angle_in_u16() as u32,
                    );

                    goalie_robot.msg.cmd.task = TaskChip as i32;
                    goalie_robot.msg.cmd.kick_speed = Some(255);

                    to_robot_msg.msg.cmd.task = TaskRecKick as i32;

                    // Insert this command back into the robots hashmap
                    cp.robots.insert(goalie_robot.msg.robot_id, goalie_robot);
                    cp.robots.insert(to_robot_msg.msg.robot_id, to_robot_msg);
                  }
                  None => {
                    // Chip to goal
                    shoot_to_goal(&mut goalie_robot, goalie_robot_state, &all_robots, cp);
                  }
                }
              } else {
                // Chip to goal
                shoot_to_goal(&mut goalie_robot, goalie_robot_state, &all_robots, cp);
              }
            } else {
              ai_handler(&all_robots, cp);
            }
          }
        }
      } else {
        ai_handler(&all_robots, cp);
      }

      // Do goalie wall math
    }
    GamePhase::Timeout => {
      // Place all the robots in a line, defined in the config file
      // Max Speed 1500mm/s
      for robot in cp.robots.iter_mut() {
        robot.1.msg.cmd.task = TaskPos as i32;
        robot.1.msg.cmd.speed = Some(1500);

        let pos: CpVector2 = match cp.config.robots.get(&robot.1.msg.robot_id) {
          None => CpVector2 { x: 0, y: 0 },
          Some(r) => r.substitution_pos.to_cp_vec2(),
        };
        robot.1.msg.cmd.pos = Some(pos);
      }
    }
    GamePhase::BallPlacement => {
      // Get the robot closes to the ball and give it the ball placement command
      let robot_closest_ball: &Robot = match cp.state.robots_self.iter().min_by(|a, b| {
        a.distance_ball
          .unwrap_or(10000f32)
          .total_cmp(&b.distance_ball.unwrap_or(10000f32))
      }) {
        None => return,
        Some(robot) => robot,
      };

      let mut robot_msg: RobotData = match cp
        .robots
        .get(&(robot_closest_ball.robot_id as u32))
        .cloned()
      {
        None => return,
        Some(r) => r,
      };

      let ball_pos = match cp.packet_buffer.referee.designated_position {
        None => {
          return;
        }
        Some(pos) => pos,
      };

      robot_msg.msg.cmd.task = TaskPosBall as i32;
      robot_msg.msg.cmd.speed = Some(1500);
      robot_msg.msg.cmd.pos = Some(CpVector2 {
        x: ball_pos.x as i32,
        y: ball_pos.y as i32,
      });
    }
  }
}
