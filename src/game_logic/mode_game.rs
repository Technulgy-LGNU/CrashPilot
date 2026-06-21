use crate::game_logic::ai_handler::ai_handler;
use crate::game_logic::types::{GamePhase, Robot};
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::{CrashPilot, RobotData};
use artificial_incompetence::types::Ai;
use core_dump::proto::CpTask::{TaskChip, TaskRecKick};

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
      // First check if the goalie has the ball
      if cp.state.goalie.is_some() {
        match cp
          .robots
          .get(&(cp.state.goalie.unwrap_or_default() as u32))
          .cloned()
        {
          None => return,
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
                if to_robot_id.is_some() {
                  // Get the robots
                  let mut to_robot_msg: RobotData =
                    match cp.robots.get(&(*to_robot_id.unwrap().0 as u32)).cloned() {
                      None => return,
                      Some(robot) => robot,
                    };
                  let to_robot_state: &Robot = match cp
                    .state
                    .robots_self
                    .iter()
                    .find(|r| r.robot_id == *to_robot_id.unwrap().0)
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
                } else {
                  // Chip to goal
                  shoot_to_goal(&mut goalie_robot, goalie_robot_state, &all_robots, cp);
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
  }
}
