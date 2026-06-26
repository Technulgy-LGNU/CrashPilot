use crate::game_logic::ai_handler::ai_handler;
use crate::game_logic::defend::goalie_wall;
use crate::game_logic::types::{GamePhase, PrepPhase, PrepTask, PrepTaskStatus, Robot};
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::{CommunicationChannels, CrashPilot, RobotData};
use artificial_incompetence::types::Ai;
use core_dump::proto::CpState::{StateFree, StateGoalie, StateHalt, StateStop};
use core_dump::proto::CpTask::{StateFreekick, StateKickoff, TaskKick, TaskPos, TaskPosBall};
use core_dump::proto::CpVector2;
use core_dump::vec::types::Vec2;

const PREP_SETUP_SPEED_MM_S: u32 = 1500;
const RESTART_KICK_SPEED: u32 = 200;
const KICKER_SETUP_DISTANCE_MM: f32 = 320.0;

#[inline]
pub fn mode_game<A: Ai + Send>(cp: &mut CrashPilot<CommunicationChannels, A>) {
  let all_robots: Vec<Robot> = cp
    .state
    .robots_self
    .iter()
    .chain(cp.state.robots_opp.iter())
    .cloned()
    .collect();

  // Give the goalie the goalie command and check if the goalie has changed
  #[cfg(feature = "ssl_game_controller")]
  if let (Some(current_goalie), Some(new_goalie)) = (cp.state.goalie, cp.state.new_goalie) {
    // Use `unwrap_or(32)` because the max id is 15
    if current_goalie != new_goalie && new_goalie != cp.state.last_requested_goalie.unwrap_or(32) {
      let gc = cp.comm.gc.clone();
      tokio::spawn(async move {
        if let Err(err) = gc.desired_keeper(new_goalie as i32).await {
          eprintln!("Failed to request new goalie {new_goalie}: {err:#}");
        }
      });
      cp.state.last_requested_goalie = Some(new_goalie);
    }
  }

  if handle_prep_task(cp, &all_robots) {
    goalie_wall(cp);
    return;
  }

  match cp.state.phase {
    GamePhase::Halted | GamePhase::Unknown => {
      set_robot_state_for_all(cp, StateHalt as i32);
    }
    GamePhase::Stopped => {
      set_robot_state_for_all(cp, StateStop as i32);
    }
    GamePhase::Running => {
      set_goalie(cp);
      ai_handler(&all_robots, cp);
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

  // Execute the goalie wall
  goalie_wall(cp);
}

fn handle_prep_task<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  all_robots: &[Robot],
) -> bool {
  let Some(task) = cp.state.prep_task else {
    return false;
  };

  match task.status {
    PrepTaskStatus::Preparing => {
      set_robot_state_for_all(cp, StateStop as i32);
      set_goalie(cp);

      if task.phase.is_offensive() {
        prepare_offensive_restart(cp, task);
      }

      true
    }
    PrepTaskStatus::Ready => {
      if task.phase.is_offensive() {
        execute_offensive_restart(cp, task, all_robots);
      } else {
        set_robot_state_for_all(cp, StateStop as i32);
        set_goalie(cp);
      }

      true
    }
  }
}

fn prepare_offensive_restart<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
) {
  let Some(actor_id) = prep_actor_id(cp, task) else {
    return;
  };

  cp.state.mark_prep_actor(actor_id);

  let target = setup_position_for_restart(cp, task);
  let orientation = kick_direction_for_restart(cp, task).angle_in_u16() as u32;

  if let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) {
    robot.msg.cmd.state = StateStop as i32;
    robot.msg.cmd.task = TaskPos as i32;
    robot.msg.cmd.pos = Some(target.to_cp_vec2());
    robot.msg.cmd.speed = Some(PREP_SETUP_SPEED_MM_S);
    robot.msg.cmd.orientation = Some(orientation);
  }
}

fn execute_offensive_restart<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
  all_robots: &[Robot],
) {
  let Some(actor_id) = prep_actor_id(cp, task) else {
    return;
  };

  cp.state.mark_prep_actor(actor_id);

  if matches!(task.phase, PrepPhase::OffensivePenalty) {
    execute_penalty(cp, actor_id, all_robots);
  } else {
    execute_kick_restart(cp, task, actor_id);
  }

  cp.state.mark_prep_acted();
}

fn execute_penalty<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  actor_id: u8,
  all_robots: &[Robot],
) {
  let Some(robot_state) = cp
    .state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == actor_id)
  else {
    return;
  };

  let Some(mut robot_msg) = cp.robots.get(&(actor_id as u32)).cloned() else {
    return;
  };

  robot_msg.msg.cmd.state = StateFree as i32;
  shoot_to_goal(&mut robot_msg, robot_state, all_robots, cp);
  cp.robots.insert(actor_id as u32, robot_msg);
}

fn execute_kick_restart<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
  actor_id: u8,
) {
  let kick_task = match task.phase {
    PrepPhase::OffensiveKickoff => StateKickoff,
    PrepPhase::OffensiveFreeKick => StateFreekick,
    _ => TaskKick,
  };
  let kick_orientation = kick_direction_for_restart(cp, task).angle_in_u16() as u32;

  if let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) {
    robot.msg.cmd.state = StateFree as i32;
    robot.msg.cmd.task = kick_task as i32;
    robot.msg.cmd.kick_orient = Some(kick_orientation);
    robot.msg.cmd.kick_speed = Some(RESTART_KICK_SPEED);
  }
}

fn prep_actor_id<A: Ai + Send>(
  cp: &CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
) -> Option<u8> {
  task
    .acting_robot
    .or(cp.state.acting_robot)
    .filter(|robot_id| is_available_field_robot(cp, *robot_id))
    .or_else(|| closest_field_robot_to_ball(cp))
}

fn closest_field_robot_to_ball<A: Ai + Send>(
  cp: &CrashPilot<CommunicationChannels, A>,
) -> Option<u8> {
  cp.state
    .robots_self
    .iter()
    .filter(|robot| is_available_field_robot(cp, robot.robot_id))
    .min_by(|a, b| {
      a.distance_ball
        .unwrap_or(f32::MAX)
        .total_cmp(&b.distance_ball.unwrap_or(f32::MAX))
    })
    .map(|robot| robot.robot_id)
}

fn is_available_field_robot<A: Ai + Send>(
  cp: &CrashPilot<CommunicationChannels, A>,
  robot_id: u8,
) -> bool {
  cp.robots.contains_key(&(robot_id as u32)) && cp.state.goalie != Some(robot_id)
}

fn setup_position_for_restart<A: Ai + Send>(
  cp: &CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
) -> Vec2<f32> {
  task.ball_pos - kick_direction_for_restart(cp, task).normalized() * KICKER_SETUP_DISTANCE_MM
}

fn kick_direction_for_restart<A: Ai + Send>(
  cp: &CrashPilot<CommunicationChannels, A>,
  task: PrepTask,
) -> Vec2<f32> {
  let opponent_goal = Vec2::new(cp.field_setup.width as f32 * 0.5 * cp.state.site, 0.0);
  let direction = opponent_goal - task.ball_pos;

  if direction.norm_squared() <= f32::EPSILON {
    Vec2::new(cp.state.site, 0.0)
  } else {
    direction
  }
}

fn set_robot_state_for_all<A: Ai + Send>(
  cp: &mut CrashPilot<CommunicationChannels, A>,
  state: i32,
) {
  for robot in cp.robots.values_mut() {
    robot.msg.cmd.state = state;
  }
}

fn set_goalie<A: Ai + Send>(cp: &mut CrashPilot<CommunicationChannels, A>) {
  if let Some(goalie) = cp.state.goalie
    && let Some(robot) = cp.robots.get_mut(&(goalie as u32))
  {
    robot.msg.cmd.state = StateGoalie as i32;
  }
}
