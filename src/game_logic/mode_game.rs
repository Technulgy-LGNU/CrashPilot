use crate::game_logic::ai_handler::ai_handler;
use crate::game_logic::defend::goalie_wall;
use crate::game_logic::types::{GamePhase, PrepPhase, PrepTask, PrepTaskStatus, Robot};
use crate::helpers::best_angle_to_goal::shoot_to_goal;
use crate::{Communication, CrashPilot};
use core_dump::proto::CpState::{StateFree, StateGoalie, StateHalt, StateStop};
use core_dump::proto::CpTask::{TaskKick, TaskPos, TaskPosBall, TaskRecKick};
use core_dump::proto::CpVector2;
use core_dump::types::Ai;
use core_dump::vec::types::Vec2;

const PREP_SETUP_SPEED_MM_S: u32 = 1500;
const RESTART_KICK_SPEED: u32 = 200;
const KICKOFF_PASS_SPEED: u32 = 50;
const KICKER_SETUP_DISTANCE_MM: f32 = 320.0;
const FREE_KICK_RECEIVER_SPEED_MM_S: u32 = 2200;
const FREE_KICK_SETUP_TOLERANCE_MM: f32 = 180.0;
const FREE_KICK_RECEIVER_TOLERANCE_MM: f32 = 350.0;
const FREE_KICK_RECEIVER_ADVANCE_MM: f32 = 2500.0;
const FREE_KICK_RECEIVER_MIN_OPPONENT_HALF_DEPTH_MM: f32 = 700.0;
const FREE_KICK_RECEIVER_GOAL_STANDOFF_MM: f32 = 1200.0;
const FREE_KICK_RECEIVER_CAPTURE_DISTANCE_MM: f32 = 220.0;
const FREE_KICK_RECEIVER_CAPTURE_SPEED_MM_S: f32 = 650.0;
const FREE_KICK_PASS_LEAD_SPEED_MM_S: f32 = 4500.0;

#[inline]
pub fn mode_game<C: Communication, A: Ai + Send>(cp: &mut CrashPilot<C, A>) {
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
      cp.comm.request_desired_keeper(new_goalie);
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

      let ball_pos = match cp.packet_buffer.referee.designated_position {
        None => {
          return;
        }
        Some(pos) => pos,
      };

      let Some(robot_msg) = cp.robots.get_mut(&(robot_closest_ball.robot_id as u32)) else {
        return;
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

fn handle_prep_task<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
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

fn prepare_offensive_restart<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
) {
  let Some(actor_id) = prep_actor_id(cp, task) else {
    return;
  };

  cp.state.mark_prep_actor(actor_id);

  let direction = prep_kick_direction(cp, task, actor_id);
  let target = setup_position_for_direction(task.ball_pos, direction);
  let orientation = direction.angle_in_u16() as u32;

  if let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) {
    robot.msg.cmd.state = StateStop as i32;
    robot.msg.cmd.task = TaskPos as i32;
    robot.msg.cmd.pos = Some(target.to_cp_vec2());
    robot.msg.cmd.speed = Some(PREP_SETUP_SPEED_MM_S);
    robot.msg.cmd.orientation = Some(orientation);
  }
}

fn execute_offensive_restart<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
  all_robots: &[Robot],
) {
  let Some(actor_id) = prep_actor_id(cp, task) else {
    return;
  };

  cp.state.mark_prep_actor(actor_id);

  if matches!(task.phase, PrepPhase::OffensivePenalty) {
    execute_penalty(cp, actor_id, all_robots);
    cp.state.mark_prep_acted();
  } else if matches!(task.phase, PrepPhase::OffensiveKickoff) {
    execute_kickoff(cp, task, actor_id);
  } else if matches!(task.phase, PrepPhase::OffensiveFreeKick) {
    execute_free_kick(cp, task, actor_id, all_robots);
  } else {
    execute_kick_restart(cp, task, actor_id);
    cp.state.mark_prep_acted();
  }
}

fn execute_penalty<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
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

  let Some(robot_msg) = cp.robots.get_mut(&(actor_id as u32)) else {
    return;
  };

  robot_msg.msg.cmd.state = StateFree as i32;
  shoot_to_goal(
    robot_msg,
    robot_state,
    all_robots,
    &cp.state,
    &cp.field_setup,
  );
}

fn execute_kickoff<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
) {
  let receiver_id = if task.has_acted {
    task
      .receiving_robot
      .filter(|robot_id| *robot_id != actor_id && is_available_field_robot(cp, *robot_id))
  } else {
    kickoff_receiver_id(cp, task, actor_id)
  };

  let Some(receiver_id) = receiver_id else {
    if task.has_acted {
      cp.state.clear_prep_task();
    } else {
      execute_kick_restart(cp, task, actor_id);
      cp.state.mark_prep_acted();
      cp.state.mark_prep_follow_up_acted(task.ball_pos);
    }
    return;
  };

  cp.state.mark_prep_receiver(receiver_id);

  let pass_direction = direction_to_robot_or_restart(cp, task, receiver_id);
  let actor_target = setup_position_for_direction(task.ball_pos, pass_direction);
  let actor_orientation = pass_direction.angle_in_u16() as u32;

  if task.has_acted {
    command_robot_pos(
      cp,
      actor_id,
      actor_target,
      actor_orientation,
      Some(0),
      StateStop as i32,
    );

    if receiver_ready_to_shoot(cp, receiver_id) {
      cp.state.clear_prep_task();
    } else {
      command_receiver_receive(cp, receiver_id, pass_direction);
    }

    return;
  }

  if command_kick_to_robot(cp, actor_id, receiver_id, task.ball_pos, KICKOFF_PASS_SPEED) {
    command_receiver_receive(cp, receiver_id, pass_direction);
    cp.state.mark_prep_acted();
  }
}

fn execute_free_kick<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
  all_robots: &[Robot],
) {
  let receiver_target = free_kick_receiver_target(cp, task);
  let Some(receiver_id) = prep_receiver_id(cp, task, actor_id, receiver_target) else {
    execute_kick_restart(cp, task, actor_id);
    cp.state.mark_prep_acted();
    cp.state.mark_prep_follow_up_acted(task.ball_pos);
    return;
  };

  cp.state.mark_prep_receiver(receiver_id);

  let pass_direction = direction_or_attack(cp, receiver_target - task.ball_pos);
  let actor_target = setup_position_for_direction(task.ball_pos, pass_direction);
  let actor_orientation = pass_direction.angle_in_u16() as u32;

  if task.follow_up_acted {
    command_robot_pos(
      cp,
      actor_id,
      actor_target,
      actor_orientation,
      Some(0),
      StateStop as i32,
    );
    command_receiver_shot(cp, receiver_id, all_robots);
    return;
  }

  if task.has_acted {
    command_robot_pos(
      cp,
      actor_id,
      actor_target,
      actor_orientation,
      Some(0),
      StateStop as i32,
    );

    if receiver_ready_to_shoot(cp, receiver_id) {
      if command_receiver_shot(cp, receiver_id, all_robots) {
        let ball_pos = cp.state.ball.ball.pos;
        cp.state.mark_prep_follow_up_acted(ball_pos);
      }
    } else {
      command_receiver_receive(cp, receiver_id, pass_direction);
    }

    return;
  }

  command_free_kick_receiver_setup(cp, receiver_id, receiver_target);

  let actor_ready = robot_is_near(cp, actor_id, actor_target, FREE_KICK_SETUP_TOLERANCE_MM);
  let receiver_ready = robot_is_near(
    cp,
    receiver_id,
    receiver_target,
    FREE_KICK_RECEIVER_TOLERANCE_MM,
  );

  if actor_ready && receiver_ready {
    if command_free_kick_pass(cp, actor_id, receiver_id, receiver_target, task.ball_pos) {
      command_receiver_receive(cp, receiver_id, pass_direction);
      cp.state.mark_prep_acted();
    }
  } else {
    command_robot_pos(
      cp,
      actor_id,
      actor_target,
      actor_orientation,
      Some(PREP_SETUP_SPEED_MM_S),
      StateFree as i32,
    );
  }
}

fn execute_kick_restart<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
) {
  let kick_orientation = kick_direction_for_restart(cp, task).angle_in_u16() as u32;

  if let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) {
    robot.msg.cmd.state = StateFree as i32;
    robot.msg.cmd.task = TaskKick as i32;
    robot.msg.cmd.kick_orient = Some(kick_orientation);
    robot.msg.cmd.kick_speed = Some(RESTART_KICK_SPEED);
  }
}

fn prep_actor_id<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
) -> Option<u8> {
  task
    .acting_robot
    .or(cp.state.acting_robot)
    .filter(|robot_id| is_available_field_robot(cp, *robot_id))
    .or_else(|| closest_field_robot_to_ball(cp))
}

fn kickoff_receiver_id<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
) -> Option<u8> {
  task
    .receiving_robot
    .filter(|robot_id| {
      *robot_id != actor_id
        && is_available_field_robot(cp, *robot_id)
        && robot_is_in_own_half(cp, *robot_id)
    })
    .or_else(|| {
      cp.state
        .robots_self
        .iter()
        .filter(|robot| {
          robot.robot_id != actor_id
            && is_available_field_robot(cp, robot.robot_id)
            && robot_pos_is_in_own_half(cp, robot.pos)
        })
        .min_by(|a, b| {
          distance_to_target(a, task.ball_pos).total_cmp(&distance_to_target(b, task.ball_pos))
        })
        .map(|robot| robot.robot_id)
    })
}

fn prep_receiver_id<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
  receiver_target: Vec2<f32>,
) -> Option<u8> {
  task
    .receiving_robot
    .filter(|robot_id| *robot_id != actor_id && is_available_field_robot(cp, *robot_id))
    .or_else(|| {
      cp.state
        .robots_self
        .iter()
        .filter(|robot| robot.robot_id != actor_id && is_available_field_robot(cp, robot.robot_id))
        .min_by(|a, b| {
          distance_to_target(a, receiver_target).total_cmp(&distance_to_target(b, receiver_target))
        })
        .map(|robot| robot.robot_id)
    })
}

fn closest_field_robot_to_ball<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
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

fn is_available_field_robot<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  robot_id: u8,
) -> bool {
  cp.robots.contains_key(&(robot_id as u32)) && cp.state.goalie != Some(robot_id)
}

fn prep_kick_direction<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  task: PrepTask,
  actor_id: u8,
) -> Vec2<f32> {
  if matches!(task.phase, PrepPhase::OffensiveKickoff)
    && let Some(receiver_id) = kickoff_receiver_id(cp, task, actor_id)
  {
    cp.state.mark_prep_receiver(receiver_id);
    return direction_to_robot_or_restart(cp, task, receiver_id);
  }

  kick_direction_for_restart(cp, task)
}

fn setup_position_for_direction(ball_pos: Vec2<f32>, direction: Vec2<f32>) -> Vec2<f32> {
  ball_pos - direction.normalized() * KICKER_SETUP_DISTANCE_MM
}

fn kick_direction_for_restart<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
) -> Vec2<f32> {
  let side = attacking_side(cp);
  let opponent_goal = Vec2::new(cp.field_setup.width as f32 * 0.5 * side, 0.0);
  let direction = opponent_goal - task.ball_pos;

  if direction.norm_squared() <= f32::EPSILON {
    Vec2::new(side, 0.0)
  } else {
    direction
  }
}

fn direction_to_robot_or_restart<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
  receiver_id: u8,
) -> Vec2<f32> {
  cp.state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == receiver_id)
    .and_then(|robot| robot.pos)
    .map(|pos| direction_or_attack(cp, pos - task.ball_pos))
    .unwrap_or_else(|| kick_direction_for_restart(cp, task))
}

fn command_kick_to_robot<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  actor_id: u8,
  receiver_id: u8,
  ball_pos: Vec2<f32>,
  kick_speed: u32,
) -> bool {
  let Some(receiver_pos) = cp
    .state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == receiver_id)
    .and_then(|robot| robot.pos)
  else {
    return false;
  };

  let dir = direction_or_attack(cp, receiver_pos - ball_pos);
  let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) else {
    return false;
  };

  robot.msg.cmd.state = StateFree as i32;
  robot.msg.cmd.task = TaskKick as i32;
  robot.msg.cmd.kick_orient = Some(dir.angle_in_u16() as u32);
  robot.msg.cmd.kick_speed = Some(kick_speed);
  true
}

fn free_kick_receiver_target<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  task: PrepTask,
) -> Vec2<f32> {
  let side = attacking_side(cp);
  let half_width = cp.field_setup.width as f32 * 0.5;
  let half_height = cp.field_setup.height as f32 * 0.5;
  let y_limit = (half_height - cp.field_setup.run_off_width as f32).max(0.0);
  let x_attack = (task.ball_pos.x * side + FREE_KICK_RECEIVER_ADVANCE_MM).clamp(
    FREE_KICK_RECEIVER_MIN_OPPONENT_HALF_DEPTH_MM,
    (half_width - FREE_KICK_RECEIVER_GOAL_STANDOFF_MM)
      .max(FREE_KICK_RECEIVER_MIN_OPPONENT_HALF_DEPTH_MM),
  );

  let lane = (half_height * 0.28).min(y_limit);
  let candidates = [
    Vec2::new(side * x_attack, 0.0),
    Vec2::new(side * x_attack, lane),
    Vec2::new(side * x_attack, -lane),
    Vec2::new(side * x_attack, (-task.ball_pos.y).clamp(-y_limit, y_limit)),
  ];

  candidates
    .into_iter()
    .max_by(|a, b| {
      free_kick_target_score(cp, task.ball_pos, *a).total_cmp(&free_kick_target_score(
        cp,
        task.ball_pos,
        *b,
      ))
    })
    .unwrap_or(Vec2::new(side * x_attack, 0.0))
}

fn free_kick_target_score<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  ball_pos: Vec2<f32>,
  target: Vec2<f32>,
) -> f32 {
  let opponent_goal = Vec2::new(cp.field_setup.width as f32 * 0.5 * attacking_side(cp), 0.0);
  let point_clearance = nearest_robot_distance_to_point(&cp.state.robots_opp, target).min(1800.0);
  let pass_clearance =
    nearest_robot_distance_to_segment(&cp.state.robots_opp, ball_pos, target).min(1400.0);
  let shot_clearance =
    nearest_robot_distance_to_segment(&cp.state.robots_opp, target, opponent_goal).min(1400.0);

  point_clearance + pass_clearance * 0.65 + shot_clearance * 0.65
}

fn command_free_kick_receiver_setup<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  receiver_id: u8,
  receiver_target: Vec2<f32>,
) {
  let opponent_goal = Vec2::new(cp.field_setup.width as f32 * 0.5 * attacking_side(cp), 0.0);
  let face_goal = direction_or_attack(cp, opponent_goal - receiver_target);

  command_robot_pos(
    cp,
    receiver_id,
    receiver_target,
    face_goal.angle_in_u16() as u32,
    Some(FREE_KICK_RECEIVER_SPEED_MM_S),
    StateFree as i32,
  );
}

fn command_free_kick_pass<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  actor_id: u8,
  receiver_id: u8,
  receiver_target: Vec2<f32>,
  ball_pos: Vec2<f32>,
) -> bool {
  let (receiver_pos, receiver_vel) = cp
    .state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == receiver_id)
    .map(|robot| {
      (
        robot.pos.unwrap_or(receiver_target),
        robot.vel.unwrap_or_default(),
      )
    })
    .unwrap_or((receiver_target, Vec2::default()));

  let base_dir = direction_or_attack(cp, receiver_pos - ball_pos);
  let base_dist = base_dir.length().max(1.0);
  let lead_s = (base_dist / FREE_KICK_PASS_LEAD_SPEED_MM_S).clamp(0.05, 0.22);
  let target = receiver_pos + receiver_vel * lead_s;
  let dir = direction_or_attack(cp, target - ball_pos);

  let Some(robot) = cp.robots.get_mut(&(actor_id as u32)) else {
    return false;
  };

  robot.msg.cmd.state = StateFree as i32;
  robot.msg.cmd.task = TaskKick as i32;
  robot.msg.cmd.kick_orient = Some(dir.angle_in_u16() as u32);
  robot.msg.cmd.kick_speed = Some(50);
  true
}

fn command_receiver_receive<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  receiver_id: u8,
  pass_direction: Vec2<f32>,
) {
  let Some(robot) = cp.robots.get_mut(&(receiver_id as u32)) else {
    return;
  };

  robot.msg.cmd.state = StateFree as i32;
  robot.msg.cmd.task = TaskRecKick as i32;
  robot.msg.cmd.kick_orient = Some(pass_direction.angle_in_u16() as u32);
}

fn command_receiver_shot<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  receiver_id: u8,
  all_robots: &[Robot],
) -> bool {
  let Some(robot_state) = cp
    .state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == receiver_id)
    .cloned()
  else {
    return false;
  };

  let Some(robot_msg) = cp.robots.get_mut(&(receiver_id as u32)) else {
    return false;
  };

  robot_msg.msg.cmd.state = StateFree as i32;
  shoot_to_goal(
    robot_msg,
    &robot_state,
    all_robots,
    &cp.state,
    &cp.field_setup,
  );
  true
}

fn command_robot_pos<C: Communication, A: Ai + Send>(
  cp: &mut CrashPilot<C, A>,
  robot_id: u8,
  target: Vec2<f32>,
  orientation: u32,
  speed: Option<u32>,
  state: i32,
) {
  if let Some(robot) = cp.robots.get_mut(&(robot_id as u32)) {
    robot.msg.cmd.state = state;
    robot.msg.cmd.task = TaskPos as i32;
    robot.msg.cmd.pos = Some(target.to_cp_vec2());
    robot.msg.cmd.speed = speed;
    robot.msg.cmd.orientation = Some(orientation);
  }
}

fn receiver_ready_to_shoot<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  receiver_id: u8,
) -> bool {
  if cp
    .robots
    .get(&(receiver_id as u32))
    .map(|robot| robot.feedback.has_ball)
    .unwrap_or_default()
  {
    return true;
  }

  let ball_speed = cp.state.ball.ball.vel.length();
  cp.state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == receiver_id)
    .and_then(|robot| robot.distance_ball)
    .map(|distance| {
      distance <= FREE_KICK_RECEIVER_CAPTURE_DISTANCE_MM
        && ball_speed <= FREE_KICK_RECEIVER_CAPTURE_SPEED_MM_S
    })
    .unwrap_or_default()
}

fn robot_is_near<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  robot_id: u8,
  target: Vec2<f32>,
  tolerance: f32,
) -> bool {
  cp.state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == robot_id)
    .and_then(|robot| robot.pos)
    .map(|pos| (pos - target).norm_squared() <= tolerance * tolerance)
    .unwrap_or_default()
}

fn distance_to_target(robot: &Robot, target: Vec2<f32>) -> f32 {
  robot
    .pos
    .map(|pos| (pos - target).length())
    .unwrap_or(f32::MAX)
}

fn nearest_robot_distance_to_point(robots: &[Robot], point: Vec2<f32>) -> f32 {
  robots
    .iter()
    .filter_map(|robot| robot.pos)
    .map(|pos| (pos - point).length())
    .min_by(|a, b| a.total_cmp(b))
    .unwrap_or(1800.0)
}

fn nearest_robot_distance_to_segment(robots: &[Robot], start: Vec2<f32>, end: Vec2<f32>) -> f32 {
  robots
    .iter()
    .filter_map(|robot| robot.pos)
    .map(|pos| distance_to_segment(pos, start, end))
    .min_by(|a, b| a.total_cmp(b))
    .unwrap_or(1400.0)
}

fn distance_to_segment(point: Vec2<f32>, start: Vec2<f32>, end: Vec2<f32>) -> f32 {
  let segment = end - start;
  let len_sq = segment.norm_squared();
  if len_sq <= f32::EPSILON {
    return (point - start).length();
  }

  let t = ((point - start).dot(&segment) / len_sq).clamp(0.0, 1.0);
  (point - (start + segment * t)).length()
}

fn robot_is_in_own_half<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  robot_id: u8,
) -> bool {
  cp.state
    .robots_self
    .iter()
    .find(|robot| robot.robot_id == robot_id)
    .map(|robot| robot_pos_is_in_own_half(cp, robot.pos))
    .unwrap_or_default()
}

fn robot_pos_is_in_own_half<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  pos: Option<Vec2<f32>>,
) -> bool {
  pos
    .map(|pos| pos.x * attacking_side(cp) <= 0.0)
    .unwrap_or_default()
}

fn direction_or_attack<C: Communication, A: Ai + Send>(
  cp: &CrashPilot<C, A>,
  direction: Vec2<f32>,
) -> Vec2<f32> {
  if direction.norm_squared() <= f32::EPSILON {
    Vec2::new(attacking_side(cp), 0.0)
  } else {
    direction
  }
}

fn attacking_side<C: Communication, A: Ai + Send>(cp: &CrashPilot<C, A>) -> f32 {
  if cp.state.site.abs() > f32::EPSILON {
    -cp.state.site.signum()
  } else {
    1.0
  }
}

fn set_robot_state_for_all<C: Communication, A: Ai + Send>(cp: &mut CrashPilot<C, A>, state: i32) {
  for robot in cp.robots.values_mut() {
    robot.msg.cmd.state = state;
  }
}

fn set_goalie<C: Communication, A: Ai + Send>(cp: &mut CrashPilot<C, A>) {
  if let Some(goalie) = cp.state.goalie
    && let Some(robot) = cp.robots.get_mut(&(goalie as u32))
  {
    robot.msg.cmd.state = StateGoalie as i32;
  }
}
