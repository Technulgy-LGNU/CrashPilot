use crate::{Commands, GameState};
use simhark::WorldState;

const OWN_ATTACK_SIGN: f64 = 1.0;
const TERMINAL_GOAL_REWARD: f64 = 14.0;
const MAX_REWARD: f64 = 18.0;
const MIN_REWARD: f64 = -18.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RewardMode {
  TouchBall,
  DribbleToGoal,
  ShootGoal,
  PassReceive,
  Full,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PossessionTeam {
  Own,
  Opp,
  Neutral,
}

#[derive(Debug, Clone, Copy)]
struct Point {
  x: f64,
  y: f64,
}

#[derive(Debug, Clone, Copy)]
struct FieldScale {
  half_x: f64,
  half_y: f64,
  goal_half_width: f64,
}

#[derive(Debug, Clone, Copy)]
struct Possession {
  team: PossessionTeam,
  own_dist: f64,
  opp_dist: f64,
  own_idx: Option<usize>,
}

pub fn compute_reward_for(
  mode: RewardMode,
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
  commands: Commands,
) -> f64 {
  let reward = match mode {
    RewardMode::TouchBall => touch_ball_reward(old_sim, new_sim, old, new),
    RewardMode::DribbleToGoal => dribble_to_goal_reward(old_sim, new_sim, old, new),
    RewardMode::ShootGoal => shoot_goal_reward(old_sim, new_sim, old, new, commands),
    RewardMode::PassReceive => pass_receive_stage_reward(old_sim, new_sim, old, new, commands),
    RewardMode::Full => compute_reward(old_sim, new_sim, old, new, commands),
  };

  finite(reward).clamp(MIN_REWARD, MAX_REWARD)
}

pub fn compute_reward(
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
  commands: Commands,
) -> f64 {
  let scale = estimate_field_scale(&old, &new);
  let diag = (scale.half_x.hypot(scale.half_y) * 2.0).max(1e-6);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let old_poss = estimate_possession(&old, scale);
  let new_poss = estimate_possession(&new, scale);

  let mut reward = 0.0;

  reward += terminal_goal_reward(old_sim, new_sim);
  reward += ball_reward(
    old_ball,
    new_ball,
    old.ball.vel.x as f64,
    new.ball.vel.x as f64,
    scale,
    diag,
  );
  reward += possession_reward(old_poss, new_poss, scale);
  reward += command_reward(&old, &new, commands, old_poss, new_poss, scale, diag);
  reward += team_shape_reward(&old, &new, commands, old_poss, new_poss, scale, diag);
  reward += sim_contact_reward(old_sim, new_sim);

  finite(reward).clamp(MIN_REWARD, MAX_REWARD)
}

fn touch_ball_reward(
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
) -> f64 {
  let scale = estimate_field_scale(&old, &new);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let (_, old_own_dist) = nearest_robot(old_ball, &old.own_robots);
  let (_, new_own_dist) = nearest_robot(new_ball, &new.own_robots);
  let (_, new_opp_dist) = nearest_robot(new_ball, &new.opp_robots);
  let own_contacts = new_sim.blue_robots.iter().filter(|r| r.infrared).count() as f64;
  let old_contacts = old_sim.blue_robots.iter().filter(|r| r.infrared).count() as f64;

  let approach = normalized_delta(old_own_dist, new_own_dist, scale.half_x);
  let possession = if new_own_dist < possession_radius(scale) && new_own_dist <= new_opp_dist {
    1.0
  } else {
    0.0
  };

  1.2 * approach + 1.6 * (own_contacts - old_contacts).max(0.0) + 0.35 * possession
}

fn dribble_to_goal_reward(
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
) -> f64 {
  let scale = estimate_field_scale(&old, &new);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let new_poss = estimate_possession(&new, scale);
  let progress = ((new_ball.x - old_ball.x) * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let contact = sim_contact_reward(old_sim, new_sim);
  let control = if new_poss.team == PossessionTeam::Own {
    0.28
  } else {
    -0.12
  };

  terminal_goal_reward(old_sim, new_sim) + 2.0 * progress + control + contact
}

fn shoot_goal_reward(
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
  commands: Commands,
) -> f64 {
  let scale = estimate_field_scale(&old, &new);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let ball_delta = point(new_ball.x - old_ball.x, new_ball.y - old_ball.y);
  let shot_speed = (new.ball.vel.x as f64 * OWN_ATTACK_SIGN / 6.0).clamp(-1.0, 1.0);
  let shot_lane = shot_lane_quality(new_ball, scale) - shot_lane_quality(old_ball, scale);
  let kick_bonus = commands
    .iter()
    .flatten()
    .filter(|cmd| {
      matches!(
        cmd,
        crate::RobotCommand::Kick(_)
          | crate::RobotCommand::Chip(_)
          | crate::RobotCommand::RecKick(_)
          | crate::RobotCommand::KickGoal
      )
    })
    .count() as f64
    * 0.08;
  let progress = (ball_delta.x * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);

  terminal_goal_reward(old_sim, new_sim)
    + 1.0 * progress
    + 0.55 * shot_speed
    + 0.4 * shot_lane
    + kick_bonus
}

fn pass_receive_stage_reward(
  old_sim: &WorldState,
  new_sim: &WorldState,
  old: GameState,
  new: GameState,
  commands: Commands,
) -> f64 {
  let scale = estimate_field_scale(&old, &new);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let ball_delta = point(new_ball.x - old_ball.x, new_ball.y - old_ball.y);
  let mut reward = terminal_goal_reward(old_sim, new_sim);

  for (idx, cmd) in commands.iter().enumerate() {
    match cmd {
      Some(crate::RobotCommand::PassTo(dst)) => {
        reward += pass_reward(idx, *dst as usize, &old, &new, true, ball_delta, scale);
      }
      Some(crate::RobotCommand::RecPass) => {
        reward += receive_reward(idx, &old, &new, commands, ball_delta, scale);
      }
      _ => {}
    }
  }

  reward + 0.3 * sim_contact_reward(old_sim, new_sim)
}

fn terminal_goal_reward(old_sim: &WorldState, new_sim: &WorldState) -> f64 {
  let mut reward = 0.0;

  if new_sim.goal_blue && !old_sim.goal_blue {
    reward += TERMINAL_GOAL_REWARD;
  }

  if new_sim.goal_yellow && !old_sim.goal_yellow {
    reward -= TERMINAL_GOAL_REWARD;
  }

  reward
}

fn ball_reward(
  old_ball: Point,
  new_ball: Point,
  old_ball_vx: f64,
  new_ball_vx: f64,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let opp_goal = point(OWN_ATTACK_SIGN * scale.half_x, 0.0);
  let old_goal_dist = distance(old_ball, opp_goal);
  let new_goal_dist = distance(new_ball, opp_goal);
  let old_danger = defensive_danger(old_ball, scale);
  let new_danger = defensive_danger(new_ball, scale);

  let x_progress = ((new_ball.x - old_ball.x) * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let goal_approach = ((old_goal_dist - new_goal_dist) / diag).clamp(-1.0, 1.0);
  let danger_reduction = (old_danger - new_danger).clamp(-1.0, 1.0);
  let velocity = (new_ball_vx * OWN_ATTACK_SIGN / scale.half_x.max(1e-6)).tanh();
  let acceleration =
    ((new_ball_vx - old_ball_vx) * OWN_ATTACK_SIGN / scale.half_x.max(1e-6)).tanh();

  let mut reward = 1.15 * x_progress;
  reward += 0.75 * goal_approach;
  reward += 1.10 * danger_reduction;
  reward += 0.06 * velocity;
  reward += 0.04 * acceleration;

  if ball_outside_field(new_ball, scale) && !inside_goal_mouth(new_ball, scale) {
    reward -= 0.18;
  }

  if new_ball.x * OWN_ATTACK_SIGN > scale.half_x * 0.86 && inside_goal_mouth(new_ball, scale) {
    reward += 0.12;
  }

  reward
}

fn possession_reward(old: Possession, new: Possession, scale: FieldScale) -> f64 {
  let mut reward = 0.0;

  match new.team {
    PossessionTeam::Own => reward += 0.08,
    PossessionTeam::Opp => reward -= 0.10,
    PossessionTeam::Neutral => {}
  }

  match (old.team, new.team) {
    (PossessionTeam::Opp, PossessionTeam::Own) => reward += 1.05,
    (PossessionTeam::Neutral, PossessionTeam::Own) => reward += 0.65,
    (PossessionTeam::Own, PossessionTeam::Opp) => reward -= 1.00,
    (PossessionTeam::Own, PossessionTeam::Neutral) => reward -= 0.25,
    (PossessionTeam::Opp, PossessionTeam::Neutral) => reward += 0.22,
    (PossessionTeam::Neutral, PossessionTeam::Opp) => reward -= 0.35,
    _ => {}
  }

  let own_pressure_delta = normalized_delta(old.own_dist, new.own_dist, scale.half_x);
  let opp_space_delta = normalized_delta(new.opp_dist, old.opp_dist, scale.half_x);

  if new.team != PossessionTeam::Own {
    reward += 0.22 * own_pressure_delta;
  }

  reward += 0.10 * opp_space_delta;

  if new.own_dist + possession_radius(scale) * 0.4 < new.opp_dist {
    reward += 0.05;
  } else if new.opp_dist + possession_radius(scale) * 0.4 < new.own_dist {
    reward -= 0.06;
  }

  reward
}

fn command_reward(
  old: &GameState,
  new: &GameState,
  commands: Commands,
  old_poss: Possession,
  new_poss: Possession,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let mut reward = 0.0;
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let ball_delta = point(new_ball.x - old_ball.x, new_ball.y - old_ball.y);
  let mut active = 0usize;
  let mut holds = 0usize;
  let mut kick_like = 0usize;
  let mut defensive_cmds = 0usize;
  let defensive_need = defensive_danger(old_ball, scale);

  for (idx, robot) in old.own_robots.iter().enumerate() {
    if robot.is_some() {
      active += 1;
    }

    match (robot, commands[idx]) {
      (Some(robot), Some(cmd)) => {
        let robot_pos = point(robot.pos.x, robot.pos.y);
        let robot_new_pos = new.own_robots[idx]
          .map(|r| point(r.pos.x, r.pos.y))
          .unwrap_or(robot_pos);
        let dist_to_ball = distance(robot_pos, old_ball);
        let has_ball = robot_has_ball(dist_to_ball, old_poss.opp_dist, scale);

        reward += match cmd {
          crate::RobotCommand::Pos(target) => target_position_reward(
            robot_pos,
            robot_new_pos,
            point(target.x, target.y),
            scale,
            diag,
          ),
          crate::RobotCommand::Kick(power)
          | crate::RobotCommand::Chip(power)
          | crate::RobotCommand::RecKick(power)
          | crate::RobotCommand::Kickoff(power)
          | crate::RobotCommand::FreeKick(power) => {
            kick_like += 1;
            direct_ball_action_reward(has_ball, power as f64, ball_delta, scale)
          }
          crate::RobotCommand::Steal => {
            steal_reward(robot.is_goalie, dist_to_ball, old_poss, new_poss, scale)
          }
          crate::RobotCommand::Dribble(target) | crate::RobotCommand::PosBall(target) => {
            controlled_move_reward(
              has_ball,
              old_ball,
              new_ball,
              robot_pos,
              robot_new_pos,
              point(target.x, target.y),
              scale,
              diag,
            ) - dribble_control_penalty(has_ball)
          }
          crate::RobotCommand::KickGoal => {
            kick_like += 1;
            direct_shot_reward(has_ball, old_ball, new_ball, ball_delta, scale)
          }
          crate::RobotCommand::PassTo(dst) => {
            kick_like += 1;
            pass_reward(idx, dst as usize, old, new, has_ball, ball_delta, scale)
          }
          crate::RobotCommand::RecPass => {
            receive_reward(idx, old, new, commands, ball_delta, scale)
          }
          crate::RobotCommand::GoalWall | crate::RobotCommand::GoalieGuard => {
            defensive_cmds += 1;
            defensive_command_reward(robot_pos, old_ball, defensive_need, scale)
          }
          crate::RobotCommand::Hold => {
            holds += 1;
            hold_reward(robot.is_goalie, defensive_need)
          }
        };
      }
      (Some(_), None) => reward -= 0.03,
      (None, Some(_)) => reward -= 0.05,
      (None, None) => {}
    }
  }

  if kick_like > 1 {
    reward -= 0.04 * (kick_like - 1) as f64;
  }

  if active > 0 {
    let hold_fraction = holds as f64 / active as f64;
    if hold_fraction > 0.55 {
      reward -= 0.10 * (hold_fraction - 0.55);
    }
  }

  if defensive_need < 0.15 && defensive_cmds > 1 {
    reward -= 0.04 * (defensive_cmds - 1) as f64;
  }

  reward
}

fn team_shape_reward(
  old: &GameState,
  new: &GameState,
  commands: Commands,
  old_poss: Possession,
  new_poss: Possession,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let mut reward = 0.0;
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let danger = defensive_danger(new_ball, scale);

  reward -= crowding_penalty(&new.own_robots, commands, scale);
  reward += field_spread_reward(&old.own_robots, &new.own_robots, commands, scale, diag);
  reward += movement_diversity_reward(&old.own_robots, &new.own_robots, commands, scale);
  reward += defensive_shape_reward(&new.own_robots, new_ball, danger, scale);
  reward += defensive_wall_reward(&new.own_robots, commands, new_ball, danger, scale);
  reward += goalie_reward(
    &old.own_robots,
    &new.own_robots,
    new_ball,
    danger,
    scale,
    diag,
  );

  if new_poss.team == PossessionTeam::Own {
    reward += attacking_support_reward(&new.own_robots, new_poss.own_idx, new_ball, scale);
  }

  if old_poss.team == PossessionTeam::Own && new_poss.team == PossessionTeam::Own {
    let carried_progress =
      ((new_ball.x - old_ball.x) * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
    reward += 0.10 * carried_progress.max(0.0);
  }

  reward
}

fn field_spread_reward(
  old_robots: &[Option<crate::RobotState>; 16],
  new_robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let old_score = field_spread_score(old_robots, commands, scale, diag);
  let new_score = field_spread_score(new_robots, commands, scale, diag);
  let progress = (new_score - old_score).clamp(-1.0, 1.0);

  0.08 * progress + 0.04 * (new_score - 0.35)
}

fn field_spread_score(
  robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let points: Vec<_> = robots
    .iter()
    .enumerate()
    .filter_map(|(idx, robot)| robot.map(|robot| (idx, robot)))
    .filter(|(idx, robot)| !defensive_formation_exempt(*idx, *robot, commands, scale))
    .map(|(_, robot)| point(robot.pos.x, robot.pos.y))
    .collect();

  if points.len() < 2 {
    return 0.0;
  }

  let mut pair_dist_sum = 0.0;
  let mut pair_count = 0usize;
  let mut min_x = f64::INFINITY;
  let mut max_x = f64::NEG_INFINITY;
  let mut min_y = f64::INFINITY;
  let mut max_y = f64::NEG_INFINITY;

  for (idx, a) in points.iter().enumerate() {
    min_x = min_x.min(a.x);
    max_x = max_x.max(a.x);
    min_y = min_y.min(a.y);
    max_y = max_y.max(a.y);

    for b in points.iter().skip(idx + 1) {
      pair_dist_sum += distance(*a, *b);
      pair_count += 1;
    }
  }

  let average_pair_dist = if pair_count > 0 {
    pair_dist_sum / pair_count as f64
  } else {
    0.0
  };
  let pair_coverage = (average_pair_dist / diag).clamp(0.0, 1.0);
  let x_coverage = ((max_x - min_x) / (scale.half_x * 2.0).max(1e-6)).clamp(0.0, 1.0);
  let y_coverage = ((max_y - min_y) / (scale.half_y * 2.0).max(1e-6)).clamp(0.0, 1.0);

  0.55 * pair_coverage + 0.25 * x_coverage + 0.20 * y_coverage
}

fn movement_diversity_reward(
  old_robots: &[Option<crate::RobotState>; 16],
  new_robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  scale: FieldScale,
) -> f64 {
  let min_move = (scale.half_x * 0.002).max(1e-4);
  let movements: Vec<_> = old_robots
    .iter()
    .zip(new_robots.iter())
    .enumerate()
    .filter_map(|(idx, (old, new))| {
      let (Some(old), Some(new)) = (old, new) else {
        return None;
      };

      if defensive_formation_exempt(idx, *new, commands, scale) {
        return None;
      }

      let delta = point(new.pos.x - old.pos.x, new.pos.y - old.pos.y);
      if delta.x.hypot(delta.y) >= min_move {
        Some(delta)
      } else {
        None
      }
    })
    .collect();

  if movements.len() < 3 {
    return 0.0;
  }

  let mut similarity_sum = 0.0;
  let mut pair_count = 0usize;

  for (idx, a) in movements.iter().enumerate() {
    for b in movements.iter().skip(idx + 1) {
      similarity_sum += vector_alignment(*a, *b).max(0.0);
      pair_count += 1;
    }
  }

  if pair_count == 0 {
    return 0.0;
  }

  let average_similarity = similarity_sum / pair_count as f64;
  let diversity = 1.0 - average_similarity;
  let moving_fraction =
    movements.len() as f64 / active_spread_robot_count(new_robots, commands, scale).max(1) as f64;

  0.05 * diversity * moving_fraction - 0.10 * (average_similarity - 0.70).max(0.0)
}

fn defensive_formation_exempt(
  idx: usize,
  robot: crate::RobotState,
  commands: Commands,
  scale: FieldScale,
) -> bool {
  if robot.is_goalie {
    return true;
  }

  if matches!(
    commands[idx],
    Some(crate::RobotCommand::GoalWall | crate::RobotCommand::GoalieGuard)
  ) {
    return true;
  }

  in_own_defense_area(point(robot.pos.x, robot.pos.y), scale)
}

fn in_own_defense_area(pos: Point, scale: FieldScale) -> bool {
  pos.x * OWN_ATTACK_SIGN < -scale.half_x * 0.72 && pos.y.abs() < scale.half_y * 0.55
}

fn active_spread_robot_count(
  robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  scale: FieldScale,
) -> usize {
  robots
    .iter()
    .enumerate()
    .filter_map(|(idx, robot)| robot.map(|robot| (idx, robot)))
    .filter(|(idx, robot)| !defensive_formation_exempt(*idx, *robot, commands, scale))
    .count()
}

fn sim_contact_reward(old_sim: &WorldState, new_sim: &WorldState) -> f64 {
  let old_blue_contacts = old_sim.blue_robots.iter().filter(|r| r.infrared).count();
  let new_blue_contacts = new_sim.blue_robots.iter().filter(|r| r.infrared).count();
  let old_yellow_contacts = old_sim.yellow_robots.iter().filter(|r| r.infrared).count();
  let new_yellow_contacts = new_sim.yellow_robots.iter().filter(|r| r.infrared).count();

  let mut reward = 0.0;

  if new_blue_contacts > old_blue_contacts {
    reward += 0.18;
  }

  if new_yellow_contacts > old_yellow_contacts {
    reward -= 0.18;
  }

  let blue_dribblers = new_sim.blue_robots.iter().filter(|r| r.dribbler_on).count();
  let yellow_dribblers = new_sim
    .yellow_robots
    .iter()
    .filter(|r| r.dribbler_on)
    .count();

  reward += 0.01 * blue_dribblers as f64;
  reward -= 0.01 * yellow_dribblers as f64;

  reward
}

fn target_position_reward(
  old_pos: Point,
  new_pos: Point,
  target: Point,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  if !target_is_reasonable(target, scale) {
    return -0.04;
  }

  let progress = ((distance(old_pos, target) - distance(new_pos, target)) / diag).clamp(-1.0, 1.0);
  let field_value = (target.x * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let centrality = 1.0 - (target.y.abs() / scale.half_y).clamp(0.0, 1.0);

  0.12 * progress + 0.015 * field_value + 0.01 * centrality
}

fn direct_ball_action_reward(
  has_ball: bool,
  power: f64,
  ball_delta: Point,
  scale: FieldScale,
) -> f64 {
  if !has_ball {
    return -0.12;
  }

  let progress = (ball_delta.x * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let controlled_power = (power - 0.55).abs();

  0.08 + 0.28 * progress.max(0.0) - 0.025 * controlled_power
}

fn steal_reward(
  is_goalie: bool,
  dist_to_ball: f64,
  old_poss: Possession,
  new_poss: Possession,
  scale: FieldScale,
) -> f64 {
  let mut reward = 0.0;
  let radius = possession_radius(scale);

  if is_goalie {
    reward -= 0.04;
  }

  if old_poss.team == PossessionTeam::Own {
    reward -= 0.05;
  } else if dist_to_ball < radius * 5.0 {
    reward += 0.08 * (1.0 - dist_to_ball / (radius * 5.0));
  } else {
    reward -= 0.05;
  }

  if old_poss.team != PossessionTeam::Own && new_poss.team == PossessionTeam::Own {
    reward += 0.18;
  }

  reward
}

fn controlled_move_reward(
  has_ball: bool,
  old_ball: Point,
  new_ball: Point,
  old_pos: Point,
  new_pos: Point,
  target: Point,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  if !has_ball {
    return -0.10;
  }

  let ball_progress = ((new_ball.x - old_ball.x) * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let target_progress = if target_is_reasonable(target, scale) {
    ((distance(old_pos, target) - distance(new_pos, target)) / diag).clamp(-1.0, 1.0)
  } else {
    0.0
  };

  0.09 + 0.22 * ball_progress.max(0.0) + 0.08 * target_progress
}

fn dribble_control_penalty(has_ball: bool) -> f64 {
  if has_ball { 0.055 } else { 0.0 }
}

fn direct_shot_reward(
  has_ball: bool,
  old_ball: Point,
  new_ball: Point,
  ball_delta: Point,
  scale: FieldScale,
) -> f64 {
  if !has_ball {
    return -0.14;
  }

  let toward_goal = (ball_delta.x * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let new_shot_lane = shot_lane_quality(new_ball, scale);
  let old_shot_lane = shot_lane_quality(old_ball, scale);

  0.12 + 0.35 * toward_goal.max(0.0) + 0.12 * (new_shot_lane - old_shot_lane)
}

fn pass_reward(
  passer_idx: usize,
  dst_idx: usize,
  old: &GameState,
  new: &GameState,
  has_ball: bool,
  ball_delta: Point,
  scale: FieldScale,
) -> f64 {
  if !has_ball {
    return -0.13;
  }

  let Some(receiver) = old.own_robots.get(dst_idx).and_then(|r| *r) else {
    return -0.12;
  };

  if passer_idx == dst_idx {
    return -0.10;
  }

  let receiver_pos = point(receiver.pos.x, receiver.pos.y);
  let passer_pos = old.own_robots[passer_idx]
    .map(|r| point(r.pos.x, r.pos.y))
    .unwrap_or(receiver_pos);
  let to_receiver = point(receiver_pos.x - passer_pos.x, receiver_pos.y - passer_pos.y);
  let alignment = vector_alignment(ball_delta, to_receiver).max(0.0);
  let receiver_ahead = ((receiver_pos.x - passer_pos.x) * OWN_ATTACK_SIGN / scale.half_x)
    .clamp(-1.0, 1.0)
    .max(0.0);
  let receiver_open = receiver_open_score(receiver_pos, &old.opp_robots, scale);
  let receiver_gets_close = new.own_robots[dst_idx]
    .map(|r| {
      let old_dist = distance(receiver_pos, point(old.ball.pos.x, old.ball.pos.y));
      let new_dist = distance(
        point(r.pos.x, r.pos.y),
        point(new.ball.pos.x, new.ball.pos.y),
      );
      ((old_dist - new_dist) / scale.half_x)
        .clamp(-1.0, 1.0)
        .max(0.0)
    })
    .unwrap_or(0.0);

  0.16
    + 0.14 * alignment
    + 0.10 * receiver_ahead
    + 0.12 * receiver_open
    + 0.10 * receiver_gets_close
}

fn receive_reward(
  idx: usize,
  old: &GameState,
  new: &GameState,
  commands: Commands,
  ball_delta: Point,
  scale: FieldScale,
) -> f64 {
  let Some(receiver) = old.own_robots[idx] else {
    return -0.05;
  };

  let receiver_pos = point(receiver.pos.x, receiver.pos.y);
  let old_ball = point(old.ball.pos.x, old.ball.pos.y);
  let new_ball = point(new.ball.pos.x, new.ball.pos.y);
  let pass_targeted = commands.iter().any(|cmd| match cmd {
    Some(crate::RobotCommand::PassTo(dst)) => *dst as usize == idx,
    _ => false,
  });
  let to_receiver = point(receiver_pos.x - old_ball.x, receiver_pos.y - old_ball.y);
  let alignment = vector_alignment(ball_delta, to_receiver).max(0.0);
  let receiver_new_pos = new.own_robots[idx]
    .map(|r| point(r.pos.x, r.pos.y))
    .unwrap_or(receiver_pos);
  let close_delta = ((distance(receiver_pos, old_ball) - distance(receiver_new_pos, new_ball))
    / scale.half_x)
    .clamp(-1.0, 1.0);

  let mut reward = 0.04 + 0.12 * alignment + 0.10 * close_delta.max(0.0);

  if pass_targeted {
    reward += 0.10;
  } else if alignment < 0.15 {
    reward -= 0.03;
  }

  reward
}

fn defensive_command_reward(
  robot_pos: Point,
  ball: Point,
  defensive_need: f64,
  scale: FieldScale,
) -> f64 {
  if defensive_need < 0.12 {
    return -0.05;
  }

  let block = block_line_score(robot_pos, ball, scale);
  0.04 + 0.14 * defensive_need * block
}

fn hold_reward(is_goalie: bool, defensive_need: f64) -> f64 {
  if is_goalie && defensive_need < 0.20 {
    0.01
  } else if defensive_need > 0.35 {
    -0.04
  } else {
    -0.015
  }
}

fn crowding_penalty(
  robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  scale: FieldScale,
) -> f64 {
  let min_sep = scale.half_x * 0.035;
  let mut penalty = 0.0;

  for i in 0..robots.len() {
    let Some(a) = robots[i] else {
      continue;
    };

    for (j, b) in robots.iter().enumerate().skip(i + 1) {
      let Some(b) = b else {
        continue;
      };

      if defensive_formation_exempt(i, a, commands, scale)
        && defensive_formation_exempt(j, *b, commands, scale)
      {
        continue;
      }

      let dist = distance(point(a.pos.x, a.pos.y), point(b.pos.x, b.pos.y));
      if dist < min_sep {
        penalty += 0.025 * (1.0 - dist / min_sep);
      }
    }
  }

  penalty.min(0.25)
}

fn defensive_shape_reward(
  robots: &[Option<crate::RobotState>; 16],
  ball: Point,
  danger: f64,
  scale: FieldScale,
) -> f64 {
  if danger < 0.10 {
    return 0.0;
  }

  let best_block = robots
    .iter()
    .flatten()
    .filter(|r| !r.is_goalie)
    .map(|r| block_line_score(point(r.pos.x, r.pos.y), ball, scale))
    .fold(0.0, f64::max);

  0.13 * danger * best_block
}

fn defensive_wall_reward(
  robots: &[Option<crate::RobotState>; 16],
  commands: Commands,
  ball: Point,
  danger: f64,
  scale: FieldScale,
) -> f64 {
  if danger < 0.12 {
    return 0.0;
  }

  let wall: Vec<_> = robots
    .iter()
    .enumerate()
    .filter_map(|(idx, robot)| robot.map(|robot| (idx, robot)))
    .filter(|(idx, robot)| {
      !robot.is_goalie
        && matches!(commands[*idx], Some(crate::RobotCommand::GoalWall))
        && in_own_defense_area(point(robot.pos.x, robot.pos.y), scale)
    })
    .map(|(_, robot)| point(robot.pos.x, robot.pos.y))
    .collect();

  if wall.is_empty() {
    return 0.0;
  }

  let block_sum: f64 = wall
    .iter()
    .map(|pos| block_line_score(*pos, ball, scale))
    .sum();
  let block_score = (block_sum / 2.0).clamp(0.0, 1.0);
  let y_span = wall
    .iter()
    .map(|pos| pos.y)
    .fold((f64::INFINITY, f64::NEG_INFINITY), |(min_y, max_y), y| {
      (min_y.min(y), max_y.max(y))
    });
  let lateral_coverage = if wall.len() >= 2 {
    ((y_span.1 - y_span.0) / (scale.goal_half_width * 3.0).max(1e-6)).clamp(0.0, 1.0)
  } else {
    0.0
  };
  let count_score = (wall.len() as f64 / 3.0).clamp(0.0, 1.0);

  danger * (0.04 * count_score + 0.08 * block_score + 0.05 * lateral_coverage)
}

fn goalie_reward(
  old_robots: &[Option<crate::RobotState>; 16],
  new_robots: &[Option<crate::RobotState>; 16],
  ball: Point,
  danger: f64,
  scale: FieldScale,
  diag: f64,
) -> f64 {
  let urgency = danger.max(0.20);
  let old_goalie = old_robots.iter().flatten().find(|r| r.is_goalie);
  let new_goalie = new_robots.iter().flatten().find(|r| r.is_goalie);
  let Some(new_goalie) = new_goalie else {
    return -0.04 * urgency;
  };

  let target = point(
    -OWN_ATTACK_SIGN * scale.half_x * 0.92,
    ball.y.clamp(-scale.goal_half_width, scale.goal_half_width),
  );
  let new_dist = distance(point(new_goalie.pos.x, new_goalie.pos.y), target);
  let closeness = (1.0 - new_dist / (scale.half_x * 0.35)).clamp(0.0, 1.0);
  let progress = old_goalie
    .map(|old_goalie| {
      let old_dist = distance(point(old_goalie.pos.x, old_goalie.pos.y), target);
      ((old_dist - new_dist) / diag).clamp(-1.0, 1.0)
    })
    .unwrap_or(0.0);

  0.025 * closeness + urgency * (0.05 * closeness + 0.08 * progress)
}

fn attacking_support_reward(
  robots: &[Option<crate::RobotState>; 16],
  holder_idx: Option<usize>,
  ball: Point,
  scale: FieldScale,
) -> f64 {
  let mut supporters = 0usize;

  for (idx, robot) in robots.iter().enumerate() {
    if Some(idx) == holder_idx {
      continue;
    }

    let Some(robot) = robot else {
      continue;
    };

    if robot.is_goalie {
      continue;
    }

    let pos = point(robot.pos.x, robot.pos.y);
    let ahead = (pos.x - ball.x) * OWN_ATTACK_SIGN > scale.half_x * 0.10;
    let separated = (pos.y - ball.y).abs() > scale.half_y * 0.12;

    if ahead && separated {
      supporters += 1;
    }
  }

  if supporters == 0 {
    -0.04
  } else {
    (supporters as f64 * 0.035).min(0.12)
  }
}

fn estimate_possession(state: &GameState, scale: FieldScale) -> Possession {
  let ball = point(state.ball.pos.x, state.ball.pos.y);
  let (own_idx, own_dist) = nearest_robot(ball, &state.own_robots);
  let (_, opp_dist) = nearest_robot(ball, &state.opp_robots);
  let radius = possession_radius(scale);

  let team = if own_dist < radius && own_dist <= opp_dist * 1.10 {
    PossessionTeam::Own
  } else if opp_dist < radius && opp_dist < own_dist * 1.10 {
    PossessionTeam::Opp
  } else {
    PossessionTeam::Neutral
  };

  Possession {
    team,
    own_dist,
    opp_dist,
    own_idx,
  }
}

fn estimate_field_scale(old: &GameState, new: &GameState) -> FieldScale {
  let mut max_x = old.ball.pos.x.abs().max(new.ball.pos.x.abs()) as f64;
  let mut max_y = old.ball.pos.y.abs().max(new.ball.pos.y.abs()) as f64;

  for robot in old
    .own_robots
    .iter()
    .chain(old.opp_robots.iter())
    .chain(new.own_robots.iter())
    .chain(new.opp_robots.iter())
    .flatten()
  {
    max_x = max_x.max(robot.pos.x.abs() as f64);
    max_y = max_y.max(robot.pos.y.abs() as f64);
  }

  if max_x <= 1.25 && max_y <= 1.25 {
    FieldScale {
      half_x: 0.5,
      half_y: 0.5,
      goal_half_width: 0.085,
    }
  } else {
    FieldScale {
      half_x: max_x.max(4.5),
      half_y: max_y.max(3.0),
      goal_half_width: 0.5,
    }
  }
}

fn nearest_robot(ball: Point, robots: &[Option<crate::RobotState>; 16]) -> (Option<usize>, f64) {
  let mut best_idx = None;
  let mut best_dist = f64::INFINITY;

  for (idx, robot) in robots.iter().enumerate() {
    let Some(robot) = robot else {
      continue;
    };

    let dist = distance(ball, point(robot.pos.x, robot.pos.y));
    if dist < best_dist {
      best_idx = Some(idx);
      best_dist = dist;
    }
  }

  (best_idx, best_dist)
}

fn defensive_danger(ball: Point, scale: FieldScale) -> f64 {
  let defensive_depth = (-ball.x * OWN_ATTACK_SIGN / scale.half_x).clamp(0.0, 1.0);
  let centrality = (-(ball.y.abs() / (scale.goal_half_width * 2.6)).powi(2)).exp();

  defensive_depth.powi(2) * (0.45 + 0.55 * centrality)
}

fn shot_lane_quality(ball: Point, scale: FieldScale) -> f64 {
  let goal_progress = (ball.x * OWN_ATTACK_SIGN / scale.half_x).clamp(-1.0, 1.0);
  let centrality = (-(ball.y.abs() / (scale.goal_half_width * 2.2)).powi(2)).exp();

  0.5 * (goal_progress + 1.0) * centrality
}

fn block_line_score(robot: Point, ball: Point, scale: FieldScale) -> f64 {
  let own_goal = point(-OWN_ATTACK_SIGN * scale.half_x, 0.0);
  let goal_to_ball = point(ball.x - own_goal.x, ball.y - own_goal.y);
  let goal_to_robot = point(robot.x - own_goal.x, robot.y - own_goal.y);
  let line_len_sq = goal_to_ball.x * goal_to_ball.x + goal_to_ball.y * goal_to_ball.y;

  if line_len_sq <= 1e-9 {
    return 0.0;
  }

  let t = ((goal_to_robot.x * goal_to_ball.x + goal_to_robot.y * goal_to_ball.y) / line_len_sq)
    .clamp(0.0, 1.0);
  let closest = point(
    own_goal.x + goal_to_ball.x * t,
    own_goal.y + goal_to_ball.y * t,
  );
  let line_dist = distance(robot, closest);
  let lane_width = (scale.half_y * 0.12).max(possession_radius(scale));
  let between = if t > 0.05 && t < 0.95 { 1.0 } else { 0.35 };

  between * (1.0 - line_dist / lane_width).clamp(0.0, 1.0)
}

fn receiver_open_score(
  receiver: Point,
  opponents: &[Option<crate::RobotState>; 16],
  scale: FieldScale,
) -> f64 {
  let (_, dist) = nearest_robot(receiver, opponents);

  if !dist.is_finite() {
    return 1.0;
  }

  (dist / (scale.half_x * 0.18)).clamp(0.0, 1.0)
}

fn robot_has_ball(dist_to_ball: f64, opp_dist: f64, scale: FieldScale) -> bool {
  dist_to_ball < possession_radius(scale) && dist_to_ball <= opp_dist * 1.15
}

fn possession_radius(scale: FieldScale) -> f64 {
  (scale.half_x * 0.04).max(0.018)
}

fn target_is_reasonable(target: Point, scale: FieldScale) -> bool {
  target.x.abs() <= scale.half_x * 1.20 && target.y.abs() <= scale.half_y * 1.20
}

fn ball_outside_field(ball: Point, scale: FieldScale) -> bool {
  ball.x.abs() > scale.half_x * 1.04 || ball.y.abs() > scale.half_y * 1.04
}

fn inside_goal_mouth(ball: Point, scale: FieldScale) -> bool {
  ball.y.abs() <= scale.goal_half_width && ball.x.abs() >= scale.half_x * 0.96
}

fn vector_alignment(a: Point, b: Point) -> f64 {
  let a_len = a.x.hypot(a.y);
  let b_len = b.x.hypot(b.y);

  if a_len <= 1e-9 || b_len <= 1e-9 {
    0.0
  } else {
    ((a.x * b.x + a.y * b.y) / (a_len * b_len)).clamp(-1.0, 1.0)
  }
}

fn distance(a: Point, b: Point) -> f64 {
  (a.x - b.x).hypot(a.y - b.y)
}

fn normalized_delta(before: f64, after: f64, scale: f64) -> f64 {
  if before.is_finite() && after.is_finite() {
    ((before - after) / scale.max(1e-6)).clamp(-1.0, 1.0)
  } else {
    0.0
  }
}

fn point<X: Into<f64>, Y: Into<f64>>(x: X, y: Y) -> Point {
  Point {
    x: finite(x.into()),
    y: finite(y.into()),
  }
}

fn finite(v: f64) -> f64 {
  if v.is_finite() { v } else { 0.0 }
}
