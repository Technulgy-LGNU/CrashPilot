use core_dump::types::cp_types::{FieldData, Robot};
use core_dump::vec::types::Vec2;

use crate::geometry::{EPSILON, clamp_magnitude, is_finite, normalize_angle};
use crate::global_planner::{GlobalPlan, PlanningWorld, path_is_clear, plan_path, track_path};
use crate::local_avoidance::avoid_velocity;
use crate::types::{
  ConfigError, CrashfinderConfig, CrashfinderDebug, CrashfinderPath, CrashfinderRequest,
  PlannerStatus, TEAM_BLUE, TEAM_YELLOW,
};

/// Stateful hybrid global/local path planner.
///
/// Construct one instance per independently controlled robot, then call
/// [`Self::plan`] on every control tick. A single instance can be reassigned to
/// another robot; doing so automatically clears its cached route and velocity
/// history.
pub struct Crashfinder {
  config: CrashfinderConfig,
  cached_plan: Option<CachedPlan>,
  plan_age_seconds: f32,
  last_robot: Option<(u8, u8)>,
  last_commanded_velocity: Vec2<f32>,
  last_commanded_angular_velocity: f32,
}

impl Crashfinder {
  pub fn new(config: CrashfinderConfig) -> Result<Self, ConfigError> {
    config.validate()?;
    Ok(Self {
      config,
      cached_plan: None,
      plan_age_seconds: f32::INFINITY,
      last_robot: None,
      last_commanded_velocity: Vec2::zero(),
      last_commanded_angular_velocity: 0.0,
    })
  }

  #[inline]
  pub fn config(&self) -> &CrashfinderConfig {
    &self.config
  }

  pub fn set_config(&mut self, config: CrashfinderConfig) -> Result<(), ConfigError> {
    config.validate()?;
    self.config = config;
    self.cached_plan = None;
    self.plan_age_seconds = f32::INFINITY;
    Ok(())
  }

  pub fn reset(&mut self) {
    self.cached_plan = None;
    self.plan_age_seconds = f32::INFINITY;
    self.last_robot = None;
    self.last_commanded_velocity = Vec2::zero();
    self.last_commanded_angular_velocity = 0.0;
  }

  /// Produces world-frame linear velocity in mm/s and angular velocity in
  /// mrad/s. Invalid perception or timing data produces a safe stopped command
  /// with the reason exposed through `debug.status`.
  pub fn plan(&mut self, request: &CrashfinderRequest, dt_seconds: f32) -> CrashfinderPath {
    let mut debug = CrashfinderDebug::default();
    if !request_is_valid(request) || !dt_seconds.is_finite() || dt_seconds <= 0.0 {
      debug.status = PlannerStatus::InvalidInput;
      self.clear_motion_state();
      return CrashfinderPath::stopped(debug);
    }

    let Some(controlled_robot) = request
      .robots
      .iter()
      .find(|robot| robot.robot_id == request.robot_id && robot.team == request.robot_team)
    else {
      debug.status = PlannerStatus::RobotNotFound;
      self.clear_motion_state();
      return CrashfinderPath::stopped(debug);
    };
    if !is_finite(controlled_robot.pos) || !controlled_robot.orientation.is_finite() {
      debug.status = PlannerStatus::InvalidInput;
      self.clear_motion_state();
      return CrashfinderPath::stopped(debug);
    }

    let robot_key = (request.robot_team, request.robot_id);
    if self.last_robot != Some(robot_key) {
      self.cached_plan = None;
      self.plan_age_seconds = f32::INFINITY;
      self.last_commanded_velocity = Vec2::zero();
      self.last_commanded_angular_velocity = 0.0;
      self.last_robot = Some(robot_key);
    }

    let current_velocity = controlled_robot
      .vel
      .filter(|velocity| is_finite(*velocity))
      .unwrap_or(self.last_commanded_velocity);
    let current_angular_velocity = controlled_robot
      .angular_vel
      .filter(|velocity| velocity.is_finite())
      .map(|velocity| velocity * 1_000.0)
      .unwrap_or(self.last_commanded_angular_velocity);

    let Some(world) = PlanningWorld::from_request(request, &self.config, controlled_robot.pos)
    else {
      debug.status = PlannerStatus::InvalidInput;
      self.clear_motion_state();
      return CrashfinderPath::stopped(debug);
    };
    debug.usable_min = world.bounds_min;
    debug.usable_max = world.bounds_max;
    debug.obstacles = world.debug_obstacles();
    let effective_target = world.constrain_target(request.end_position, controlled_robot.pos);
    debug.effective_target = effective_target;

    let angular_velocity = angular_command(
      controlled_robot.orientation,
      current_angular_velocity,
      request.target_orientation,
      dt_seconds,
      &self.config,
    );

    if let Some(escape_direction) = ball_escape_direction(request, controlled_robot, &world) {
      self.cached_plan = None;
      self.plan_age_seconds = f32::INFINITY;
      debug.status = PlannerStatus::EscapingBall;
      let ball_position = request
        .ball
        .map(|ball| ball.pos)
        .unwrap_or(controlled_robot.pos);
      let escape_distance = request.avoidance_zone + self.config.path_lookahead_mm;
      let requested_escape_target = ball_position + escape_direction * escape_distance;
      let escape_target = world.constrain_target(requested_escape_target, controlled_robot.pos);
      debug.local_target = escape_target;
      debug.raw_path = vec![controlled_robot.pos, escape_target];
      debug.smoothed_path = debug.raw_path.clone();
      let preferred_velocity = escape_direction * self.config.ball_escape_speed_mm_s;
      debug.preferred_velocity = preferred_velocity;
      let avoidance = avoid_velocity(
        request,
        &self.config,
        controlled_robot,
        current_velocity,
        preferred_velocity,
        dt_seconds,
        true,
      );
      debug.avoidance_velocity = avoidance.velocity;
      debug.avoidance_lines = avoidance.lines;
      let rule_safe_velocity = rule_safe_velocity(
        &world,
        controlled_robot.pos,
        avoidance.velocity,
        dt_seconds.max(self.config.global_prediction_horizon_seconds),
      );
      let command = rate_limit_velocity(
        current_velocity,
        rule_safe_velocity,
        dt_seconds,
        &self.config,
      );
      return self.finish(command, angular_velocity, debug);
    }

    if controlled_robot.pos.distance(effective_target) <= self.config.position_tolerance_mm {
      debug.status = PlannerStatus::AtTarget;
      debug.local_target = effective_target;
      debug.raw_path = vec![controlled_robot.pos];
      debug.smoothed_path = debug.raw_path.clone();
      let avoidance = avoid_velocity(
        request,
        &self.config,
        controlled_robot,
        current_velocity,
        Vec2::zero(),
        dt_seconds,
        false,
      );
      debug.avoidance_velocity = avoidance.velocity;
      debug.avoidance_lines = avoidance.lines;
      let command = rate_limit_velocity(
        current_velocity,
        rule_safe_velocity(
          &world,
          controlled_robot.pos,
          avoidance.velocity,
          dt_seconds.max(self.config.global_prediction_horizon_seconds),
        ),
        dt_seconds,
        &self.config,
      );
      return self.finish(command, angular_velocity, debug);
    }

    self.plan_age_seconds += dt_seconds;
    let signature = PlanSignature::new(request, effective_target);
    let should_replan = self.cached_plan.as_ref().is_none_or(|cached| {
      !cached
        .signature
        .matches(&signature, self.config.grid_resolution_mm)
        || self.plan_age_seconds >= self.config.replan_interval_seconds
        || (cached.plan.found()
          && !path_is_clear(&cached.plan.smoothed_path, controlled_robot.pos, &world))
    });
    if should_replan {
      self.cached_plan = Some(CachedPlan {
        signature,
        plan: plan_path(&world, controlled_robot.pos, effective_target, &self.config),
      });
      self.plan_age_seconds = 0.0;
      debug.replanned = true;
    }

    let cached = self
      .cached_plan
      .as_ref()
      .expect("a plan is cached after replanning");
    copy_plan_debug(&cached.plan, &mut debug);
    if !cached.plan.found() {
      debug.status = PlannerStatus::NoPath;
      debug.local_target = effective_target;
      let avoidance = avoid_velocity(
        request,
        &self.config,
        controlled_robot,
        current_velocity,
        Vec2::zero(),
        dt_seconds,
        true,
      );
      debug.avoidance_velocity = avoidance.velocity;
      debug.avoidance_lines = avoidance.lines;
      let command = rate_limit_velocity(
        current_velocity,
        rule_safe_velocity(
          &world,
          controlled_robot.pos,
          avoidance.velocity,
          dt_seconds.max(self.config.global_prediction_horizon_seconds),
        ),
        dt_seconds,
        &self.config,
      );
      return self.finish(command, angular_velocity, debug);
    }

    let Some(tracking) = track_path(
      &cached.plan.smoothed_path,
      controlled_robot.pos,
      self.config.path_lookahead_mm,
      &world,
    ) else {
      debug.status = PlannerStatus::NoPath;
      let command = rate_limit_velocity(current_velocity, Vec2::zero(), dt_seconds, &self.config);
      self.cached_plan = None;
      return self.finish(command, angular_velocity, debug);
    };

    debug.status = PlannerStatus::FollowingPath;
    debug.local_target = tracking.local_target;
    let target_delta = tracking.local_target - controlled_robot.pos;
    let direction = if target_delta.norm_squared() > EPSILON * EPSILON {
      target_delta.normalized()
    } else {
      (effective_target - controlled_robot.pos).normalized()
    };
    let braking_distance =
      (tracking.remaining_distance - self.config.position_tolerance_mm).max(0.0);
    let braking_speed = (2.0 * self.config.max_linear_deceleration_mm_s2 * braking_distance).sqrt();
    let corner_speed = if tracking.curvature > EPSILON {
      (self.config.max_lateral_acceleration_mm_s2 / tracking.curvature).sqrt()
    } else {
      self.config.max_linear_speed_mm_s
    };
    let mut speed = self
      .config
      .max_linear_speed_mm_s
      .min(braking_speed)
      .min(corner_speed);
    let minimum_speed_braking_distance =
      self.config.min_linear_speed_mm_s.powi(2) / (2.0 * self.config.max_linear_deceleration_mm_s2);
    if braking_distance >= minimum_speed_braking_distance
      && corner_speed >= self.config.min_linear_speed_mm_s
    {
      speed = speed.max(self.config.min_linear_speed_mm_s);
    }
    let preferred_velocity = direction * speed;
    debug.preferred_velocity = preferred_velocity;

    let avoidance = avoid_velocity(
      request,
      &self.config,
      controlled_robot,
      current_velocity,
      preferred_velocity,
      dt_seconds,
      true,
    );
    debug.avoidance_velocity = avoidance.velocity;
    debug.avoidance_lines = avoidance.lines;
    let command = rate_limit_velocity(
      current_velocity,
      rule_safe_velocity(
        &world,
        controlled_robot.pos,
        avoidance.velocity,
        dt_seconds.max(self.config.global_prediction_horizon_seconds),
      ),
      dt_seconds,
      &self.config,
    );
    self.finish(command, angular_velocity, debug)
  }

  fn finish(
    &mut self,
    command: Vec2<f32>,
    angular_velocity: f32,
    mut debug: CrashfinderDebug,
  ) -> CrashfinderPath {
    debug.commanded_velocity = command;
    self.last_commanded_velocity = command;
    self.last_commanded_angular_velocity = angular_velocity;
    CrashfinderPath {
      robot_vel: command,
      angular_vel: angular_velocity,
      debug,
    }
  }

  fn clear_motion_state(&mut self) {
    self.cached_plan = None;
    self.plan_age_seconds = f32::INFINITY;
    self.last_robot = None;
    self.last_commanded_velocity = Vec2::zero();
    self.last_commanded_angular_velocity = 0.0;
  }
}

impl Default for Crashfinder {
  fn default() -> Self {
    Self::new(CrashfinderConfig::default()).expect("default crashfinder config must be valid")
  }
}

#[derive(Debug, Clone)]
struct CachedPlan {
  signature: PlanSignature,
  plan: GlobalPlan,
}

#[derive(Debug, Clone, Copy)]
struct PlanSignature {
  robot_id: u8,
  robot_team: u8,
  avoid_ball: bool,
  avoid_penalty: bool,
  avoidance_zone: f32,
  field: FieldData,
  effective_target: Vec2<f32>,
}

impl PlanSignature {
  fn new(request: &CrashfinderRequest, effective_target: Vec2<f32>) -> Self {
    Self {
      robot_id: request.robot_id,
      robot_team: request.robot_team,
      avoid_ball: request.avoid_ball(),
      avoid_penalty: request.avoid_penalty(),
      avoidance_zone: request.avoidance_zone,
      field: request.field_data,
      effective_target,
    }
  }

  fn matches(&self, other: &Self, resolution: f32) -> bool {
    self.robot_id == other.robot_id
      && self.robot_team == other.robot_team
      && self.avoid_ball == other.avoid_ball
      && self.avoid_penalty == other.avoid_penalty
      && self.avoidance_zone == other.avoidance_zone
      && self.field == other.field
      && self.effective_target.distance(other.effective_target) <= resolution * 0.1
  }
}

fn copy_plan_debug(plan: &GlobalPlan, debug: &mut CrashfinderDebug) {
  debug.search_nodes = plan.search_nodes.clone();
  debug.search_edges = plan.search_edges.clone();
  debug.raw_path = plan.raw_path.clone();
  debug.smoothed_path = plan.smoothed_path.clone();
}

fn request_is_valid(request: &CrashfinderRequest) -> bool {
  let field = request.field_data;
  let valid_team = request.robot_team == TEAM_YELLOW || request.robot_team == TEAM_BLUE;
  valid_team
    && is_finite(request.end_position)
    && request.target_orientation.is_finite()
    && request.avoidance_zone.is_finite()
    && request.avoidance_zone >= 0.0
    && field.width.is_finite()
    && field.width > 0.0
    && field.height.is_finite()
    && field.height > 0.0
    && field.runoff_area.is_finite()
    && field.runoff_area >= 0.0
    && field.goal_width.is_finite()
    && field.goal_width >= 0.0
    && field.penalty_area_width.is_finite()
    && field.penalty_area_width >= 0.0
    && field.penalty_area_width <= field.width
    && field.penalty_area_height.is_finite()
    && field.penalty_area_height >= 0.0
    && field.penalty_area_height <= field.height
}

fn ball_escape_direction(
  request: &CrashfinderRequest,
  controlled_robot: &Robot,
  world: &PlanningWorld,
) -> Option<Vec2<f32>> {
  if !request.avoid_ball() || request.avoidance_zone <= 0.0 {
    return None;
  }
  let ball = request.ball?;
  if !is_finite(ball.pos) || controlled_robot.pos.distance(ball.pos) >= request.avoidance_zone {
    return None;
  }

  let separation = controlled_robot.pos - ball.pos;
  if separation.norm_squared() > EPSILON * EPSILON {
    return Some(separation.normalized());
  }
  let toward_target = request.end_position - ball.pos;
  if toward_target.norm_squared() > EPSILON * EPSILON {
    return Some(toward_target.normalized());
  }

  let candidates = [
    (world.bounds_max.x - ball.pos.x, Vec2::new(1.0, 0.0)),
    (ball.pos.x - world.bounds_min.x, Vec2::new(-1.0, 0.0)),
    (world.bounds_max.y - ball.pos.y, Vec2::new(0.0, 1.0)),
    (ball.pos.y - world.bounds_min.y, Vec2::new(0.0, -1.0)),
  ];
  candidates
    .into_iter()
    .max_by(|left, right| left.0.total_cmp(&right.0))
    .map(|(_, direction)| direction)
}

fn rule_safe_velocity(
  world: &PlanningWorld,
  position: Vec2<f32>,
  velocity: Vec2<f32>,
  horizon: f32,
) -> Vec2<f32> {
  if velocity.norm_squared() <= EPSILON * EPSILON {
    return Vec2::zero();
  }
  if world.segment_is_rule_free(position, position + velocity * horizon) {
    return velocity;
  }

  let mut lower = 0.0;
  let mut upper = 1.0;
  for _ in 0..12 {
    let scale = (lower + upper) * 0.5;
    if world.segment_is_rule_free(position, position + velocity * horizon * scale) {
      lower = scale;
    } else {
      upper = scale;
    }
  }
  velocity * lower
}

fn rate_limit_velocity(
  current: Vec2<f32>,
  requested: Vec2<f32>,
  dt_seconds: f32,
  config: &CrashfinderConfig,
) -> Vec2<f32> {
  let requested = clamp_magnitude(requested, config.max_linear_speed_mm_s);
  let delta = requested - current;
  if delta.norm_squared() <= EPSILON * EPSILON {
    return requested;
  }

  let slowing = requested.norm() + EPSILON < current.norm()
    || (current.norm_squared() > EPSILON * EPSILON && current.dot(&requested) <= 0.0);
  let rate = if slowing {
    config.max_linear_deceleration_mm_s2
  } else {
    config.max_linear_acceleration_mm_s2
  };
  clamp_magnitude(
    current + clamp_magnitude(delta, rate * dt_seconds),
    config.max_linear_speed_mm_s,
  )
}

fn angular_command(
  current_orientation: f32,
  current_velocity_mrad_s: f32,
  target_orientation: f32,
  dt_seconds: f32,
  config: &CrashfinderConfig,
) -> f32 {
  let error = normalize_angle(target_orientation - current_orientation);
  let desired = if error.abs() <= config.orientation_tolerance_rad {
    0.0
  } else {
    let braking_speed =
      (2.0 * config.max_angular_deceleration_mrad_s2 * error.abs() * 1_000.0).sqrt();
    error.signum() * braking_speed.min(config.max_angular_speed_mrad_s)
  };
  let slowing = desired.abs() + EPSILON < current_velocity_mrad_s.abs()
    || (current_velocity_mrad_s != 0.0 && desired.signum() != current_velocity_mrad_s.signum());
  let rate = if slowing {
    config.max_angular_deceleration_mrad_s2
  } else {
    config.max_angular_acceleration_mrad_s2
  };
  let maximum_change = rate * dt_seconds;
  (current_velocity_mrad_s
    + (desired - current_velocity_mrad_s).clamp(-maximum_change, maximum_change))
  .clamp(
    -config.max_angular_speed_mrad_s,
    config.max_angular_speed_mrad_s,
  )
}

#[cfg(test)]
mod tests {
  use core_dump::types::cp_types::{Ball, Robot};

  use super::*;
  use crate::types::{FLAG_ALLOW_PENALTY_AREAS, FLAG_AVOID_BALL};

  fn field() -> FieldData {
    FieldData {
      height: 6_000.0,
      width: 9_000.0,
      runoff_area: 300.0,
      goal_width: 1_000.0,
      penalty_area_width: 1_000.0,
      penalty_area_height: 2_000.0,
    }
  }

  fn robot(id: u8, team: u8, position: Vec2<f32>) -> Robot {
    Robot {
      robot_id: id,
      team,
      pos: position,
      ..Robot::default()
    }
  }

  fn request(start: Vec2<f32>, target: Vec2<f32>) -> CrashfinderRequest {
    CrashfinderRequest {
      robot_id: 0,
      robot_team: TEAM_YELLOW,
      robots: vec![robot(0, TEAM_YELLOW, start)],
      ball: None,
      field_data: field(),
      end_position: target,
      target_orientation: 0.0,
      flags: FLAG_ALLOW_PENALTY_AREAS,
      avoidance_zone: 500.0,
    }
  }

  #[test]
  fn acceleration_is_limited_on_the_first_tick() {
    let mut planner = Crashfinder::default();
    let request = request(Vec2::new(-2_000.0, 0.0), Vec2::new(2_000.0, 0.0));
    let result = planner.plan(&request, 0.02);
    assert!((result.robot_vel.norm() - 80.0).abs() < 0.01);
    assert_eq!(result.debug.status, PlannerStatus::FollowingPath);
  }

  #[test]
  fn ball_overlap_at_target_commands_active_escape() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::zero(), Vec2::zero());
    request.flags |= FLAG_AVOID_BALL;
    request.ball = Some(Ball {
      pos: Vec2::new(100.0, 0.0),
      vel: None,
    });
    let result = planner.plan(&request, 0.02);
    assert_eq!(result.debug.status, PlannerStatus::EscapingBall);
    assert!(result.robot_vel.x < 0.0);
    assert!(!result.debug.avoidance_lines.is_empty());
  }

  #[test]
  fn target_inside_ball_zone_is_replaced_by_closest_legal_target() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::new(-2_000.0, 0.0), Vec2::zero());
    request.flags |= FLAG_AVOID_BALL;
    request.ball = Some(Ball {
      pos: Vec2::zero(),
      vel: None,
    });
    let result = planner.plan(&request, 0.02);
    assert!(
      (result.debug.effective_target.distance(Vec2::zero()) - request.avoidance_zone).abs() < 0.01
    );
    assert!(result.debug.effective_target.x < 0.0);
  }

  #[test]
  fn obstacle_forces_a_debuggable_global_detour() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::new(-2_000.0, 0.0), Vec2::new(2_000.0, 0.0));
    request.robots.push(robot(1, TEAM_BLUE, Vec2::zero()));
    let result = planner.plan(&request, 0.02);
    assert_eq!(result.debug.status, PlannerStatus::FollowingPath);
    assert!(result.debug.raw_path.len() > 2);
    assert!(!result.debug.search_nodes.is_empty());
    assert!(!result.debug.search_edges.is_empty());
    assert!(
      result
        .debug
        .smoothed_path
        .windows(2)
        .all(|segment| Vec2::zero().distance_to_segment(segment[0], segment[1]) >= 220.0)
    );
  }

  #[test]
  fn at_target_ignores_robot_repulsion_and_brakes() {
    let mut planner = Crashfinder::default();
    let mut own = robot(0, TEAM_YELLOW, Vec2::zero());
    own.vel = Some(Vec2::new(500.0, 0.0));
    let mut request = request(Vec2::zero(), Vec2::zero());
    request.robots = vec![own, robot(1, TEAM_BLUE, Vec2::new(100.0, 0.0))];
    let result = planner.plan(&request, 0.02);
    assert_eq!(result.debug.status, PlannerStatus::AtTarget);
    assert!(result.debug.avoidance_lines.is_empty());
    assert!((result.robot_vel.x - 420.0).abs() < 0.01);
  }

  #[test]
  fn moving_ball_is_avoided_while_holding_target() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::zero(), Vec2::zero());
    request.flags |= FLAG_AVOID_BALL;
    request.ball = Some(Ball {
      pos: Vec2::new(600.0, 0.0),
      vel: Some(Vec2::new(-1_000.0, 0.0)),
    });
    let result = planner.plan(&request, 0.02);
    assert_eq!(result.debug.status, PlannerStatus::AtTarget);
    assert!(result.robot_vel.x < 0.0);
    assert!(
      result
        .debug
        .avoidance_lines
        .iter()
        .all(|line| line.source == crate::types::AvoidanceSource::Ball)
    );
  }

  #[test]
  fn angular_controller_uses_the_shortest_wrapped_direction() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::zero(), Vec2::zero());
    request.robots[0].orientation = 179.0_f32.to_radians();
    request.target_orientation = -179.0_f32.to_radians();
    let result = planner.plan(&request, 0.02);
    assert!(result.angular_vel > 0.0);
    assert!(result.angular_vel <= 240.0 + EPSILON);
  }

  #[test]
  fn missing_controlled_robot_stops_with_a_status() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::zero(), Vec2::new(1_000.0, 0.0));
    request.robots.clear();
    let result = planner.plan(&request, 0.02);
    assert_eq!(result.debug.status, PlannerStatus::RobotNotFound);
    assert_eq!(result.robot_vel, Vec2::zero());
  }

  #[test]
  fn team_and_id_together_select_the_controlled_robot() {
    let mut planner = Crashfinder::default();
    let mut request = request(Vec2::new(-2_000.0, 0.0), Vec2::new(-1_000.0, 0.0));
    request
      .robots
      .insert(0, robot(0, TEAM_BLUE, Vec2::new(2_000.0, 0.0)));
    let result = planner.plan(&request, 0.02);
    assert!(result.robot_vel.x > 0.0);
    assert!(result.debug.obstacles.iter().any(|obstacle| {
      obstacle.kind
        == crate::types::DebugObstacleKind::Robot {
          team: TEAM_BLUE,
          id: 0,
        }
    }));
  }

  #[test]
  fn penalty_flag_switches_both_areas_together() {
    let start = Vec2::new(2_500.0, 0.0);
    let target_inside_right_penalty = Vec2::new(4_300.0, 0.0);

    let mut avoiding_planner = Crashfinder::default();
    let mut avoiding_request = request(start, target_inside_right_penalty);
    avoiding_request.flags = 0;
    let avoiding = avoiding_planner.plan(&avoiding_request, 0.02);
    assert_eq!(
      avoiding
        .debug
        .obstacles
        .iter()
        .filter(|obstacle| obstacle.kind == crate::types::DebugObstacleKind::PenaltyArea)
        .count(),
      2
    );
    assert_ne!(avoiding.debug.effective_target, target_inside_right_penalty);
    assert!(avoiding.debug.raw_path.len() > 2);

    let mut allowed_planner = Crashfinder::default();
    let allowed_request = request(start, target_inside_right_penalty);
    let allowed = allowed_planner.plan(&allowed_request, 0.02);
    assert_eq!(allowed.debug.effective_target, target_inside_right_penalty);
    assert_eq!(
      allowed.debug.raw_path,
      vec![start, target_inside_right_penalty]
    );
  }

  #[test]
  fn unchanged_route_is_reused_between_replan_intervals() {
    let mut planner = Crashfinder::default();
    let request = request(Vec2::new(-2_000.0, 0.0), Vec2::new(2_000.0, 0.0));
    let first = planner.plan(&request, 0.02);
    let second = planner.plan(&request, 0.02);
    assert!(first.debug.replanned);
    assert!(!second.debug.replanned);
    assert_eq!(first.debug.smoothed_path, second.debug.smoothed_path);
  }

  #[test]
  fn invalid_config_is_rejected() {
    let config = CrashfinderConfig {
      max_linear_acceleration_mm_s2: 0.0,
      ..CrashfinderConfig::default()
    };
    assert!(Crashfinder::new(config).is_err());
  }
}
