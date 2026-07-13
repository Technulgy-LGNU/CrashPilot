use crate::config::WorldModelConfig;
use crate::utils::FieldSetup;
use core_dump::proto::{
  CpCommand, CpTask, KickedBall, RobotId, SslWrapperPacket, Team, TrackedBall, TrackedFrame,
  TrackedRobot, TrackerWrapperPacket, Vector2, Vector3,
};
use std::collections::{HashMap, HashSet, VecDeque};

const TIME_EPSILON_S: f64 = 1.0e-6;
const GROUP_EPSILON_S: f64 = 0.0025;
const MIN_FIT_SPAN_S: f64 = 0.025;

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct WorldTime(pub f64);

#[derive(Debug, Clone, Copy, Default)]
pub struct VectorQuality {
  pub confidence: f32,
  pub covariance_xx: f32,
  pub covariance_xy: f32,
  pub covariance_yy: f32,
  pub p95_major_radius: f32,
  pub p95_minor_radius: f32,
  pub p95_angle_rad: f32,
  pub p95_x_min: f32,
  pub p95_x_max: f32,
  pub p95_y_min: f32,
  pub p95_y_max: f32,
}

impl VectorQuality {
  fn from_covariance(confidence: f32, xx: f64, xy: f64, yy: f64) -> Self {
    let xx = xx.max(1.0);
    let yy = yy.max(1.0);
    let trace = xx + yy;
    let disc = (((xx - yy) * 0.5).powi(2) + xy * xy).sqrt();
    let lambda_major = (trace * 0.5 + disc).max(1.0);
    let lambda_minor = (trace * 0.5 - disc).max(1.0);
    let angle = 0.5 * (2.0 * xy).atan2(xx - yy);
    let major = (5.991 * lambda_major).sqrt();
    let minor = (5.991 * lambda_minor).sqrt();
    let x_radius = (5.991 * xx).sqrt();
    let y_radius = (5.991 * yy).sqrt();
    Self {
      confidence: confidence.clamp(0.0, 1.0),
      covariance_xx: xx as f32,
      covariance_xy: xy as f32,
      covariance_yy: yy as f32,
      p95_major_radius: major as f32,
      p95_minor_radius: minor as f32,
      p95_angle_rad: angle as f32,
      p95_x_min: -x_radius as f32,
      p95_x_max: x_radius as f32,
      p95_y_min: -y_radius as f32,
      p95_y_max: y_radius as f32,
    }
  }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AngularQuality {
  pub confidence: f32,
  pub variance_deg2: f32,
  pub p95_half_width_deg: f32,
  pub p95_lower_deg: f32,
  pub p95_upper_deg: f32,
  pub p95_wraps_zero: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EstimateQuality {
  pub valid: bool,
  pub overall_confidence: f32,
  pub measurement_age_s: f32,
  pub position: VectorQuality,
  pub velocity: VectorQuality,
  pub heading: Option<AngularQuality>,
  pub trajectory_confidence: f32,
  pub prediction_100ms: VectorQuality,
  pub prediction_250ms: VectorQuality,
  pub prediction_500ms: VectorQuality,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MotionEstimate {
  pub pos: [f64; 2],
  pub vel: [f64; 2],
  pub accel: [f64; 2],
  pub heading_deg: Option<f64>,
  pub angular_vel_deg_s: Option<f64>,
  pub quality: EstimateQuality,
}

#[derive(Debug, Clone)]
pub struct CleanWorldSnapshot {
  pub timestamp: WorldTime,
  pub ball: Option<MotionEstimate>,
  pub robots: HashMap<(i32, u32), MotionEstimate>,
  pub tracked_frame: TrackerWrapperPacket,
}

#[derive(Debug, Clone)]
struct Observation {
  time: WorldTime,
  pos: [f64; 2],
  heading_rad: Option<f64>,
  weight: f64,
  source: String,
}

#[derive(Debug, Clone, Default)]
struct MotionTrack {
  observations: VecDeque<Observation>,
  estimate: Option<MotionEstimate>,
  last_measurement: Option<WorldTime>,
  last_bounce: Option<WorldTime>,
}

#[derive(Debug, Clone, Copy, Default)]
struct AxisFit {
  pos: f64,
  vel: f64,
  accel: f64,
  residual_variance: f64,
  velocity_variance: f64,
}

#[derive(Debug, Clone, Copy)]
struct WallModel {
  coordinates: [f64; 4], // +X, +Y, -X, -Y
  samples: [u32; 4],
}

impl WallModel {
  fn new(field: FieldSetup, offset: f64) -> Self {
    Self {
      coordinates: [
        field.width as f64 * 0.5 + offset,
        field.height as f64 * 0.5 + offset,
        -(field.width as f64 * 0.5 + offset),
        -(field.height as f64 * 0.5 + offset),
      ],
      samples: [0; 4],
    }
  }

  fn update_geometry_prior(&mut self, field: FieldSetup, offset: f64) {
    let prior = Self::new(field, offset);
    for index in 0..4 {
      if self.samples[index] == 0 {
        self.coordinates[index] = prior.coordinates[index];
      }
    }
  }

  fn observe(&mut self, side: usize, coordinate: f64, field: FieldSetup, offset: f64) {
    let prior = Self::new(field, offset).coordinates[side];
    let bounded = coordinate.clamp(prior - 1_000.0, prior + 1_000.0);
    self.samples[side] = self.samples[side].saturating_add(1);
    let gain = (1.0 / self.samples[side].min(20) as f64).clamp(0.05, 0.25);
    self.coordinates[side] += (bounded - self.coordinates[side]) * gain;
  }
}

#[derive(Debug, Clone, Copy)]
struct BounceTransition {
  reflected_velocity: [f64; 2],
  side: usize,
  inferred_wall_coordinate: f64,
}

#[derive(Debug, Clone)]
pub struct WorldModel {
  config: WorldModelConfig,
  ball: MotionTrack,
  robots: HashMap<(i32, u32), MotionTrack>,
  raw_history: HashMap<String, VecDeque<SslWrapperPacket>>,
  tracked_history: HashMap<String, VecDeque<TrackerWrapperPacket>>,
  seen_raw: HashSet<(String, u32)>,
  seen_tracked: HashSet<(String, u32)>,
  source_clock_offsets: HashMap<String, f64>,
  latest_time: Option<WorldTime>,
  output_frame_number: u32,
  last_output_time: Option<WorldTime>,
  field: FieldSetup,
  walls: WallModel,
}

impl WorldModel {
  pub fn new(config: WorldModelConfig, field: FieldSetup) -> Self {
    let walls = WallModel::new(field, config.wall_offset_mm as f64);
    Self {
      config,
      ball: MotionTrack::default(),
      robots: HashMap::new(),
      raw_history: HashMap::new(),
      tracked_history: HashMap::new(),
      seen_raw: HashSet::new(),
      seen_tracked: HashSet::new(),
      source_clock_offsets: HashMap::new(),
      latest_time: None,
      output_frame_number: 0,
      last_output_time: None,
      field,
      walls,
    }
  }

  pub fn set_field(&mut self, field: FieldSetup) {
    self.field = field;
    self
      .walls
      .update_geometry_prior(field, self.config.wall_offset_mm as f64);
  }

  pub fn latest_raw_packets(&self) -> Vec<SslWrapperPacket> {
    self
      .raw_history
      .values()
      .filter_map(|history| history.back().cloned())
      .collect()
  }

  pub fn latest_tracked_packets(&self) -> Vec<TrackerWrapperPacket> {
    self
      .tracked_history
      .values()
      .filter_map(|history| history.back().cloned())
      .collect()
  }

  pub fn ingest_raw(&mut self, packet: SslWrapperPacket) {
    let Some(frame) = packet.detection.as_ref() else {
      return;
    };
    let source = format!("camera:{}", frame.camera_id);
    if !self.seen_raw.insert((source.clone(), frame.frame_number)) {
      return;
    }
    let time = self.logical_time(&source, frame.t_capture);
    if !valid_time(time) {
      return;
    }
    self.advance_time(time);
    self.push_raw_history(source.clone(), packet.clone());

    if let Some(ball) = select_raw_ball(&self.ball, frame.balls.as_slice(), time) {
      let observation = Observation {
        time,
        pos: [ball.x as f64, ball.y as f64],
        heading_rad: None,
        weight: (ball.confidence as f64).clamp(0.05, 1.0) * 0.75,
        source: source.clone(),
      };
      self.ingest_ball_observation(observation);
    }

    for robot in frame.robots_yellow.iter().chain(frame.robots_blue.iter()) {
      let Some(id) = robot.robot_id else { continue };
      let team = if frame
        .robots_yellow
        .iter()
        .any(|candidate| std::ptr::eq(candidate, robot))
      {
        Team::Yellow as i32
      } else {
        Team::Blue as i32
      };
      self.ingest_robot_observation(
        (team, id),
        Observation {
          time,
          pos: [robot.x as f64, robot.y as f64],
          heading_rad: robot.orientation.map(f64::from),
          weight: (robot.confidence as f64).clamp(0.05, 1.0) * 0.7,
          source: source.clone(),
        },
      );
    }
  }

  pub fn ingest_tracked(&mut self, packet: TrackerWrapperPacket) {
    let Some(frame) = packet.tracked_frame.as_ref() else {
      return;
    };
    let source = tracker_source(&packet);
    if !self
      .seen_tracked
      .insert((source.clone(), frame.frame_number))
    {
      return;
    }
    let time = self.logical_time(&source, frame.timestamp);
    if !valid_time(time) {
      return;
    }
    self.advance_time(time);
    self.push_tracked_history(source.clone(), packet.clone());

    if let Some(ball) = select_tracked_ball(&self.ball, frame.balls.as_slice(), time) {
      self.ingest_ball_observation(Observation {
        time,
        pos: [ball.pos.x as f64 * 1_000.0, ball.pos.y as f64 * 1_000.0],
        heading_rad: None,
        weight: ball.visibility.unwrap_or(0.7) as f64 * 0.65,
        source: source.clone(),
      });
    }

    for robot in &frame.robots {
      let Some(id) = robot.robot_id.id else {
        continue;
      };
      let Some(team) = robot.robot_id.team else {
        continue;
      };
      self.ingest_robot_observation(
        (team, id),
        Observation {
          time,
          pos: [robot.pos.x as f64 * 1_000.0, robot.pos.y as f64 * 1_000.0],
          heading_rad: Some(robot.orientation as f64),
          weight: robot.visibility.unwrap_or(0.7) as f64 * 0.65,
          source: source.clone(),
        },
      );
    }
  }

  pub fn snapshot(
    &mut self,
    own_team: i32,
    own_commands: &HashMap<u32, CpCommand>,
  ) -> Option<CleanWorldSnapshot> {
    let now = self.latest_time?;
    if self.last_output_time != Some(now) {
      self.output_frame_number = self.output_frame_number.wrapping_add(1);
      self.last_output_time = Some(now);
    }
    let ball = fit_track(&mut self.ball, now, &self.config, true, None);
    let mut robots = HashMap::new();
    for (key, track) in &mut self.robots {
      if let Some(mut estimate) = fit_track(track, now, &self.config, false, Some(*key)) {
        if key.0 == own_team
          && let Some(command) = own_commands.get(&key.1)
        {
          apply_own_command_prior(&mut estimate, command, track, &self.config);
        }
        robots.insert(*key, estimate);
      }
    }

    let tracked_frame = cleaned_tracker_packet(now, self.output_frame_number, ball, &robots);
    Some(CleanWorldSnapshot {
      timestamp: now,
      ball,
      robots,
      tracked_frame,
    })
  }

  fn advance_time(&mut self, time: WorldTime) {
    if self.latest_time.is_none_or(|latest| time > latest) {
      self.latest_time = Some(time);
    }
  }

  fn logical_time(&mut self, source: &str, source_time: f64) -> WorldTime {
    let initial_offset = self
      .latest_time
      .filter(|latest| (latest.0 - source_time).abs() > 5.0)
      .map(|latest| latest.0 - source_time)
      .unwrap_or(0.0);
    let offset = *self
      .source_clock_offsets
      .entry(source.to_string())
      .or_insert(initial_offset);
    WorldTime(source_time + offset)
  }

  fn ingest_ball_observation(&mut self, observation: Observation) {
    let bounce = ball_bounce_transition(
      self.ball.estimate,
      self.ball.last_measurement,
      &observation,
      &self.walls,
      &self.config,
    );
    if let Some(bounce) = bounce {
      self.ball.observations.clear();
      self.ball.last_bounce = Some(observation.time);
      self.ball.estimate = Some(MotionEstimate {
        pos: observation.pos,
        vel: bounce.reflected_velocity,
        ..Default::default()
      });
      self.walls.observe(
        bounce.side,
        bounce.inferred_wall_coordinate,
        self.field,
        self.config.wall_offset_mm as f64,
      );
    }
    push_observation(&mut self.ball, observation, &self.config);
  }

  fn ingest_robot_observation(&mut self, key: (i32, u32), observation: Observation) {
    let track = self.robots.entry(key).or_default();
    push_observation(track, observation, &self.config);
  }

  fn push_raw_history(&mut self, source: String, packet: SslWrapperPacket) {
    let history = self.raw_history.entry(source).or_default();
    history.push_back(packet);
    while history.len() > 1_000 {
      history.pop_front();
    }
  }

  fn push_tracked_history(&mut self, source: String, packet: TrackerWrapperPacket) {
    let history = self.tracked_history.entry(source).or_default();
    history.push_back(packet);
    while history.len() > 1_000 {
      history.pop_front();
    }
  }
}

fn tracker_source(packet: &TrackerWrapperPacket) -> String {
  if !packet.uuid.is_empty() {
    format!(
      "{}:{}",
      packet.source_name.as_deref().unwrap_or("tracker"),
      packet.uuid
    )
  } else {
    packet
      .source_name
      .clone()
      .unwrap_or_else(|| "tracker:unknown".to_string())
  }
}

fn valid_time(time: WorldTime) -> bool {
  time.0.is_finite() && time.0 >= 0.0
}

fn push_observation(track: &mut MotionTrack, observation: Observation, config: &WorldModelConfig) {
  if track.observations.back().is_some_and(|last| {
    last.source == observation.source && (last.time.0 - observation.time.0).abs() < TIME_EPSILON_S
  }) {
    return;
  }
  let newest = observation.time;
  track.last_measurement = Some(newest);
  track.observations.push_back(observation);
  let cutoff = newest.0 - config.history_ms as f64 / 1_000.0;
  while track
    .observations
    .front()
    .is_some_and(|observation| observation.time.0 < cutoff)
  {
    track.observations.pop_front();
  }
}

fn select_raw_ball<'a>(
  track: &MotionTrack,
  balls: &'a [core_dump::proto::SslDetectionBall],
  time: WorldTime,
) -> Option<&'a core_dump::proto::SslDetectionBall> {
  let predicted = predict_track(track, time);
  balls
    .iter()
    .filter(|ball| ball.x.is_finite() && ball.y.is_finite())
    .min_by(|a, b| {
      candidate_cost([a.x as f64, a.y as f64], a.confidence as f64, predicted).total_cmp(
        &candidate_cost([b.x as f64, b.y as f64], b.confidence as f64, predicted),
      )
    })
}

fn select_tracked_ball<'a>(
  track: &MotionTrack,
  balls: &'a [TrackedBall],
  time: WorldTime,
) -> Option<&'a TrackedBall> {
  let predicted = predict_track(track, time);
  balls.iter().min_by(|a, b| {
    candidate_cost(
      [a.pos.x as f64 * 1_000.0, a.pos.y as f64 * 1_000.0],
      a.visibility.unwrap_or(0.5) as f64,
      predicted,
    )
    .total_cmp(&candidate_cost(
      [b.pos.x as f64 * 1_000.0, b.pos.y as f64 * 1_000.0],
      b.visibility.unwrap_or(0.5) as f64,
      predicted,
    ))
  })
}

fn candidate_cost(pos: [f64; 2], visibility: f64, predicted: Option<[f64; 2]>) -> f64 {
  let distance = predicted
    .map(|predicted| hypot(pos[0] - predicted[0], pos[1] - predicted[1]))
    .unwrap_or(0.0);
  distance - visibility.clamp(0.0, 1.0) * 100.0
}

fn predict_track(track: &MotionTrack, time: WorldTime) -> Option<[f64; 2]> {
  let estimate = track.estimate?;
  let last = track.last_measurement?;
  let dt = (time.0 - last.0).max(0.0);
  Some([
    estimate.pos[0] + estimate.vel[0] * dt + 0.5 * estimate.accel[0] * dt * dt,
    estimate.pos[1] + estimate.vel[1] * dt + 0.5 * estimate.accel[1] * dt * dt,
  ])
}

fn grouped_observations(track: &MotionTrack, now: WorldTime, window_s: f64) -> Vec<Observation> {
  let mut selected: Vec<_> = track
    .observations
    .iter()
    .filter(|observation| now.0 - observation.time.0 <= window_s + TIME_EPSILON_S)
    .cloned()
    .collect();
  selected.sort_by(|a, b| {
    a.time
      .partial_cmp(&b.time)
      .unwrap_or(std::cmp::Ordering::Equal)
  });

  let mut grouped: Vec<Observation> = Vec::new();
  for observation in selected {
    if let Some(last) = grouped.last_mut()
      && (last.time.0 - observation.time.0).abs() <= GROUP_EPSILON_S
    {
      let old_weight = last.weight;
      let total = (old_weight + observation.weight).max(f64::EPSILON);
      last.pos[0] = (last.pos[0] * old_weight + observation.pos[0] * observation.weight) / total;
      last.pos[1] = (last.pos[1] * old_weight + observation.pos[1] * observation.weight) / total;
      last.weight = total.min(1.0);
      if let Some(heading) = observation.heading_rad {
        last.heading_rad = Some(match last.heading_rad {
          Some(old) => old + angle_difference(heading, old) * observation.weight / total,
          None => heading,
        });
      }
      continue;
    }
    grouped.push(observation);
  }
  grouped
}

fn fit_track(
  track: &mut MotionTrack,
  now: WorldTime,
  config: &WorldModelConfig,
  is_ball: bool,
  _robot_key: Option<(i32, u32)>,
) -> Option<MotionEstimate> {
  let last_measurement = track.last_measurement?;
  let age = (now.0 - last_measurement.0).max(0.0);
  if age > config.invalid_timeout_ms as f64 / 1_000.0 {
    track.estimate = None;
    return None;
  }

  let window_s = config.fit_window_ms as f64 / 1_000.0;
  let observations = grouped_observations(track, now, window_s);
  if observations.is_empty() {
    return track.estimate;
  }
  let reference = observations.last().unwrap().time.0;
  let x_fit = robust_axis_fit(&observations, reference, 0, is_ball, config);
  let y_fit = robust_axis_fit(&observations, reference, 1, is_ball, config);

  let extrapolate = (now.0 - reference).max(0.0);
  let mut estimate = MotionEstimate {
    pos: [
      x_fit.pos + x_fit.vel * extrapolate + 0.5 * x_fit.accel * extrapolate.powi(2),
      y_fit.pos + y_fit.vel * extrapolate + 0.5 * y_fit.accel * extrapolate.powi(2),
    ],
    vel: [
      x_fit.vel + x_fit.accel * extrapolate,
      y_fit.vel + y_fit.accel * extrapolate,
    ],
    accel: [x_fit.accel, y_fit.accel],
    ..Default::default()
  };

  if is_ball {
    clamp_vector(&mut estimate.vel, config.ball_max_speed_mm_s as f64);
    clamp_vector(&mut estimate.accel, config.ball_max_accel_mm_s2 as f64);
  }

  let span = observations
    .first()
    .map(|first| reference - first.time.0)
    .unwrap_or(0.0)
    .max(0.0);
  let support = 1.0 - (-(observations.len() as f64) / 4.0).exp();
  let span_score = (span / 0.12).clamp(0.0, 1.0);
  let residual = ((x_fit.residual_variance + y_fit.residual_variance) * 0.5).sqrt();
  let fit_score = (-residual / if is_ball { 120.0 } else { 80.0 }).exp();
  let freshness = (-age / (config.predict_timeout_ms as f64 / 1_000.0).max(0.001)).exp();
  let position_confidence = (support * fit_score * freshness).clamp(0.0, 1.0) as f32;
  let velocity_confidence = (support * span_score * fit_score * freshness).clamp(0.0, 1.0) as f32;

  let pos_xx = x_fit.residual_variance + x_fit.velocity_variance * extrapolate.powi(2);
  let pos_yy = y_fit.residual_variance + y_fit.velocity_variance * extrapolate.powi(2);
  let pos_xy = residual_cross_covariance(&observations, reference, &x_fit, &y_fit);
  let vel_xx = x_fit.velocity_variance + x_fit.residual_variance * extrapolate;
  let vel_yy = y_fit.velocity_variance + y_fit.residual_variance * extrapolate;

  estimate.quality = EstimateQuality {
    valid: true,
    overall_confidence: (position_confidence * velocity_confidence.sqrt()).clamp(0.0, 1.0),
    measurement_age_s: age as f32,
    position: VectorQuality::from_covariance(position_confidence, pos_xx, pos_xy, pos_yy),
    velocity: VectorQuality::from_covariance(velocity_confidence, vel_xx, 0.0, vel_yy),
    trajectory_confidence: velocity_confidence,
    prediction_100ms: predicted_quality(
      pos_xx,
      pos_xy,
      pos_yy,
      vel_xx,
      vel_yy,
      0.1,
      velocity_confidence,
    ),
    prediction_250ms: predicted_quality(
      pos_xx,
      pos_xy,
      pos_yy,
      vel_xx,
      vel_yy,
      0.25,
      velocity_confidence,
    ),
    prediction_500ms: predicted_quality(
      pos_xx,
      pos_xy,
      pos_yy,
      vel_xx,
      vel_yy,
      0.5,
      velocity_confidence,
    ),
    ..Default::default()
  };

  if !is_ball {
    if let Some((heading, angular_velocity, quality)) =
      fit_heading(&observations, reference, now, freshness)
    {
      estimate.heading_deg = Some(heading);
      estimate.angular_vel_deg_s = Some(angular_velocity);
      estimate.quality.heading = Some(quality);
      estimate.quality.overall_confidence =
        estimate.quality.overall_confidence.min(quality.confidence);
    }
  }

  track.estimate = Some(estimate);
  Some(estimate)
}

fn robust_axis_fit(
  observations: &[Observation],
  reference: f64,
  axis: usize,
  allow_accel: bool,
  config: &WorldModelConfig,
) -> AxisFit {
  let linear = weighted_linear_fit(observations, reference, axis, None);
  let gate = if allow_accel {
    config.ball_outlier_gate_mm as f64
  } else {
    180.0
  };
  let robust_weights: Vec<f64> = observations
    .iter()
    .map(|observation| {
      let t = observation.time.0 - reference;
      let residual = observation.pos[axis] - (linear.pos + linear.vel * t);
      observation.weight * (gate / residual.abs().max(gate)).clamp(0.05, 1.0)
    })
    .collect();
  let linear = weighted_linear_fit(observations, reference, axis, Some(&robust_weights));

  if allow_accel && observations.len() >= 6 {
    if let Some(quadratic) = weighted_quadratic_fit(observations, reference, axis, &robust_weights)
      && quadratic.residual_variance < linear.residual_variance * 0.8
      && quadratic.accel.abs() <= config.ball_max_accel_mm_s2 as f64
    {
      return quadratic;
    }
  }
  linear
}

fn weighted_linear_fit(
  observations: &[Observation],
  reference: f64,
  axis: usize,
  override_weights: Option<&[f64]>,
) -> AxisFit {
  let mut sw = 0.0;
  let mut st = 0.0;
  let mut sy = 0.0;
  let mut stt = 0.0;
  let mut sty = 0.0;
  for (index, observation) in observations.iter().enumerate() {
    let weight = override_weights.map_or(observation.weight, |weights| weights[index]);
    let t = observation.time.0 - reference;
    let y = observation.pos[axis];
    sw += weight;
    st += weight * t;
    sy += weight * y;
    stt += weight * t * t;
    sty += weight * t * y;
  }
  let denominator = sw * stt - st * st;
  let (pos, vel) = if sw > f64::EPSILON && denominator.abs() > 1.0e-12 {
    (
      (sy * stt - st * sty) / denominator,
      (sw * sty - st * sy) / denominator,
    )
  } else {
    (sy / sw.max(f64::EPSILON), 0.0)
  };
  let variance = residual_variance(
    observations,
    reference,
    axis,
    pos,
    vel,
    0.0,
    override_weights,
  );
  AxisFit {
    pos,
    vel,
    accel: 0.0,
    residual_variance: variance.max(4.0),
    velocity_variance: (variance * sw / denominator.abs().max(1.0e-6)).max(25.0),
  }
}

fn weighted_quadratic_fit(
  observations: &[Observation],
  reference: f64,
  axis: usize,
  weights: &[f64],
) -> Option<AxisFit> {
  let mut a = [[0.0; 3]; 3];
  let mut b = [0.0; 3];
  for (index, observation) in observations.iter().enumerate() {
    let t = observation.time.0 - reference;
    let basis = [1.0, t, 0.5 * t * t];
    let weight = weights[index];
    for row in 0..3 {
      b[row] += weight * basis[row] * observation.pos[axis];
      for column in 0..3 {
        a[row][column] += weight * basis[row] * basis[column];
      }
    }
  }
  let solution = solve_3x3(a, b)?;
  let variance = residual_variance(
    observations,
    reference,
    axis,
    solution[0],
    solution[1],
    solution[2],
    Some(weights),
  );
  Some(AxisFit {
    pos: solution[0],
    vel: solution[1],
    accel: solution[2],
    residual_variance: variance.max(4.0),
    velocity_variance: (variance / observations.len().max(1) as f64 / MIN_FIT_SPAN_S.powi(2))
      .max(25.0),
  })
}

fn solve_3x3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
  for pivot in 0..3 {
    let best =
      (pivot..3).max_by(|left, right| a[*left][pivot].abs().total_cmp(&a[*right][pivot].abs()))?;
    if a[best][pivot].abs() < 1.0e-12 {
      return None;
    }
    a.swap(pivot, best);
    b.swap(pivot, best);
    let divisor = a[pivot][pivot];
    for column in pivot..3 {
      a[pivot][column] /= divisor;
    }
    b[pivot] /= divisor;
    for row in 0..3 {
      if row == pivot {
        continue;
      }
      let factor = a[row][pivot];
      for column in pivot..3 {
        a[row][column] -= factor * a[pivot][column];
      }
      b[row] -= factor * b[pivot];
    }
  }
  Some(b)
}

fn residual_variance(
  observations: &[Observation],
  reference: f64,
  axis: usize,
  pos: f64,
  vel: f64,
  accel: f64,
  override_weights: Option<&[f64]>,
) -> f64 {
  let mut weighted_error = 0.0;
  let mut total_weight = 0.0;
  for (index, observation) in observations.iter().enumerate() {
    let weight = override_weights.map_or(observation.weight, |weights| weights[index]);
    let t = observation.time.0 - reference;
    let prediction = pos + vel * t + 0.5 * accel * t * t;
    weighted_error += weight * (observation.pos[axis] - prediction).powi(2);
    total_weight += weight;
  }
  weighted_error / total_weight.max(1.0)
}

fn residual_cross_covariance(
  observations: &[Observation],
  reference: f64,
  x: &AxisFit,
  y: &AxisFit,
) -> f64 {
  let mut covariance = 0.0;
  let mut total = 0.0;
  for observation in observations {
    let t = observation.time.0 - reference;
    let rx = observation.pos[0] - (x.pos + x.vel * t + 0.5 * x.accel * t * t);
    let ry = observation.pos[1] - (y.pos + y.vel * t + 0.5 * y.accel * t * t);
    covariance += observation.weight * rx * ry;
    total += observation.weight;
  }
  covariance / total.max(1.0)
}

fn fit_heading(
  observations: &[Observation],
  reference: f64,
  now: WorldTime,
  freshness: f64,
) -> Option<(f64, f64, AngularQuality)> {
  let mut unwrapped = Vec::new();
  let mut previous = None;
  for observation in observations {
    let Some(heading) = observation.heading_rad else {
      continue;
    };
    let value = previous.map_or(heading, |old| old + angle_difference(heading, old));
    previous = Some(value);
    unwrapped.push((observation.time.0 - reference, value, observation.weight));
  }
  if unwrapped.is_empty() {
    return None;
  }
  let sw: f64 = unwrapped.iter().map(|(_, _, weight)| weight).sum();
  let st: f64 = unwrapped.iter().map(|(t, _, weight)| t * weight).sum();
  let sy: f64 = unwrapped
    .iter()
    .map(|(_, value, weight)| value * weight)
    .sum();
  let stt: f64 = unwrapped.iter().map(|(t, _, weight)| t * t * weight).sum();
  let sty: f64 = unwrapped
    .iter()
    .map(|(t, value, weight)| t * value * weight)
    .sum();
  let denominator = sw * stt - st * st;
  let (at_reference, angular_velocity) = if denominator.abs() > 1.0e-12 {
    (
      (sy * stt - st * sty) / denominator,
      (sw * sty - st * sy) / denominator,
    )
  } else {
    (sy / sw.max(f64::EPSILON), 0.0)
  };
  let extrapolate = (now.0 - reference).max(0.0);
  let heading = wrap_degrees((at_reference + angular_velocity * extrapolate).to_degrees());
  let variance_rad = unwrapped
    .iter()
    .map(|(t, value, weight)| weight * (value - (at_reference + angular_velocity * t)).powi(2))
    .sum::<f64>()
    / sw.max(1.0);
  let variance_deg = variance_rad.to_degrees().powi(2).max(0.25);
  let half_width = 1.96 * variance_deg.sqrt();
  let lower = wrap_degrees(heading - half_width);
  let upper = wrap_degrees(heading + half_width);
  let support = 1.0 - (-(unwrapped.len() as f64) / 4.0).exp();
  let confidence = (support * freshness * (-half_width / 20.0).exp()).clamp(0.0, 1.0) as f32;
  Some((
    heading,
    angular_velocity.to_degrees(),
    AngularQuality {
      confidence,
      variance_deg2: variance_deg as f32,
      p95_half_width_deg: half_width as f32,
      p95_lower_deg: lower as f32,
      p95_upper_deg: upper as f32,
      p95_wraps_zero: lower > upper,
    },
  ))
}

fn predicted_quality(
  pos_xx: f64,
  pos_xy: f64,
  pos_yy: f64,
  vel_xx: f64,
  vel_yy: f64,
  horizon: f64,
  confidence: f32,
) -> VectorQuality {
  VectorQuality::from_covariance(
    confidence * (-horizon as f32 / 0.75).exp(),
    pos_xx + horizon * horizon * vel_xx,
    pos_xy,
    pos_yy + horizon * horizon * vel_yy,
  )
}

fn ball_bounce_transition(
  estimate: Option<MotionEstimate>,
  last_time: Option<WorldTime>,
  observation: &Observation,
  walls: &WallModel,
  config: &WorldModelConfig,
) -> Option<BounceTransition> {
  let estimate = estimate?;
  let last_time = last_time?;
  let dt = observation.time.0 - last_time.0;
  if !(0.002..=0.2).contains(&dt) || hypot(estimate.vel[0], estimate.vel[1]) < 300.0 {
    return None;
  }
  let normal_prediction = [
    estimate.pos[0] + estimate.vel[0] * dt,
    estimate.pos[1] + estimate.vel[1] * dt,
  ];
  let normal_error = hypot(
    observation.pos[0] - normal_prediction[0],
    observation.pos[1] - normal_prediction[1],
  );
  if normal_error < config.ball_outlier_gate_mm as f64 {
    return None;
  }

  let mut best = None;
  for axis in 0..2 {
    for sign in [-1.0, 1.0] {
      let side = match (axis, sign > 0.0) {
        (0, true) => 0,
        (1, true) => 1,
        (0, false) => 2,
        _ => 3,
      };
      let wall = walls.coordinates[side].abs();
      let coordinate = estimate.pos[axis];
      let velocity = estimate.vel[axis];
      if velocity * sign <= 0.0 {
        continue;
      }
      let collision_dt = (sign * wall - coordinate) / velocity;
      if !(0.0..=dt).contains(&collision_dt) {
        continue;
      }
      let mut reflected = estimate.vel;
      reflected[axis] = -reflected[axis] * config.wall_restitution as f64;
      let remaining = dt - collision_dt;
      let mut predicted = estimate.pos;
      predicted[0] += estimate.vel[0] * collision_dt + reflected[0] * remaining;
      predicted[1] += estimate.vel[1] * collision_dt + reflected[1] * remaining;
      let error = hypot(
        observation.pos[0] - predicted[0],
        observation.pos[1] - predicted[1],
      );
      let restitution = config.wall_restitution as f64;
      let inferred_wall_coordinate = (observation.pos[axis]
        + restitution * estimate.vel[axis] * dt
        + restitution * estimate.pos[axis])
        / (1.0 + restitution);
      if error < normal_error * 0.65 && best.is_none_or(|(best_error, _)| error < best_error) {
        best = Some((
          error,
          BounceTransition {
            reflected_velocity: reflected,
            side,
            inferred_wall_coordinate,
          },
        ));
      }
    }
  }
  best.map(|(_, transition)| transition)
}

fn cleaned_tracker_packet(
  time: WorldTime,
  frame_number: u32,
  ball: Option<MotionEstimate>,
  robots: &HashMap<(i32, u32), MotionEstimate>,
) -> TrackerWrapperPacket {
  let balls = ball
    .map(|estimate| TrackedBall {
      pos: Vector3 {
        x: estimate.pos[0] as f32 / 1_000.0,
        y: estimate.pos[1] as f32 / 1_000.0,
        z: 0.0,
      },
      vel: Some(Vector3 {
        x: estimate.vel[0] as f32 / 1_000.0,
        y: estimate.vel[1] as f32 / 1_000.0,
        z: 0.0,
      }),
      visibility: Some(estimate.quality.overall_confidence),
    })
    .into_iter()
    .collect();
  let mut tracked_robots: Vec<_> = robots
    .iter()
    .map(|((team, id), estimate)| TrackedRobot {
      robot_id: RobotId {
        id: Some(*id),
        team: Some(*team),
      },
      pos: Vector2 {
        x: estimate.pos[0] as f32 / 1_000.0,
        y: estimate.pos[1] as f32 / 1_000.0,
      },
      orientation: estimate.heading_deg.unwrap_or_default().to_radians() as f32,
      vel: Some(Vector2 {
        x: estimate.vel[0] as f32 / 1_000.0,
        y: estimate.vel[1] as f32 / 1_000.0,
      }),
      vel_angular: estimate
        .angular_vel_deg_s
        .map(|value| value.to_radians() as f32),
      visibility: Some(estimate.quality.overall_confidence),
    })
    .collect();
  tracked_robots.sort_by_key(|robot| {
    (
      robot.robot_id.team.unwrap_or_default(),
      robot.robot_id.id.unwrap_or_default(),
    )
  });
  let kicked_ball = ball.and_then(|estimate| {
    let speed = hypot(estimate.vel[0], estimate.vel[1]);
    if speed < 100.0 {
      return None;
    }
    let decel = (-dot(estimate.accel, estimate.vel) / speed).clamp(150.0, 1_200.0);
    let time_to_stop = speed / decel;
    let distance = speed * time_to_stop - 0.5 * decel * time_to_stop * time_to_stop;
    let direction = [estimate.vel[0] / speed, estimate.vel[1] / speed];
    Some(KickedBall {
      pos: Vector2 {
        x: estimate.pos[0] as f32 / 1_000.0,
        y: estimate.pos[1] as f32 / 1_000.0,
      },
      vel: Vector3 {
        x: estimate.vel[0] as f32 / 1_000.0,
        y: estimate.vel[1] as f32 / 1_000.0,
        z: 0.0,
      },
      start_timestamp: time.0,
      stop_timestamp: Some(time.0 + time_to_stop),
      stop_pos: Some(Vector2 {
        x: (estimate.pos[0] + direction[0] * distance) as f32 / 1_000.0,
        y: (estimate.pos[1] + direction[1] * distance) as f32 / 1_000.0,
      }),
      robot_id: None,
    })
  });
  TrackerWrapperPacket {
    uuid: "crashpilot-world-model".to_string(),
    source_name: Some("CrashPilot filter".to_string()),
    tracked_frame: Some(TrackedFrame {
      frame_number,
      timestamp: time.0,
      balls,
      robots: tracked_robots,
      kicked_ball,
      capabilities: Vec::new(),
    }),
  }
}

fn apply_own_command_prior(
  estimate: &mut MotionEstimate,
  command: &CpCommand,
  track: &MotionTrack,
  config: &WorldModelConfig,
) {
  let desired = if command.speed == Some(0) {
    Some([0.0, 0.0])
  } else {
    match CpTask::try_from(command.task).ok() {
      Some(CpTask::TaskPos | CpTask::TaskDribble | CpTask::TaskPosBall) => {
        let Some(target) = command.pos else { return };
        let delta = [
          target.x as f64 - estimate.pos[0],
          target.y as f64 - estimate.pos[1],
        ];
        let distance = hypot(delta[0], delta[1]);
        let max_speed = command.speed.unwrap_or_default() as f64;
        if distance < 1.0 || max_speed <= 0.0 {
          Some([0.0, 0.0])
        } else {
          let speed = (2.0 * distance).min(max_speed);
          Some([delta[0] / distance * speed, delta[1] / distance * speed])
        }
      }
      _ => None,
    }
  };
  let Some(desired) = desired else { return };
  let dt = track
    .observations
    .iter()
    .rev()
    .take(2)
    .map(|observation| observation.time.0)
    .reduce(|newest, older| newest - older)
    .filter(|dt| *dt > 0.0)
    .unwrap_or(1.0 / 60.0)
    .clamp(0.001, 0.1);
  let accelerating = hypot(desired[0], desired[1]) >= hypot(estimate.vel[0], estimate.vel[1]);
  let limit = if accelerating {
    config.own_robot_max_accel_mm_s2
  } else {
    config.own_robot_max_decel_mm_s2
  } as f64
    * dt;
  let mut change = [desired[0] - estimate.vel[0], desired[1] - estimate.vel[1]];
  clamp_vector(&mut change, limit);
  let trust = (1.0 - estimate.quality.velocity.confidence as f64).clamp(0.05, 0.5);
  estimate.vel[0] += change[0] * trust;
  estimate.vel[1] += change[1] * trust;
}

fn clamp_vector(vector: &mut [f64; 2], maximum: f64) {
  let magnitude = hypot(vector[0], vector[1]);
  if magnitude > maximum && maximum > 0.0 {
    vector[0] *= maximum / magnitude;
    vector[1] *= maximum / magnitude;
  }
}

fn angle_difference(value: f64, reference: f64) -> f64 {
  (value - reference + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
    - std::f64::consts::PI
}

fn wrap_degrees(value: f64) -> f64 {
  value.rem_euclid(360.0)
}

fn hypot(x: f64, y: f64) -> f64 {
  (x * x + y * y).sqrt()
}

fn dot(left: [f64; 2], right: [f64; 2]) -> f64 {
  left[0] * right[0] + left[1] * right[1]
}

#[cfg(test)]
mod tests {
  use super::*;

  fn observation(time: f64, x: f64, y: f64) -> Observation {
    Observation {
      time: WorldTime(time),
      pos: [x, y],
      heading_rad: None,
      weight: 1.0,
      source: "test".to_string(),
    }
  }

  #[test]
  fn logical_time_fit_is_independent_of_wall_clock() {
    let config = WorldModelConfig::default();
    let mut track = MotionTrack::default();
    for index in 0..10 {
      push_observation(
        &mut track,
        observation(index as f64 / 60.0, index as f64 * 20.0, 100.0),
        &config,
      );
    }
    let estimate = fit_track(&mut track, WorldTime(9.0 / 60.0), &config, true, None).unwrap();
    assert!((estimate.vel[0] - 1_200.0).abs() < 1.0);
    assert!(estimate.vel[1].abs() < 1.0);
  }

  #[test]
  fn position_quality_contains_correlated_ellipse() {
    let quality = VectorQuality::from_covariance(0.8, 400.0, 150.0, 100.0);
    assert!(quality.p95_major_radius > quality.p95_minor_radius);
    assert!(quality.p95_angle_rad.abs() > 0.01);
  }

  #[test]
  fn heading_interval_wraps_zero() {
    let observations = [
      Observation {
        heading_rad: Some(358f64.to_radians()),
        ..observation(0.0, 0.0, 0.0)
      },
      Observation {
        heading_rad: Some(1f64.to_radians()),
        ..observation(0.1, 0.0, 0.0)
      },
      Observation {
        heading_rad: Some(3f64.to_radians()),
        ..observation(0.2, 0.0, 0.0)
      },
    ];
    let (heading, _, quality) = fit_heading(&observations, 0.2, WorldTime(0.2), 1.0).unwrap();
    assert!(heading < 10.0 || heading > 350.0);
    assert!(quality.p95_wraps_zero || quality.p95_half_width_deg < 1.0);
  }

  #[test]
  fn wall_bounce_reflects_and_learns_physical_boundary() {
    let config = WorldModelConfig::default();
    let field = FieldSetup::default();
    let walls = WallModel::new(field, config.wall_offset_mm as f64);
    let transition = ball_bounce_transition(
      Some(MotionEstimate {
        pos: [4_650.0, 0.0],
        vel: [4_000.0, 0.0],
        ..Default::default()
      }),
      Some(WorldTime(1.0)),
      &observation(1.1, 4_427.0, 0.0),
      &walls,
      &config,
    )
    .unwrap();
    assert!(transition.reflected_velocity[0] < 0.0);
    assert_eq!(transition.side, 0);
    assert!(transition.inferred_wall_coordinate > 4_400.0);
  }
}
