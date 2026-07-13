use crate::world_model::{CleanWorldSnapshot, EstimateQuality, VectorQuality};
use core_dump::proto::{CpGamePhase, CpRobot, Referee, SslWrapperPacket, TrackerWrapperPacket};
use prost::Message;

/// Wire-compatible superset of core_dump's CP_InterfaceWrapper. Keeping this
/// local lets CrashPilot expose world-model diagnostics while the shared
/// core_dump ssl_27 API migration is still in progress.
#[derive(Clone, PartialEq, Message)]
pub struct ExtendedInterfaceWrapper {
  #[prost(message, optional, tag = "1")]
  pub vision_raw: Option<SslWrapperPacket>,
  #[prost(message, optional, tag = "2")]
  pub vision_tracked: Option<TrackerWrapperPacket>,
  #[prost(message, optional, tag = "3")]
  pub gc_data: Option<Referee>,
  #[prost(message, repeated, tag = "4")]
  pub robot_commands: Vec<CpRobot>,
  #[prost(message, optional, tag = "5")]
  pub cp_gamephase: Option<CpGamePhase>,
  #[prost(message, repeated, tag = "6")]
  pub vision_raw_sources: Vec<SslWrapperPacket>,
  #[prost(message, repeated, tag = "7")]
  pub vision_tracked_sources: Vec<TrackerWrapperPacket>,
  #[prost(message, optional, tag = "8")]
  pub vision_filtered: Option<TrackerWrapperPacket>,
  #[prost(message, optional, tag = "9")]
  pub world_model_quality: Option<WorldModelQuality>,
}

#[derive(Clone, PartialEq, Message)]
pub struct WorldModelQuality {
  #[prost(double, required, tag = "1")]
  pub timestamp: f64,
  #[prost(message, optional, tag = "2")]
  pub ball: Option<EntityQuality>,
  #[prost(message, repeated, tag = "3")]
  pub robots: Vec<EntityQuality>,
}

#[derive(Clone, PartialEq, Message)]
pub struct EntityQuality {
  #[prost(uint32, optional, tag = "1")]
  pub robot_id: Option<u32>,
  #[prost(int32, optional, tag = "2")]
  pub team: Option<i32>,
  #[prost(bool, required, tag = "3")]
  pub valid: bool,
  #[prost(float, required, tag = "4")]
  pub overall_confidence: f32,
  #[prost(float, required, tag = "5")]
  pub measurement_age_s: f32,
  #[prost(message, optional, tag = "6")]
  pub position: Option<VectorQualityMessage>,
  #[prost(message, optional, tag = "7")]
  pub velocity: Option<VectorQualityMessage>,
  #[prost(message, optional, tag = "8")]
  pub heading: Option<AngularQualityMessage>,
  #[prost(float, tag = "9")]
  pub trajectory_confidence: f32,
  #[prost(message, optional, tag = "10")]
  pub prediction_100ms: Option<VectorQualityMessage>,
  #[prost(message, optional, tag = "11")]
  pub prediction_250ms: Option<VectorQualityMessage>,
  #[prost(message, optional, tag = "12")]
  pub prediction_500ms: Option<VectorQualityMessage>,
}

#[derive(Clone, PartialEq, Message)]
pub struct VectorQualityMessage {
  #[prost(float, required, tag = "1")]
  pub confidence: f32,
  #[prost(float, required, tag = "2")]
  pub covariance_xx: f32,
  #[prost(float, required, tag = "3")]
  pub covariance_xy: f32,
  #[prost(float, required, tag = "4")]
  pub covariance_yy: f32,
  #[prost(float, required, tag = "5")]
  pub p95_major_radius: f32,
  #[prost(float, required, tag = "6")]
  pub p95_minor_radius: f32,
  #[prost(float, required, tag = "7")]
  pub p95_angle_rad: f32,
  #[prost(float, required, tag = "8")]
  pub p95_x_min: f32,
  #[prost(float, required, tag = "9")]
  pub p95_x_max: f32,
  #[prost(float, required, tag = "10")]
  pub p95_y_min: f32,
  #[prost(float, required, tag = "11")]
  pub p95_y_max: f32,
}

#[derive(Clone, PartialEq, Message)]
pub struct AngularQualityMessage {
  #[prost(float, required, tag = "1")]
  pub confidence: f32,
  #[prost(float, required, tag = "2")]
  pub variance_deg2: f32,
  #[prost(float, required, tag = "3")]
  pub p95_half_width_deg: f32,
  #[prost(float, required, tag = "4")]
  pub p95_lower_deg: f32,
  #[prost(float, required, tag = "5")]
  pub p95_upper_deg: f32,
  #[prost(bool, required, tag = "6")]
  pub p95_wraps_zero: bool,
}

impl WorldModelQuality {
  pub fn from_snapshot(snapshot: &CleanWorldSnapshot) -> Self {
    let ball = snapshot
      .ball
      .map(|estimate| EntityQuality::from_estimate(None, None, estimate.quality));
    let mut robots: Vec<_> = snapshot
      .robots
      .iter()
      .map(|((team, id), estimate)| {
        EntityQuality::from_estimate(Some(*id), Some(*team), estimate.quality)
      })
      .collect();
    robots.sort_by_key(|quality| {
      (
        quality.team.unwrap_or_default(),
        quality.robot_id.unwrap_or_default(),
      )
    });
    Self {
      timestamp: snapshot.timestamp.0,
      ball,
      robots,
    }
  }
}

impl EntityQuality {
  fn from_estimate(robot_id: Option<u32>, team: Option<i32>, quality: EstimateQuality) -> Self {
    Self {
      robot_id,
      team,
      valid: quality.valid,
      overall_confidence: quality.overall_confidence,
      measurement_age_s: quality.measurement_age_s,
      position: Some(quality.position.into()),
      velocity: Some(quality.velocity.into()),
      heading: quality.heading.map(|heading| AngularQualityMessage {
        confidence: heading.confidence,
        variance_deg2: heading.variance_deg2,
        p95_half_width_deg: heading.p95_half_width_deg,
        p95_lower_deg: heading.p95_lower_deg,
        p95_upper_deg: heading.p95_upper_deg,
        p95_wraps_zero: heading.p95_wraps_zero,
      }),
      trajectory_confidence: quality.trajectory_confidence,
      prediction_100ms: Some(quality.prediction_100ms.into()),
      prediction_250ms: Some(quality.prediction_250ms.into()),
      prediction_500ms: Some(quality.prediction_500ms.into()),
    }
  }
}

impl From<VectorQuality> for VectorQualityMessage {
  fn from(quality: VectorQuality) -> Self {
    Self {
      confidence: quality.confidence,
      covariance_xx: quality.covariance_xx,
      covariance_xy: quality.covariance_xy,
      covariance_yy: quality.covariance_yy,
      p95_major_radius: quality.p95_major_radius,
      p95_minor_radius: quality.p95_minor_radius,
      p95_angle_rad: quality.p95_angle_rad,
      p95_x_min: quality.p95_x_min,
      p95_x_max: quality.p95_x_max,
      p95_y_min: quality.p95_y_min,
      p95_y_max: quality.p95_y_max,
    }
  }
}
