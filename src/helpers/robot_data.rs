use crate::RobotData;
use crate::helpers::as_cp_vec2;
use crate::helpers::ball_helper::{VisionBalls, convert_ball};
use core_dump::proto::{
  CpTrackedRobot, InterfaceCommandCp, SslDetectionBall, SslWrapperPacket, TrackerWrapperPacket,
};
use prost_types::Timestamp;
use std::collections::HashMap;
use std::time::SystemTime;

#[inline]
pub fn create_robot_data(
  mut robots: HashMap<u32, RobotData>,
  packet_id: u32,
  vis_tracked: &TrackerWrapperPacket,
  vis_raw: &SslWrapperPacket,
  interface_command: &InterfaceCommandCp,
) -> HashMap<u32, RobotData> {
  // Create data for each robot
  for robot in robots.values_mut() {
    // Basic data
    robot.msg.packet_id = packet_id;
    robot.msg.timestamp = Timestamp::from(SystemTime::now());

    // Tracked frame, if not empty
    // Robot Position Data
    if let Some(frame) = vis_tracked.tracked_frame.clone() {
      // Robot
      // Clear robots already in array
      robot.msg.robots_yellow = vec![];
      robot.msg.robots_blue = vec![];
      for robot_tracked in frame.robots {
        let robot_vis: CpTrackedRobot = CpTrackedRobot {
          robot_id: robot_tracked.robot_id.id.unwrap_or_default(),
          pos: as_cp_vec2(robot_tracked.pos),
          orientation: robot_tracked.orientation.to_degrees() as i32,
          vel: Option::from(as_cp_vec2(robot_tracked.vel.unwrap_or_default())),
          visibility: (robot_tracked.visibility.unwrap_or_default() * 100f32) as u32,
        };

        match robot_tracked.robot_id.team {
          // Yellow robots
          // Check if this yellow robot already exists
          Some(1)
            if !robot
              .msg
              .robots_yellow
              .iter()
              .any(|robot| robot.robot_id == robot_vis.robot_id) =>
          {
            robot.msg.robots_yellow.push(robot_vis)
          }
          // Blue Robots
          // Check if this blue robot already exists
          Some(2)
            if !robot
              .msg
              .robots_blue
              .iter()
              .any(|robot| robot.robot_id == robot_vis.robot_id) =>
          {
            robot.msg.robots_blue.push(robot_vis);
          }
          _ => (),
        }
      }

      // Raw or Tracked vision can be used here
      // Tracked vision is superior and will be used by default
      // Ball
      if !interface_command.manual.ball_tracked {
        let vis_raw_balls: Vec<SslDetectionBall> = match vis_raw.detection.clone() {
          Some(frame) => frame.balls,
          None => vec![],
        };
        robot.msg.ball = convert_ball(VisionBalls::Raw(vis_raw_balls), interface_command);
      } else {
        robot.msg.ball = convert_ball(VisionBalls::Tracked(frame.balls), interface_command);
      }
    };
  }

  robots
}
