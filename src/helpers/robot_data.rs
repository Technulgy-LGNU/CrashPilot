use crate::RobotData;
use crate::helpers::as_cp_vec2;
use crate::helpers::ball_helper::{VisionBalls, convert_ball};
use crate::proto::{
  CpCommand, CpTrackedRobot, InterfaceCommandCp, Referee, SslDetectionBall, SslWrapperPacket,
  TrackerWrapperPacket,
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
  referee: &Referee,
  interface_command: &InterfaceCommandCp,
  robots_ws_data: &HashMap<u32, CpCommand>,
) -> HashMap<u32, RobotData> {
  // Create data for each robot
  for robot in robots.values_mut() {
    // Basic data
    robot.msg.packet_id = packet_id;
    robot.msg.timestamp = Timestamp::from(SystemTime::now());

    // Tracked frame, if not empty
    // Robot Position Data
    match vis_tracked.tracked_frame.clone() {
      Some(frame) => {
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
            Some(1) => {
              // Check if this yellow robot already exists
              if !robot
                .msg
                .robots_yellow
                .iter()
                .any(|robot| robot.robot_id == robot_vis.robot_id)
              {
                robot.msg.robots_yellow.push(robot_vis);
              }
            }
            // Blue Robots
            Some(2) => {
              // Check if this blue robot already exists
              if !robot
                .msg
                .robots_blue
                .iter()
                .any(|robot| robot.robot_id == robot_vis.robot_id)
              {
                robot.msg.robots_yellow.push(robot_vis);
              }
            }
            _ => (),
          }
        }

        // Raw or Tracked vision can be used here
        // Tracked vision is superior and will be used by default
        // Ball
        if !interface_command.ball_tracked {
          let vis_raw_balls: Vec<SslDetectionBall> = match vis_raw.detection.clone() {
            Some(frame) => frame.balls,
            None => vec![],
          };
          robot.msg.ball = convert_ball(VisionBalls::Raw(vis_raw_balls), interface_command.clone());
        } else {
          robot.msg.ball =
            convert_ball(VisionBalls::Tracked(frame.balls), interface_command.clone());
        }
      }
      None => (),
    };

    // Commands
    // Check for the referee command and overwrite cp commands
    //
    // HALT Command, all robots stop
    if referee.command == 0 && interface_command.gc_data {
      robot.msg.cmd = match robots_ws_data.get(&robot.msg.robot_id) {
        Some(cmd) => *cmd,
        None => Default::default(),
      };
      robot.msg.cmd.state = 0;

    // STOP Command, all robots are only allowed to move with a max velocity of 1.5m/s and should avoid the ball with a clearance of 0.5m
    } else if referee.command == 1 && interface_command.gc_data {
      robot.msg.cmd = match robots_ws_data.get(&robot.msg.robot_id) {
        Some(cmd) => *cmd,
        None => Default::default(),
      };
      robot.msg.cmd.state = 1;

    // Send the last command received by the interface
    } else {
      robot.msg.cmd = match robots_ws_data.get(&robot.msg.robot_id) {
        Some(cmd) => *cmd,
        None => Default::default(),
      };
    }
  }

  robots
}
