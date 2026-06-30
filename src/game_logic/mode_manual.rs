use crate::RobotData;
use core_dump::proto::CpCommand;
use std::collections::HashMap;

#[inline]
pub fn mode_manual(
  robot_data: &mut HashMap<u32, RobotData>,
  robots_ws_data: &HashMap<u32, CpCommand>,
  gc_enabled: bool,
  referee_command: i32,
) {
  for robot in robot_data.values_mut() {
    let cmd: CpCommand = match robots_ws_data.get(&robot.msg.robot_id) {
      None => Default::default(),
      Some(cmd) => cmd.clone(),
    };
    robot.msg.cmd = cmd;

    if gc_enabled && (referee_command == 0 || referee_command == 1) {
      robot.msg.cmd.state = referee_command;
    }
  }
}
