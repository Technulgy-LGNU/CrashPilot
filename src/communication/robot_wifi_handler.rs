use crate::communication::multicast_handler::create_multicast_socket;
use crate::communication::{EventShare, RobotsOut};
use crate::config;
use core_dump::protocol::robot_command_frame::RobotCommandFrame;
use core_dump::protocol::robot_debug_wire::RobotDebugWire;
use core_dump::protocol::robot_telemetry_wire::RobotTelemetryWire;
use std::io;
use std::net::SocketAddrV4;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::time::{MissedTickBehavior, interval, sleep};

const ROBOT_COMMAND_INTERVAL: Duration = Duration::from_millis(1);

/// Sends and receives the robot data over multicast
pub fn robot_wifi_handler(cfg: &config::Config, tx: EventShare, robots_out: RobotsOut) {
  let multicast_host = cfg.cp_config.cp_multicast_host;
  let multicast_port = cfg.cp_config.cp_multicast_port;
  let multicast_interface = cfg.cp_config.cp_multicast_interface;

  tokio::spawn(async move {
    // Auto reconnect loop
    let mut reconnect_delay = Duration::from_secs(1);
    let max_reconnect_delay = Duration::from_secs(30);

    loop {
      let tokio_socket =
        match create_multicast_socket(multicast_host, multicast_port, multicast_interface) {
          Ok(socket) => {
            reconnect_delay = Duration::from_secs(1);
            socket
          }
          Err(err) => {
            eprintln!(
              "Failed to create multicast socket for {}:{} on interface {}: {}. Retrying in {:?}",
              multicast_host, multicast_port, multicast_interface, err, reconnect_delay
            );

            sleep(reconnect_delay).await;
            reconnect_delay = (reconnect_delay * 2).min(max_reconnect_delay);
            continue;
          }
        };

      let socket = Arc::new(tokio_socket);
      let receive_socket = socket.clone();
      let send_target = SocketAddrV4::new(multicast_host, multicast_port);

      let result = tokio::select! {
        result = receive_robot_data(receive_socket, tx.clone()) => result,
        result = send_robot_commands(socket, send_target, robots_out.clone()) => result,
      };

      eprintln!(
        "Robot multicast error on {}:{} via {}: {}. Reconnecting in {:?}",
        multicast_host,
        multicast_port,
        multicast_interface,
        result.unwrap_err(),
        reconnect_delay
      );
      sleep(reconnect_delay).await;
      reconnect_delay = (reconnect_delay * 2).min(max_reconnect_delay);
    }
  });
}

async fn receive_robot_data(socket: Arc<UdpSocket>, tx: EventShare) -> io::Result<()> {
  let mut buf = [0u8; 65_536];

  loop {
    let (size, _addr) = match socket.recv_from(&mut buf).await {
      Ok(packet) => packet,
      Err(err) if err.kind() == io::ErrorKind::Interrupted => continue,
      Err(err) => return Err(err),
    };

    match size {
      RobotTelemetryWire::ENCODED_LEN => {
        match RobotTelemetryWire::decode(buf[..size].try_into().unwrap()) {
          Ok(data) => {
            let robot_id = data.robot_id as usize;
            let mut events = tx.write().await;
            if let Some(robot) = events.robot_data.get_mut(robot_id) {
              robot.robot_telemetry = Some(data);
            } else {
              eprintln!("Received telemetry data for unknown robot_id: {}", robot_id);
            }
          }
          Err(err) => eprintln!("Failed to decode robot telemetry data: {}", err),
        }
      }
      RobotDebugWire::ENCODED_LEN => {
        match RobotDebugWire::decode(buf[..size].try_into().unwrap()) {
          Ok(data) => {
            let robot_id = data.robot_id as usize;
            let mut events = tx.write().await;
            if let Some(robot) = events.robot_data.get_mut(robot_id) {
              robot.robot_debug = Some(data);
            } else {
              eprintln!("Received debug data for unknown robot_id: {}", robot_id);
            }
          }
          Err(err) => eprintln!("Failed to decode robot debug data: {}", err),
        }
      }
      // The local socket may receive its own multicast command packet.
      RobotCommandFrame::ENCODED_LEN => {}
      _ => eprintln!("Received unexpected data size: {}", size),
    }
  }
}

async fn send_robot_commands(
  socket: Arc<UdpSocket>,
  target: SocketAddrV4,
  robots_out: RobotsOut,
) -> io::Result<()> {
  let mut ticker = interval(ROBOT_COMMAND_INTERVAL);
  ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

  loop {
    ticker.tick().await;

    // Copy the latest frame so the read lock is released before network I/O.
    let frame = *robots_out.read().await;
    let Some(frame) = frame else {
      continue;
    };

    socket.send_to(&frame.encode(), target).await?;
  }
}
