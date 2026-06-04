use crate::communication::communication_receiver;
#[cfg(feature = "loki")]
use crate::communication::loki::spawn_loki_publisher;
use crate::communication::robot_sender::{NetworkSender, RobotSender};
use crate::game_logic::{BallData, GameState, Robot, game_logic};
use crate::helpers::robot_data::create_robot_data;
use crate::metrics::PrometheusMetrics;
use core_dump::proto::{
  CpCommand, CpInterfaceWrapper, CpRobot, InterfaceCommandCp, Referee, RobotCp, SslWrapperPacket,
  TrackerWrapperPacket,
};
use prost::Message;
use std::collections::{HashMap, HashSet};
#[cfg(feature = "interface")]
use std::fs;
use std::io::ErrorKind;
#[cfg(feature = "interface")]
use std::os::unix::fs::PermissionsExt;
#[cfg(feature = "interface")]
use std::process::Command;
use tokio::time::{Duration, MissedTickBehavior, interval};

mod communication;
mod config;
mod game_logic;
mod helpers;
mod metrics;

// Embed frontend (crashpilot-interface) binary
#[cfg(feature = "interface")]
static GO_BINARY: &[u8] = include_bytes!("../crashpilot-interface");

struct RobotData {
  pub msg: CpRobot,
  pub feedback: RobotCp,
}

#[tokio::main]
async fn main() {
  #[cfg(feature = "interface")]
  spawn_interface();
  // Get config
  let config = match config::load_or_create_config("config.toml") {
    Ok(config) => config,
    Err(e) => panic!("{}", e),
  };

  // Prometheus metrics endpoint and shared per-robot registry.
  let metrics: PrometheusMetrics = match metrics::spawn_prometheus_server(&config).await {
    Ok(metrics) => metrics,
    Err(e) => panic!("{}", e),
  };
  for robot_id in config.robots.keys().copied() {
    metrics.register_robot(robot_id).await;
  }

  #[cfg(feature = "loki")]
  let loki = spawn_loki_publisher(&config);

  // Receiver for the communication
  let (rx, ws_out) = match communication_receiver(&config).await {
    Ok(comm) => (comm.events, comm.ws_out),
    Err(e) => panic!("{}", e),
  };

  // UDPSocket for robot communication
  let robot_socket = spawn_robot_socket(&config).await;

  // Robots Hashmap
  let mut packet_id = 0;
  let mut robots: HashMap<u32, RobotData> = HashMap::new();
  for robot in config.robots.iter() {
    robots.insert(
      *robot.0,
      RobotData {
        msg: CpRobot {
          robot_id: *robot.0,
          timestamp: Default::default(),
          packet_id,
          ball: Default::default(),
          robots_yellow: vec![],
          robots_blue: vec![],
          cmd: Default::default(),
        },
        feedback: Default::default(),
      },
    );
  }
  // Initialize the hashmap for the websocket data, which will be used to store the last command received for each robot
  let mut robots_ws_data: HashMap<u32, CpCommand> = HashMap::new();
  for robot in config.robots.iter() {
    robots_ws_data.insert(*robot.0, Default::default());
  }

  println!("Starting robots...");
  // Data packets
  let mut vis_raw: SslWrapperPacket = Default::default();
  let mut vis_tracked: TrackerWrapperPacket = Default::default();
  let mut interface_command: InterfaceCommandCp = Default::default();
  let mut referee: Referee = Default::default();
  // Other Vars
  let mut state: GameState = Default::default();

  // Sending should not depend on receiving new packets: when vision/GC packets pause,
  // we still want to keep sending the latest known command/state to the robots.
  // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
  let mut tick = interval(Duration::from_millis(2)); // ~500 Hz
  tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

  loop {
    tick.tick().await;

    // Drain the *latest* events from the shared state. We handle all types each tick,
    // not just one, so state stays fresh.
    let (raw, tracked, ws, gc, rf) = {
      let mut lock = rx.lock().await;
      (
        lock.0.take(),
        lock.1.take(),
        lock.2.take(),
        lock.3.take(),
        lock.4.take(),
      )
    };

    if let Some(packet) = raw {
      vis_raw = packet;
    }
    if let Some(packet) = tracked {
      metrics.record_tracked_frame(&packet).await;
      vis_tracked = packet;
    }
    if let Some(packet) = ws {
      println!("{:?}", packet);
      for robot_command in packet.robot_commands {
        robots_ws_data.insert(robot_command.robot_id, robot_command.command);
      }
      interface_command = packet.interface_command;
    }
    if let Some(packet) = gc {
      referee = packet;
    }
    if let Some(packet) = rf {
      if let Some((_, data)) = robots
        .iter_mut()
        .find(|(_, data)| data.msg.robot_id == packet.robot_id)
      {
        data.feedback = packet;
      }
      metrics.record_robot_feedback(packet).await;
    }

    robots = create_robot_data(
      robots,
      packet_id,
      &vis_tracked,
      &vis_raw,
      &interface_command,
    );

    // Actual game logic is going to happen here
    // First checks, on game state, and coordinating robots for that
    // Checks if one of multiple predetermine strategies apply
    //  - Goalie has Ball -> Chips automatically to the furthest own robot -> This robot should get the receive command
    // Still WIP, teamfaabs_ssl_robot_code is still in W.I.P., but nearing its completion
    robots = game_logic(
      &config,
      robots,
      &mut state,
      Robot::new_from_tracked(&vis_tracked),
      BallData::new(),
      &referee,
      &interface_command,
      &robots_ws_data
    )
    .await;

    // Send the data to the robots
    robot_sender(
      &config,
      &robot_socket,
      &robots,
      &metrics,
      #[cfg(feature = "loki")]
      &loki,
    )
    .await;

    // Websocket sender
    websocket_sender(&vis_raw, &vis_tracked, &referee, &robots, &ws_out).await;

    // So the next packet has a higher id
    packet_id += 1;
  }
}

/// Starts the Crashpilot interface, has to be in the repository as a compiled binary
#[cfg(feature = "interface")]
fn spawn_interface() {
  tokio::spawn(async move {
    let path = "./crashpilot-interface";

    fs::write(path, GO_BINARY).expect("Failed to write binary file");

    let mut perms = fs::metadata(path)
      .expect("Failed to read metadata")
      .permissions();

    perms.set_mode(0o755);

    fs::set_permissions(path, perms).expect("Failed to set executable permissions");

    Command::new(path)
      .spawn()
      .expect("Failed to spawn binary")
      .wait()
      .expect("Failed to wait on binary");
  });
}

/// Spawns the socket for the robot sender
#[inline]
async fn spawn_robot_socket(cfg: &config::Config) -> tokio::net::UdpSocket {
  match tokio::net::UdpSocket::bind(format!(
    "{}:{}",
    cfg.server.robot_socket_host, cfg.server.robot_socket_port
  ))
  .await
  {
    Ok(socket) => socket,
    Err(e) => match e.kind() {
      ErrorKind::AddrNotAvailable => {
        panic!(
          "Failed to bind UDP socket: Address not available. Please check if the IP address and port are correct and not in use."
        );
      }
      _ => panic!("Failed to bind UDP socket: {}", e),
    },
  }
}

/// Sends the latest data to all robots
#[inline]
async fn robot_sender(
  cfg: &config::Config,
  robot_socket: &tokio::net::UdpSocket,
  robots: &HashMap<u32, RobotData>,
  metrics: &PrometheusMetrics,
  #[cfg(feature = "loki")] loki: &Option<communication::loki::LokiPublisher>,
) {
  let network_sender: NetworkSender = NetworkSender {
    socket: robot_socket,
    data: robots,
    #[cfg(feature = "loki")]
    loki: Some(loki.clone()),
  };
  let send_report = network_sender.send_to_all_robots(cfg).await;
  if !send_report.failed.is_empty() {
    eprintln!(
      "Robot send: {} ok, {} failed",
      send_report.sent,
      send_report.failed.len()
    );
    for failure in &send_report.failed {
      eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
    }
  }

  let failed_robot_ids: HashSet<u32> = send_report
    .failed
    .iter()
    .map(|failure| failure.robot_id)
    .collect();
  for robot_id in robots.keys().copied() {
    metrics
      .record_send_result(robot_id, !failed_robot_ids.contains(&robot_id))
      .await;
  }
}

/// Broadcast the latest state to all websocket clients (CP -> interface)
/// Note: if no clients are connected, send() returns an error; that's fine.
#[inline]
async fn websocket_sender(
  vis_raw: &SslWrapperPacket,
  vis_tracked: &TrackerWrapperPacket,
  referee: &Referee,
  robots: &HashMap<u32, RobotData>,
  ws_out: &communication::WebsocketOut,
) {
  let ws_packet = CpInterfaceWrapper {
    vision_raw: Some(vis_raw.clone()),
    vision_tracked: Some(vis_tracked.clone()),
    gc_data: if referee.packet_timestamp != 0 {
      Some(referee.clone())
    } else {
      None
    },
    robot_commands: robots.values().map(|robot| robot.msg.clone()).collect(),
  };
  let mut buf = Vec::with_capacity(ws_packet.encoded_len());
  if let Err(e) = ws_packet.encode(&mut buf) {
    eprintln!("Failed to encode websocket packet: {}", e);
  } else {
    ws_out.publish(buf.into()).await;
  }
}
