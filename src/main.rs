use crate::communication::communication_receiver;
use crate::communication::robot_sender::{NetworkSender, RobotSender};
use crate::helpers::robot_data::create_robot_data;
use crate::proto::{CpInterfaceWrapper, CpRobot, InterfaceCommandCp, RobotCp};
use prost::Message;
use std::collections::HashMap;
use std::fs;
use std::io::ErrorKind;
use std::os::unix::fs::PermissionsExt;
use std::process::Command;
use tokio::time::{Duration, MissedTickBehavior, interval};

mod communication;
mod config;
mod game_logic;
mod helpers;
mod proto;

// Embed frontend (crashpilot-interface) binary
static GO_BINARY: &[u8] = include_bytes!("../crashpilot-interface");

struct RobotData {
  pub msg: CpRobot,
  pub feedback: RobotCp,
}

#[tokio::main]
async fn main() {
  tokio::spawn(async move {
    let path = "./crashpilot-interface";

    fs::write(path, GO_BINARY).expect("Failed to write binary file");

    let mut perms = fs::metadata(path)
      .expect("Failed to read metadata")
      .permissions();

    perms.set_mode(0o755);

    fs::set_permissions(path, perms).expect("Failed to set executable permissions");

    Command::new(path).spawn().expect("Failed to spawn binary");
  });

  // Get config
  let config = match config::load_or_create_config("config.toml") {
    Ok(config) => config,
    Err(e) => panic!("{}", e),
  };

  // Receiver for the communication
  let (rx, ws_out)= match communication_receiver(&config).await {
    Ok(comm) => (comm.events, comm.ws_out),
    Err(e) => panic!("{}", e),
  };

  // UDPSocket for robot communication
  let robot_socket = match tokio::net::UdpSocket::bind(format!(
    "{}:{}",
    config.server.robot_socket_host, config.server.robot_socket_port
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
  };

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
  let mut robots_ws_data: HashMap<u32, proto::CpCommand> = HashMap::new();
  for robot in config.robots.iter() {
    robots_ws_data.insert(*robot.0, Default::default());
  }

  println!("Starting robots...");
  // Data packets
  let mut vis_raw: proto::SslWrapperPacket = Default::default();
  let mut vis_tracked: proto::TrackerWrapperPacket = Default::default();
  let mut interface_command: InterfaceCommandCp = Default::default();
  let mut referee: proto::Referee = Default::default();

  // Sending should not depend on receiving new packets: when vision/GC packets pause,
  // we still want to keep sending the latest known command/state to the robots.
  // Also, waiting on an interval prevents busy-spinning on `rx.lock()`.
  let mut tick = interval(Duration::from_millis(2)); // ~480 Hz
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
      robots
        .iter_mut()
        .find(|(_, data)| data.msg.robot_id == packet.robot_id)
        .map(|(_, data)| data.feedback = packet);
    }

    robots = create_robot_data(
      robots,
      packet_id,
      &vis_tracked,
      &vis_raw,
      &referee,
      &interface_command,
      &robots_ws_data,
    );

    // Actual game logic is going to happen here
    // First checks, on game state, and coordinating robots for that
    // Checks if one of multiple predetermine strategies apply
    //  - Goalie has Ball -> Chips automatically to the furthest own robot -> This robot should get the receive command
    // Still WIP, teamfaabs_ssl_robot_code is still in W.I.P., but nearing its completion

    // Send the data to the robots
    let network_sender: NetworkSender = NetworkSender {
      socket: &robot_socket,
      data: &robots,
    };
    let send_report = network_sender.send_to_all_robots(&config).await;
    if !send_report.failed.is_empty() {
      eprintln!(
        "Robot send: {} ok, {} failed",
        send_report.sent,
        send_report.failed.len()
      );
      for failure in send_report.failed {
        eprintln!("  robot {}: {:#}", failure.robot_id, failure.error);
      }
    }

    // Broadcast the latest state to all websocket clients (CP -> interface)
    // Note: if no clients are connected, send() returns an error; that's fine.
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

    // So the next packet has a higher id
    packet_id += 1;
  }
}
