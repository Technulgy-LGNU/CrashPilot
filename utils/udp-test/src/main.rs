use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::net::UdpSocket;
use prost::Message;
use prost_types::Timestamp;
use std::fs::OpenOptions;
use std::io::Write;

mod proto;

#[tokio::main]
async fn main() {
  // Open (or create) CSV file in append mode
  let mut file = OpenOptions::new()
    .append(true)
    .create(true)
    .open("network_delays.csv")
    .expect("Failed to open CSV file");

  // Write header if the file is empty
  if file.metadata().unwrap().len() == 0 {
    writeln!(file, "timestamp_ms,delay_ms").expect("Failed to write CSV header");
  }

  let socket = UdpSocket::bind("0.0.0.0:1024").await.expect("failed to bind socket");
  println!("Listening on UDP port 1024...");

  let mut buf = vec![0u8; 1024];

  loop {
    let (size, src) = socket.recv_from(&mut buf).await.expect("failed to receive");
    println!("Received {} bytes from {}", size, src);

    match proto::CpRobot::decode(&buf[..size]) {
      Ok(msg) => {
        println!("Command: {:?}", msg.cmd.state);
      }
      Err(e) => eprintln!("Failed to decode protobuf: {}", e),
    }
  }
}

fn timestamp_to_system_time(ts: &Timestamp) -> SystemTime {
  let duration = Duration::new(ts.seconds as u64, ts.nanos as u32);
  UNIX_EPOCH + duration
}
