use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::net::UdpSocket;
use prost::Message;
use prost_types::Timestamp;

mod proto;

#[tokio::main]
async fn main() {
  let socket = match UdpSocket::bind("0.0.0.0:1024").await {
    Ok(socket) => socket,
    Err(e) => panic!("failed to bind socket: {:?}", e),
  };
  println!("Listening on UDP port 1024...");

  let mut buf = vec![0u8; 1024]; // Buffer to hold incoming data

  loop {
    let (size, src) = match socket.recv_from(&mut buf).await {
      Ok((size, src)) => (size, src),
      Err(e) => panic!("Error receiving from socket; err={:?}", e),
    };

    println!("\nReceived {} bytes from {}", size, src);

    match proto::CpRobot::decode(&buf[..size]) {
      Ok(msg) => {
        // Print the network delay
        let timestamp = timestamp_to_system_time(&msg.timestamp);
        let now = SystemTime::now();
        let delay = now.duration_since(timestamp).unwrap();
        println!("Network delay: {:?}", delay);
      }
      Err(e) => {
        eprintln!("Failed to decode protobuf: {}", e);
      }
    }
  }
}

fn timestamp_to_system_time(ts: &Timestamp) -> SystemTime {
  let duration = Duration::new(ts.seconds as u64, ts.nanos as u32);
  UNIX_EPOCH + duration
}
