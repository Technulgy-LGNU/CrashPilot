use prost::Message;
use tokio::net::UdpSocket;

mod proto;

#[tokio::main]
async fn main() {
  let socket = UdpSocket::bind("0.0.0.0:1024").await.expect("failed to bind socket");
  println!("Listening on UDP port 1024...");

  let mut buf = vec![0u8; 1024];

  loop {
    let (size, src) = socket.recv_from(&mut buf).await.expect("failed to receive");
    println!("Received {} bytes from {}", size, src);

    match proto::CpRobot::decode(&buf[..size]) {
      Ok(msg) => {
        println!("Command: {:?}", msg);
      }
      Err(e) => eprintln!("Failed to decode protobuf: {}", e),
    }
  }
}

