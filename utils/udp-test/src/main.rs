use tokio::net::UdpSocket;
use prost::Message;

mod proto;

#[tokio::main]
async fn main() {
  let socket = match UdpSocket::bind("0.0.0.0:1024").await {
    Ok(socket) => socket,
    Err(e) => panic!("failed to bind socket: {:?}", e),
  };
  println!("Listening on UDP port 1024...");

  let mut buf = Vec::new();

  loop {
    let (size, src) = match socket.recv_from(&mut buf).await {
      Ok((size, src)) => (size, src),
      Err(e) => panic!("Error receiving from socket; err={:?}", e),
    };

    println!("\nReceived {} bytes from {}", size, src);

    match proto::CpTracked::decode(&buf[..size]) {
      Ok(msg) => {
        println!("Got message: {:?}", msg);
      }
      Err(e) => {
        eprintln!("Failed to decode protobuf: {}", e);
      }
    }
  }
}
