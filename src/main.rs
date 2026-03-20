use tokio::net::UnixStream;
use crate::ssl::Event;

mod proto;
mod ssl;
mod network;

#[tokio::main]
async fn main() {
  let mut rx = ssl::get_ssl_data().await;

  // Unix socket
  let stream = UnixStream::connect("/tmp/rust_to_cpp.sock").await.expect("Failed to connect");
  println!("Connected to {:?}", stream);

  // Event loop
  while let Some(event) = rx.recv().await {
    match event {
      Event::Referee(referee) => {
        println!("Referee");
        let message = format!("{:?}", referee.command);
        stream.try_write(message.as_bytes()).unwrap();
      }
      Event::SslWrapper(wrapper) => {
        println!("SslWrapper");
      }
    }
  }
}


