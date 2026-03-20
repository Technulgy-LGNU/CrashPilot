use crate::ssl::Event;

mod proto;
mod ssl;
mod network;

#[tokio::main]
async fn main() {
  let mut rx = ssl::get_ssl_data().await;

  // Event loop
  while let Some(event) = rx.recv().await {
    match event {
      Event::Referee(referee) => {
        println!("Referee: {:?}", referee);
      }
      Event::SslWrapper(wrapper) => {
        println!("SslWrapper: {:?}", wrapper);
      }
    }
  }
}


