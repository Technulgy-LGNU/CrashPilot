use crate::ssl::Event;
use crate::utils::write_to_file::write_to_file;

mod network;
mod proto;
mod ssl;
mod utils;

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
        if wrapper.geometry.is_some() {
          tokio::spawn(write_to_file("", wrapper.clone()));
        }
        println!("SslWrapper: {:?}", wrapper);
      }
    }
  }
}
