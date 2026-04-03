use crate::ssl_communication::Event;

mod ssl_communication;
mod proto;
mod utils;
mod robot_communication;
mod config;

#[tokio::main]
async fn main() {
  // Get config
  let config = match config::load_or_create_config("config.toml") {
    Ok(config) => config,
    Err(e) => panic!("{}", e),
  };

  let mut rx = ssl_communication::get_ssl_data(config).await;

  while let Some(event) = rx.recv().await {
    match event {
      Event::Referee(_) => {}
      Event::SslWrapper(_) => {}
    }
  }
}
