use tokio::sync::mpsc;
pub(crate) use crate::communication::Event;
use crate::config::Config;
use crate::proto::{Referee, TrackerWrapperPacket};
use crate::ssl_communication::create_multicast_socket::create_multicast_socket;
use crate::ssl_communication::udp_listener::spawn_udp_listener;

pub mod udp_listener;
pub mod create_multicast_socket;

pub async fn get_ssl_data<'a>(cfg: &Config, tx: mpsc::Sender<Event>){
  // Referee
  let ref_socket = create_multicast_socket(cfg.ssl.ssl_gc_ip, cfg.ssl.ssl_gc_port);

  spawn_udp_listener::<Referee>(ref_socket.unwrap(), tx.clone(), Event::Referee);

  // Vision
  let vis_socket = create_multicast_socket(cfg.ssl.ssl_vision_ip, cfg.ssl.ssl_vision_port);

  spawn_udp_listener::<TrackerWrapperPacket>(vis_socket.unwrap(), tx.clone(), Event::SslWrapper);

  // Drop extra sender on stream drop
  drop(tx);
}
