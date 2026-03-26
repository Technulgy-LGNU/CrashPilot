use crate::network::create_multicast_socket::create_multicast_socket;
use crate::network::udp_listener::spawn_udp_listener;
use crate::proto::{Referee, SslWrapperPacket};
use std::net::Ipv4Addr;
use tokio::sync::mpsc;

#[derive(Debug)]
pub enum Event {
  Referee(Referee),
  SslWrapper(SslWrapperPacket),
}

pub async fn get_ssl_data() -> mpsc::Receiver<Event> {
  let (tx, rx) = mpsc::channel::<Event>(100);

  // Referee
  let ref_socket = create_multicast_socket(Ipv4Addr::new(224, 5, 23, 1), 10003);

  spawn_udp_listener::<Referee>(ref_socket.unwrap(), tx.clone(), Event::Referee);

  // Vision
  let vis_socket = create_multicast_socket(Ipv4Addr::new(224, 5, 23, 2), 10006);

  spawn_udp_listener::<SslWrapperPacket>(vis_socket.unwrap(), tx.clone(), Event::SslWrapper);

  // Drop extra sender on stream drop
  drop(tx);

  rx
}
