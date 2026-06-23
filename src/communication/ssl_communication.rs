use crate::communication::create_multicast_socket::create_multicast_socket;
use crate::communication::udp_listener::spawn_udp_listener;
use crate::communication::EventShare;
use crate::config::Config;
use core_dump::proto::{Referee, SslWrapperPacket, TrackerWrapperPacket};

pub fn get_ssl_data(cfg: &Config, tx: EventShare) {
  // Vision raw
  let vis_raw_socket =
    match create_multicast_socket(cfg.ssl.ssl_vision_raw_ip, cfg.ssl.ssl_vision_raw_port) {
      Ok(s) => s,
      Err(err) => panic!("Failed to create multicast socket for raw-vision: {}", err),
    };

  spawn_udp_listener::<SslWrapperPacket>(vis_raw_socket, tx.clone(), |event, mut lock| {
    lock.raw = Some(event)
  });

  // Vision Tracked
  let vis_tracked_socket = match create_multicast_socket(
    cfg.ssl.ssl_vision_tracked_ip,
    cfg.ssl.ssl_vision_tracked_port,
  ) {
    Ok(s) => s,
    Err(e) => panic!(
      "Failed to create multicast socket for tracked-vision: {}",
      e
    ),
  };

  spawn_udp_listener::<TrackerWrapperPacket>(vis_tracked_socket, tx.clone(), |event, mut lock| {
    lock.tracked = Some(event);
  });

  // Referee
  let ref_socket = match create_multicast_socket(cfg.ssl.ssl_gc_ip, cfg.ssl.ssl_gc_port) {
    Ok(s) => s,
    Err(err) => panic!("Failed to create multicast socket for referee: {}", err),
  };

  spawn_udp_listener::<Referee>(ref_socket, tx.clone(), |event, mut lock| {
    lock.gc = Some(event);
  });

  // Drop extra sender on stream drop
  drop(tx);
}
