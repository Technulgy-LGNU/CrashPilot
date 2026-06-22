use crate::communication::{EventShare, Events};
use prost::Message;
use tokio::net::UdpSocket;
use tokio::sync::RwLockWriteGuard;

pub fn spawn_udp_listener<T>(
  socket: UdpSocket,
  tx: EventShare,
  wrap: fn(T, RwLockWriteGuard<Events>),
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    loop {
      let mut buf = [0; 1024];
      match socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(mut latest_msg) = T::decode(&buf[..size]) {
            // Drain all buffered packets, keeping only the most recent
            loop {
              match socket.try_recv_from(&mut buf) {
                Ok((size, _)) => {
                  if let Ok(msg) = T::decode(&buf[..size]) {
                    latest_msg = msg;
                  }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => {
                  eprintln!("recv error: {:?}", e);
                  break;
                }
              }
            }

            let lock = tx.write().await;
            wrap(latest_msg, lock);
          }
        }
        Err(e) => {
          eprintln!("recv error: {:?}", e);
        }
      }
    }
  });
}
