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
      let mut buf = [0; 65536];
      match socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(msg) = T::decode(&buf[..size]) {
            let lock = tx.write().await;
            wrap(msg, lock);
          }
        }
        Err(e) => {
          eprintln!("recv error: {:?}", e);
        }
      }
    }
  });
}
