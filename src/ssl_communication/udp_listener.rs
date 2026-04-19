use prost::Message;
use tokio::net::UdpSocket;
use tokio::sync::MutexGuard;
use crate::communication::EventShare;
use crate::proto::{CpInterface, Referee, TrackerWrapperPacket};

pub(crate) fn spawn_udp_listener<T>(
  socket: UdpSocket,
  tx: EventShare,
  wrap: fn(T, MutexGuard<(Option<TrackerWrapperPacket>, Option<CpInterface>, Option<Referee>)>),
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    let mut buf = [0u8; 65536];

    dbg!("UDP listener started on {}", socket.local_addr().unwrap());

    loop {
      match socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(msg) = T::decode(&buf[..size]) {

            let lock = tx.lock().await;

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
