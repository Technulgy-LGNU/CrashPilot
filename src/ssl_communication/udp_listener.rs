use prost::Message;
use tokio::net::UdpSocket;
use tokio::sync::MutexGuard;
use crate::communication::EventShare;
use crate::proto::{InterfaceWrapperCp, Referee, SslWrapperPacket, TrackerWrapperPacket};

pub(crate) fn spawn_udp_listener<T>(
  socket: UdpSocket,
  tx: EventShare,
  wrap: fn(T, MutexGuard<(Option<SslWrapperPacket>, Option<TrackerWrapperPacket>, Option<InterfaceWrapperCp>, Option<Referee>)>),
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    let mut buf = [0u8; 65536];

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
