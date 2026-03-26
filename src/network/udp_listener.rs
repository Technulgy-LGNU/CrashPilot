use crate::ssl::Event;
use prost::Message;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;

pub(crate) fn spawn_udp_listener<T>(
  socket: UdpSocket,
  tx: mpsc::Sender<Event>,
  wrap: fn(T) -> Event,
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    let mut buf = [0u8; 65536];

    loop {
      match socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(msg) = T::decode(&buf[..size]) {
            let event = wrap(msg);

            // If receiver dropped, stop task
            if tx.send(event).await.is_err() {
              break;
            }
          }
        }
        Err(e) => {
          eprintln!("recv error: {:?}", e);
        }
      }
    }
  });
}
