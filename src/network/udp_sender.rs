use std::collections::HashMap;
use std::net::SocketAddr;
use prost::Message;
use tokio::net::UdpSocket;

pub async fn send_all<T>(
  socket: &UdpSocket,
  packets: HashMap<SocketAddr, T>
) -> anyhow::Result<()>
where
  T: Message,
{
  for (addr, msg) in packets {
    let mut buf = Vec::with_capacity(msg.encoded_len());
    msg.encode(&mut buf)?;
    socket.send_to(&buf, &addr).await?;
  }

  Ok(())
}
