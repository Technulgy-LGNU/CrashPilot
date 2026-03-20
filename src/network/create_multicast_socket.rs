use std::net::Ipv4Addr;
use tokio::net::UdpSocket;
use std::net::UdpSocket as StdUdpSocket;

pub fn create_multicast_socket(multicast: Ipv4Addr, port: u16) -> anyhow::Result<UdpSocket> {
  let std_socket = StdUdpSocket::bind(("0.0.0.0", port))?;
  std_socket.join_multicast_v4(&multicast, &Ipv4Addr::UNSPECIFIED)?;
  std_socket.set_nonblocking(true)?;

  Ok(UdpSocket::from_std(std_socket)?)
}
