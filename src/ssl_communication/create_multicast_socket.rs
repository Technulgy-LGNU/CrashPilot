use std::net::Ipv4Addr;
use std::net::UdpSocket as StdUdpSocket;
use tokio::net::UdpSocket;

pub fn create_multicast_socket(multicast: Ipv4Addr, port: u16) -> anyhow::Result<UdpSocket> {
  let std_socket = match StdUdpSocket::bind(("0.0.0.0", port)) {
    Ok(std_socket) => std_socket,
    Err(err) => return Err(anyhow::Error::msg(err.to_string())),
  };
  match std_socket.join_multicast_v4(&multicast, &Ipv4Addr::UNSPECIFIED) {
      Ok(_) => (),
      Err(err) => return Err(anyhow::Error::msg(err.to_string())),
  };
  match std_socket.set_nonblocking(true) {
    Ok(_) => (),
    Err(err) => return Err(anyhow::Error::msg(err.to_string())),
  };

  match UdpSocket::from_std(std_socket) {
    Ok(socket) => Ok(socket),
    Err(err) => Err(anyhow::Error::msg(err.to_string())),
  }
}
