use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::net::Ipv4Addr;
use std::net::{SocketAddrV4, UdpSocket as StdUdpSocket};
use tokio::net::UdpSocket;

pub fn create_multicast_socket(
  multicast: Ipv4Addr,
  port: u16,
  interface: Ipv4Addr,
) -> anyhow::Result<UdpSocket> {
  let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
  socket.set_reuse_address(true)?;
  #[cfg(unix)]
  socket.set_reuse_port(true)?;
  socket.bind(&SockAddr::from(SocketAddrV4::new(
    Ipv4Addr::UNSPECIFIED,
    port,
  )))?;
  let std_socket: StdUdpSocket = socket.into();
  match std_socket.join_multicast_v4(&multicast, &interface) {
    Ok(_) => (),
    Err(err) => {
      return Err(anyhow::Error::msg(
        "Error during multicast join: ".to_owned() + &*err.to_string(),
      ));
    }
  };
  match std_socket.set_nonblocking(true) {
    Ok(_) => (),
    Err(err) => {
      return Err(anyhow::Error::msg(
        "Error during setting socket to non blocking: ".to_owned() + &*err.to_string(),
      ));
    }
  };

  match UdpSocket::from_std(std_socket) {
    Ok(socket) => Ok(socket),
    Err(err) => Err(anyhow::Error::msg(
      "Error converting std socket to tokio socket: ".to_owned() + &*err.to_string(),
    )),
  }
}
