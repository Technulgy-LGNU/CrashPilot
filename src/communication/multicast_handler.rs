use crate::communication::{EventShare, Events};
use prost::Message;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::net::Ipv4Addr;
use std::net::{SocketAddrV4, UdpSocket as StdUdpSocket};
use tokio::net::UdpSocket;
use tokio::sync::RwLockWriteGuard;

/// Generic Multicast Listener for Protobuff streams
pub fn multicast_handler<T>(
  multicas_host: Ipv4Addr,
  port: u16,
  interface: Ipv4Addr,
  tx: EventShare,
  wrap: fn(T, RwLockWriteGuard<Events>),
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    // Create Socket2, so reuse Addr/Port is possible
    let socket = match Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)) {
      Ok(socket) => socket,
      Err(e) => {
        panic!("Failed to create socket: {}", e);
      }
    };

    // Allow reuse of address and port
    match socket.set_reuse_address(true) {
      Ok(_) => {}
      Err(e) => {
        panic!("Failed to set reuse address: {}", e);
      }
    }
    match socket.set_reuse_port(true) {
      Ok(_) => {}
      Err(e) => {
        eprintln!("Failed to set reuse port: {}", e);
      }
    }

    // Bind the socket2
    match socket.bind(&SockAddr::from(SocketAddrV4::new(interface, port))) {
      Ok(_) => {}
      Err(e) => {
        eprintln!("Failed to bind socket: {}", e);
      }
    }

    // Convert to stdsocket
    let std_socket: StdUdpSocket = socket.into();

    // Join multicast stream
    match std_socket.join_multicast_v4(&multicas_host, &interface) {
      Ok(_) => (),
      Err(err) => {
        eprintln!("Error during multicast join: {}", err);
      }
    };

    // Set nonblocking
    match std_socket.set_nonblocking(true) {
      Ok(_) => (),
      Err(err) => {
        eprintln!("Error during setting socket to non blocking: {}", err);
      }
    };

    // Convert to tokio udpsocket
    let tokio_socket = match UdpSocket::from_std(std_socket) {
      Ok(socket) => socket,
      Err(err) => {
        panic!("Error converting std socket to tokio socket: {}", err);
      }
    };

    loop {
      let mut buf = [0; 65536];
      match tokio_socket.recv_from(&mut buf).await {
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
