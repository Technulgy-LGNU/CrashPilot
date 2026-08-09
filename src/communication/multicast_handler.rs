use crate::communication::{EventShare, Events};
use prost::Message;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::net::Ipv4Addr;
use std::net::{SocketAddrV4, UdpSocket as StdUdpSocket};
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::RwLockWriteGuard;
use tokio::time::sleep;

/// Generic Multicast Listener for Protobuf streams
pub fn multicast_handler<T>(
  multicast_host: Ipv4Addr,
  port: u16,
  interface: Ipv4Addr,
  tx: EventShare,
  wrap: fn(T, RwLockWriteGuard<Events>),
) where
  T: Message + Default + Send + 'static,
{
  tokio::spawn(async move {
    let mut reconnect_delay = Duration::from_secs(1);
    let max_reconnect_delay = Duration::from_secs(30);

    // Auto reconnect loop
    loop {
      let tokio_socket = match create_multicast_socket(multicast_host, port, interface) {
        Ok(socket) => {
          reconnect_delay = Duration::from_secs(1);
          socket
        }
        Err(err) => {
          eprintln!(
            "Failed to create multicast socket for {}:{} on interface {}: {}. Retrying in {:?}",
            multicast_host, port, interface, err, reconnect_delay
          );

          sleep(reconnect_delay).await;
          reconnect_delay = (reconnect_delay * 2).min(max_reconnect_delay);
          continue;
        }
      };

      // Listen to packets
      let mut buf = [0u8; 65_536];

      loop {
        match tokio_socket.recv_from(&mut buf).await {
          Ok((size, _addr)) => match T::decode(&buf[..size]) {
            Ok(msg) => {
              let lock = tx.write().await;
              wrap(msg, lock);
            }
            Err(err) => {
              eprintln!("Failed to decode multicast protobuf message: {}", err);
            }
          },

          Err(err) if err.kind() == io::ErrorKind::Interrupted => {
            continue;
          }

          Err(err) => {
            eprintln!(
              "Multicast receive error on {}:{} via {}: {}. Reconnecting in {:?}",
              multicast_host, port, interface, err, reconnect_delay
            );

            sleep(reconnect_delay).await;
            reconnect_delay = (reconnect_delay * 2).min(max_reconnect_delay);

            break;
          }
        }
      }
    }
  });
}

/// Creates a simple multicast socket
pub fn create_multicast_socket(
  multicast_host: Ipv4Addr,
  port: u16,
  interface: Ipv4Addr,
) -> io::Result<UdpSocket> {
  // Start with socket2, so reuse addr & port work
  let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

  socket.set_reuse_address(true)?;

  #[cfg(unix)]
  {
    if let Err(err) = socket.set_reuse_port(true) {
      eprintln!("Failed to set SO_REUSEPORT: {}", err);
    }
  }

  socket.bind(&SockAddr::from(SocketAddrV4::new(interface, port)))?;

  let std_socket: StdUdpSocket = socket.into();

  std_socket.join_multicast_v4(&multicast_host, &interface)?;
  std_socket.set_nonblocking(true)?;

  UdpSocket::from_std(std_socket)
}
