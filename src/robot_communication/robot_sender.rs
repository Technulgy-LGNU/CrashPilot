use std::string::String;
use std::net::Ipv4Addr;
use prost::Message;
use crate::proto::CpRobot;

trait RobotSender {
  async fn robot_send_to(&mut self, message: &CpRobot) -> std::io::Result<()>;
}

struct NetworkSender {
  addr: Ipv4Addr,
  socket: tokio::net::UdpSocket,
}

struct SocketSender {
  path: String,
  stream: tokio::net::UnixStream
}

impl RobotSender for NetworkSender {
  async fn robot_send_to(&mut self, message: &CpRobot) -> std::io::Result<()> {
    let mut buf  = Vec::with_capacity(message.encoded_len());
    message.encode(&mut buf)?;

    // ip and port (used port 1024)
    let mut r_ip = String::new();
    r_ip += &self.addr.to_string();
    r_ip += ":1024";

    match self.socket.send_to(&buf[..], r_ip).await {
      Ok(_) => Ok(()),
      Err(e) => Err(e),
    }
  }
}

impl RobotSender for SocketSender {
  async fn robot_send_to(&mut self, message: &CpRobot) -> std::io::Result<()> {
    let mut buf  = Vec::with_capacity(message.encoded_len());
    message.encode(&mut buf)?;

    match self.stream.try_write(&buf) {
      Ok(_) => Ok(()),
      Err(e) => Err(e),
    }
  }
}
