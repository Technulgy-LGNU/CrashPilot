use std::collections::HashMap;
use std::net::Ipv4Addr;
use tokio::net::UdpSocket;
use crate::proto;

mod robot_sender;
mod robot_receiver;

pub async fn send_to_all_robots(socket: &UdpSocket, data: HashMap<Ipv4Addr, proto::CpRobot>, packet_id: u32) {

}
