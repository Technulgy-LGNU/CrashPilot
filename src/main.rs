use std::io::Write;
use std::net::{UdpSocket, Ipv4Addr};
use std::os::unix::net::UnixStream;
use prost::Message;

mod proto;

fn main() {
    let multicast_addr = Ipv4Addr::new(224, 5, 23, 1);
    let port = 10003;
    let socket_path = "/tmp/rust_to_cpp.sock";

    // Connect to multicast ssl_gc controller
    let socket = UdpSocket::bind(("0.0.0.0", port)).expect("couldn't bind to address");

    socket.join_multicast_v4(&multicast_addr, &Ipv4Addr::UNSPECIFIED).expect("Error joining stream");

    let mut buf = [0u8; 65536];

    // Connect to unix socket
    let mut stream = UnixStream::connect(&socket_path).expect("couldn't connect to stream");
    println!("Connected to {}", socket_path);

    loop {
        let (size, src) = socket.recv_from(&mut buf).expect("Didn't receive data");
        println!("Received {} bytes from {}", size, src);


        match proto::Referee::decode(&buf[..size]) {
            Ok(msg) => {
                println!("{:?}", msg.command);
                let message = format!("{:?}", msg.command);
                stream.write_all(message.as_bytes()).expect("couldn't write");
            }
            Err(err) => {
                println!("{:?}", err);
            }
        }
    }
}


