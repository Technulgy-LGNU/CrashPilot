use futures_util::StreamExt;
use prost::Message;
use tokio::net::TcpListener;
use tokio::sync::mpsc::Sender;
use crate::config;
use crate::proto::CpInterface;
use crate::ssl_communication::Event;

pub async fn spawn_websocket(cfg: &config::Config, tx: Sender<Event>) {
  let addr = format!("{}:{}", cfg.server.websocket_host, cfg.server.websocket_port);

  // Create raw TCP Stream
  let tcp_socket = match TcpListener::bind(&addr).await {
    Ok(socket) => socket,
    Err(e) => panic!("Can't bind websocket to {}: {}", addr, e),
  };

  // Accept incoming connections
  tokio::spawn(async move {
    loop {
      let (stream, peer_addr) = match tcp_socket.accept().await {
        Ok(connection) => connection,
        Err(e) => {
          eprintln!("Failed to accept websocket TCP connection: {}", e);
          continue;
        }
      };

      let ws_stream = match tokio_tungstenite::accept_async(stream).await {
        Ok(ws_stream) => ws_stream,
        Err(e) => {
          eprintln!(
            "WebSocket handshake failed from {}: {:?}. Ensure the client connects with ws:// and sends a valid HTTP Upgrade request.",
            peer_addr,
            e
          );
          continue;
        }
      };

      let (_, mut incoming) =  ws_stream.split();

      // Process incoming messages
      let tx = tx.clone();
      tokio::spawn(async move {
        while let Some(msg) = incoming.next().await {
          match msg {
            Ok(msg) if msg.is_binary() => {
              let data = msg.into_data();

              match CpInterface::decode(&*data) {
                Ok(decoded) => {
                  tx.send(Event::Websocket(decoded)).await.unwrap_or_else(|e| {
                    eprintln!("Failed to send WebSocket event: {}", e);
                  });
                }
                Err(e) => {
                  eprintln!("Protobuf decode error: {}", e);
                }
              }
            }
            Ok(_) => {}
            Err(e) => {
              eprintln!("WebSocket error: {}", e);
              break;
            }
          }
        }
      });
    }
  });
}
