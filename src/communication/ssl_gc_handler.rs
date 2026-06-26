use crate::communication::EventShare;
use crate::config::Config;
use crate::TEAM_NAME;
use anyhow::{bail, Context};
use core_dump::proto::{
    controller_reply, controller_to_team, team_to_controller, AdvantageChoice, ControllerReply, ControllerToTeam,
    Team, TeamRegistration, TeamToController,
};
use prost::Message;
use std::net::SocketAddr;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Duration};

const MAX_FRAME_LEN: usize = 1024 * 1024;
const RECONNECT_DELAY: Duration = Duration::from_secs(5);

#[derive(Debug)]
struct GameControllerRequest {
  request: TeamToController,
  reply: oneshot::Sender<anyhow::Result<ControllerToTeam>>,
}

#[derive(Clone, Debug)]
pub struct SslGameController {
  tx: mpsc::Sender<GameControllerRequest>,
}

impl SslGameController {
  pub fn spawn(cfg: &Config, events: EventShare) -> Self {
    let (tx, rx) = mpsc::channel(32);
    let actor = GameControllerActor {
      addr: SocketAddr::new(cfg.ssl.ssl_gc_msg_ip.into(), cfg.ssl.ssl_gc_msg_port),
      team_name: TEAM_NAME.to_string(),
      team: None,
      events,
    };

    tokio::spawn(actor.run(rx));

    Self { tx }
  }

  pub async fn request(&self, request: TeamToController) -> anyhow::Result<ControllerToTeam> {
    let (reply_tx, reply_rx) = oneshot::channel();
    self
      .tx
      .send(GameControllerRequest {
        request,
        reply: reply_tx,
      })
      .await
      .context("SSL GameController handler is not running")?;

    reply_rx
      .await
      .context("SSL GameController handler stopped before replying")?
  }

  pub async fn desired_keeper(&self, id: i32) -> anyhow::Result<ControllerToTeam> {
    self
      .request(TeamToController {
        signature: None,
        msg: Some(team_to_controller::Msg::DesiredKeeper(id)),
      })
      .await
  }

  pub async fn advantage_choice(
    &self,
    choice: AdvantageChoice,
  ) -> anyhow::Result<ControllerToTeam> {
    self
      .request(TeamToController {
        signature: None,
        msg: Some(team_to_controller::Msg::AdvantageChoice(choice as i32)),
      })
      .await
  }

  pub async fn substitute_bot(&self, requested: bool) -> anyhow::Result<ControllerToTeam> {
    self
      .request(TeamToController {
        signature: None,
        msg: Some(team_to_controller::Msg::SubstituteBot(requested)),
      })
      .await
  }

  pub async fn ping(&self) -> anyhow::Result<ControllerToTeam> {
    self
      .request(TeamToController {
        signature: None,
        msg: Some(team_to_controller::Msg::Ping(true)),
      })
      .await
  }
}

struct GameControllerActor {
  addr: SocketAddr,
  team_name: String,
  team: Option<Team>,
  events: EventShare,
}

impl GameControllerActor {
  async fn run(self, mut rx: mpsc::Receiver<GameControllerRequest>) {
    loop {
      let mut connection = match self.connect_and_register().await {
        Ok(connection) => connection,
        Err(err) => {
          eprintln!(
            "SSL GameController: failed to connect/register at {}: {:#}",
            self.addr, err
          );
          sleep(RECONNECT_DELAY).await;
          continue;
        }
      };

      while let Some(command) = rx.recv().await {
        match self.send_request(&mut connection, command.request).await {
          Ok(reply) => {
            let _ = command.reply.send(Ok(reply));
          }
          Err(err) => {
            let _ = command.reply.send(Err(err));
            break;
          }
        }
      }

      if rx.is_closed() {
        return;
      }
    }
  }

  async fn connect_and_register(&self) -> anyhow::Result<GameControllerConnection> {
    let mut stream = TcpStream::connect(self.addr).await?;
    stream.set_nodelay(true)?;

    let hello: ControllerToTeam = recv_message(&mut stream).await?;
    publish_controller_message(&self.events, hello.clone()).await;
    let mut token = controller_reply(&hello).and_then(|reply| reply.next_token.clone());

    let registration = TeamRegistration {
      team_name: self.team_name.clone(),
      signature: None,
      team: self.team.map(|team| team as i32),
    };
    send_message(&mut stream, &registration).await?;

    let reply: ControllerToTeam = recv_message(&mut stream).await?;
    publish_controller_message(&self.events, reply.clone()).await;
    let Some(controller_reply) = controller_reply(&reply) else {
      bail!("registration reply did not contain a ControllerReply");
    };

    if controller_reply.status_code != Some(controller_reply::StatusCode::Ok as i32) {
      bail!(
        "registration rejected: {}",
        controller_reply
          .reason
          .clone()
          .unwrap_or_else(|| "unknown reason".to_string())
      );
    }

    token = controller_reply.next_token.clone().or(token);

    println!("SSL GameController: registered as {}", self.team_name);

    Ok(GameControllerConnection { stream, token })
  }

  async fn send_request(
    &self,
    connection: &mut GameControllerConnection,
    request: TeamToController,
  ) -> anyhow::Result<ControllerToTeam> {
    send_message(&mut connection.stream, &request).await?;

    let reply: ControllerToTeam = recv_message(&mut connection.stream).await?;
    publish_controller_message(&self.events, reply.clone()).await;

    if let Some(controller_reply) = controller_reply(&reply) {
      connection.token = controller_reply.next_token.clone();
    }

    Ok(reply)
  }
}

struct GameControllerConnection {
  stream: TcpStream,
  token: Option<String>,
}

async fn publish_controller_message(events: &EventShare, message: ControllerToTeam) {
  let mut lock = events.write().await;
  lock.gc_team_messages.push(message);
}

fn controller_reply(message: &ControllerToTeam) -> Option<&ControllerReply> {
  match message.msg.as_ref() {
    Some(controller_to_team::Msg::ControllerReply(reply)) => Some(reply),
    None => None,
  }
}

async fn send_message<M: Message>(stream: &mut TcpStream, message: &M) -> anyhow::Result<()> {
  let mut payload = Vec::with_capacity(message.encoded_len());
  message.encode(&mut payload)?;

  let mut frame = Vec::new();
  prost::encoding::encode_varint(payload.len() as u64, &mut frame);
  frame.extend_from_slice(&payload);

  stream.write_all(&frame).await?;
  Ok(())
}

async fn recv_message<M: Message + Default>(stream: &mut TcpStream) -> anyhow::Result<M> {
  let len = read_variant(stream).await? as usize;
  if len > MAX_FRAME_LEN {
    bail!("SSL GameController frame too large: {len} bytes");
  }

  let mut payload = vec![0_u8; len];
  stream.read_exact(&mut payload).await?;
  Ok(M::decode(payload.as_slice())?)
}

async fn read_variant(stream: &mut TcpStream) -> anyhow::Result<u64> {
  let mut shift = 0_u32;
  let mut value = 0_u64;

  for _ in 0..10 {
    let byte = stream.read_u8().await?;
    value |= ((byte & 0x7f) as u64) << shift;
    if byte & 0x80 == 0 {
      return Ok(value);
    }
    shift += 7;
  }

  bail!("invalid protobuf variant frame length")
}
