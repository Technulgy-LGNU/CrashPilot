use anyhow::{Context, Result, bail};
use clap::Parser;
use core_dump::proto::{CrashpilotCommand, Referee, SslWrapperPacket, Team, TrackerWrapperPacket};
use crossterm::cursor::{Hide, MoveTo, Show};
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::terminal::{
  Clear, ClearType, EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use crossterm::{execute, queue};
use futures_util::{SinkExt, StreamExt};
use loguna::{LogReader, MessageId};
use prost::Message;
use std::collections::HashMap;
use std::fs;
use std::io::{self, IsTerminal, Write};
use std::net::Ipv4Addr;
use std::os::unix::fs::FileExt;
use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Duration;
use tempfile::{NamedTempFile, TempDir};
use tokio::net::TcpListener;
use tokio::sync::{mpsc, watch};
use tokio_tungstenite::tungstenite::{Bytes, Message as WebsocketMessage};

#[path = "../../../src/config.rs"]
#[allow(dead_code)]
mod config;
#[path = "../../../src/interface_protocol.rs"]
mod interface_protocol;
#[path = "../../../src/utils.rs"]
#[allow(dead_code)]
mod utils;
#[path = "../../../src/world_model.rs"]
#[allow(clippy::collapsible_if, clippy::needless_range_loop)]
mod world_model;

use config::Config;
use interface_protocol::{ExtendedInterfaceWrapper, WorldModelQuality};
use utils::FieldSetup;
use world_model::{CleanWorldSnapshot, WorldModel};

const EMBEDDED_INTERFACE: &[u8] = include_bytes!("../../../crashpilot-interface");
const MIN_SPEED: f64 = 0.03125;
const MAX_SPEED: f64 = 64.0;

#[derive(Debug, Parser)]
#[command(
  name = "vision-filter-replay",
  about = "Replay an SSL log through CrashPilot's vision filter and show it in the existing interface"
)]
struct Args {
  /// SSL game log to replay (.log or .log.gz).
  log_file: PathBuf,

  /// Initial playback speed multiplier.
  #[arg(short, long, default_value_t = 1.0)]
  speed: f64,

  /// Open the replay paused on its first vision packet.
  #[arg(long)]
  paused: bool,

  /// CrashPilot config used for world-model parameters.
  #[arg(short, long, default_value = "config.toml")]
  config: PathBuf,

  /// Port used by the embedded web interface.
  #[arg(long, default_value_t = 8080)]
  interface_port: u16,

  /// WebSocket port used between the replay process and interface.
  #[arg(long)]
  websocket_port: Option<u16>,

  /// Do not open the interface in the default browser automatically.
  #[arg(long)]
  no_browser: bool,
}

#[derive(Debug, Clone, Copy)]
enum ReplayKind {
  Raw,
  Tracked,
  Referee,
}

impl ReplayKind {
  fn label(self) -> &'static str {
    match self {
      Self::Raw => "raw vision",
      Self::Tracked => "tracked vision",
      Self::Referee => "referee",
    }
  }
}

#[derive(Debug, Clone, Copy)]
struct ReplayEvent {
  timestamp_ns: i64,
  kind: ReplayKind,
  payload_offset: u64,
  payload_len: usize,
}

struct ReplayCache {
  file: NamedTempFile,
}

impl ReplayCache {
  fn create() -> Result<Self> {
    let file = NamedTempFile::new().context("create replay cache")?;
    Ok(Self { file })
  }

  fn payload(&self, event: ReplayEvent) -> Vec<u8> {
    let mut payload = vec![0; event.payload_len];
    self
      .file
      .as_file()
      .read_exact_at(&mut payload, event.payload_offset)
      .expect("validated replay cache became unreadable");
    payload
  }
}

#[derive(Debug, Default)]
struct LoadStats {
  raw: usize,
  tracked: usize,
  referee: usize,
  ignored: usize,
  decode_errors: usize,
}

struct ReplayEngine {
  events: Vec<ReplayEvent>,
  cache: ReplayCache,
  cursor: usize,
  base_timestamp_ns: i64,
  end_timestamp_ns: i64,
  world_model_config: config::WorldModelConfig,
  field: FieldSetup,
  model: WorldModel,
  latest_raw: Option<SslWrapperPacket>,
  latest_tracked: Option<TrackerWrapperPacket>,
  latest_referee: Option<Referee>,
  snapshot: Option<CleanWorldSnapshot>,
}

impl ReplayEngine {
  fn new(events: Vec<ReplayEvent>, cache: ReplayCache, config: &Config) -> Self {
    let base_timestamp_ns = events.first().map_or(0, |event| event.timestamp_ns);
    let end_timestamp_ns = events
      .last()
      .map_or(base_timestamp_ns, |event| event.timestamp_ns);
    let field = FieldSetup::default();
    let model = WorldModel::new(config.world_model.clone(), field);
    Self {
      events,
      cache,
      cursor: 0,
      base_timestamp_ns,
      end_timestamp_ns,
      world_model_config: config.world_model.clone(),
      field,
      model,
      latest_raw: None,
      latest_tracked: None,
      latest_referee: None,
      snapshot: None,
    }
  }

  fn reset(&mut self) {
    self.cursor = 0;
    self.field = FieldSetup::default();
    self.model = WorldModel::new(self.world_model_config.clone(), self.field);
    self.latest_raw = None;
    self.latest_tracked = None;
    self.latest_referee = None;
    self.snapshot = None;
  }

  fn apply_next(&mut self) -> bool {
    let Some(event) = self.events.get(self.cursor).copied() else {
      return false;
    };
    self.cursor += 1;
    let payload = self.cache.payload(event);
    match event.kind {
      ReplayKind::Raw => {
        let packet = SslWrapperPacket::decode(payload.as_slice())
          .expect("validated raw replay packet no longer decodes");
        if let Some(geometry) = packet.geometry.as_ref() {
          self.field = geometry.into();
          self.model.set_field(self.field);
        }
        self.model.ingest_raw(packet.clone());
        self.latest_raw = Some(packet);
        self.refresh_snapshot();
      }
      ReplayKind::Tracked => {
        let packet = TrackerWrapperPacket::decode(payload.as_slice())
          .expect("validated tracked replay packet no longer decodes");
        self.model.ingest_tracked(packet.clone());
        self.latest_tracked = Some(packet);
        self.refresh_snapshot();
      }
      ReplayKind::Referee => {
        self.latest_referee = Some(
          Referee::decode(payload.as_slice())
            .expect("validated referee replay packet no longer decodes"),
        )
      }
    }
    true
  }

  fn refresh_snapshot(&mut self) {
    self.snapshot = self.model.snapshot(
      Team::Yellow as i32,
      &HashMap::<u32, CrashpilotCommand>::new(),
    );
  }

  fn rebuild_to(&mut self, applied_events: usize) {
    let target = applied_events.min(self.events.len());
    self.reset();
    while self.cursor < target {
      self.apply_next();
    }
  }

  fn move_to(&mut self, applied_events: usize) {
    let target = applied_events.min(self.events.len());
    if target < self.cursor {
      self.rebuild_to(target);
    } else {
      while self.cursor < target {
        self.apply_next();
      }
    }
  }

  fn seek_relative(&mut self, delta_seconds: f64) {
    let target = (self.position_seconds() + delta_seconds).clamp(0.0, self.duration_seconds());
    self.seek_to_seconds(target);
  }

  fn seek_to_seconds(&mut self, seconds: f64) {
    let target_ns = self.base_timestamp_ns
      + (seconds.clamp(0.0, self.duration_seconds()) * 1_000_000_000.0) as i64;
    let count = self
      .events
      .partition_point(|event| event.timestamp_ns <= target_ns);
    self.move_to(count.max(self.initial_cursor()));
  }

  fn step_back(&mut self) {
    self.rebuild_to(self.cursor.saturating_sub(1).max(self.initial_cursor()));
  }

  fn initial_cursor(&self) -> usize {
    self
      .events
      .iter()
      .position(|event| matches!(event.kind, ReplayKind::Raw | ReplayKind::Tracked))
      .map_or(1, |index| index + 1)
  }

  fn position_seconds(&self) -> f64 {
    let timestamp = self
      .cursor
      .checked_sub(1)
      .and_then(|index| self.events.get(index))
      .map_or(self.base_timestamp_ns, |event| event.timestamp_ns);
    (timestamp - self.base_timestamp_ns).max(0) as f64 / 1_000_000_000.0
  }

  fn duration_seconds(&self) -> f64 {
    (self.end_timestamp_ns - self.base_timestamp_ns).max(0) as f64 / 1_000_000_000.0
  }

  fn delay_to_next(&self, speed: f64) -> Duration {
    if self.cursor == 0 || self.cursor >= self.events.len() {
      return Duration::ZERO;
    }
    let current = self.events[self.cursor - 1].timestamp_ns;
    let next = self.events[self.cursor].timestamp_ns;
    Duration::from_secs_f64((next - current).max(0) as f64 / 1_000_000_000.0 / speed)
  }

  fn last_event_label(&self) -> &'static str {
    self
      .cursor
      .checked_sub(1)
      .and_then(|index| self.events.get(index))
      .map_or("none", |event| event.kind.label())
  }

  fn encoded_interface_packet(&self) -> Vec<u8> {
    ExtendedInterfaceWrapper {
      vision_raw: self.latest_raw.clone(),
      vision_tracked: self.latest_tracked.clone(),
      gc_data: self.latest_referee.clone(),
      robot_commands: Vec::new(),
      cp_gamephase: None,
      vision_raw_sources: self.model.latest_raw_packets(),
      vision_tracked_sources: self.model.latest_tracked_packets(),
      vision_filtered: self
        .snapshot
        .as_ref()
        .map(|snapshot| snapshot.tracked_frame.clone()),
      world_model_quality: self.snapshot.as_ref().map(WorldModelQuality::from_snapshot),
    }
    .encode_to_vec()
  }
}

#[derive(Debug, Clone, Copy)]
enum Control {
  TogglePlayback,
  Faster,
  Slower,
  Seek(f64),
  Next,
  Previous,
  Start,
  End,
  Restart,
  ToggleHelp,
  Quit,
}

struct Terminal {
  help: bool,
}

impl Terminal {
  fn start(tx: mpsc::UnboundedSender<Control>) -> Result<Self> {
    if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
      bail!("interactive playback requires a terminal");
    }
    enable_raw_mode().context("enable terminal raw mode")?;
    execute!(io::stdout(), EnterAlternateScreen, Hide)
      .context("enter alternate terminal screen")?;
    std::thread::spawn(move || {
      while let Ok(event) = event::read() {
        let Event::Key(key) = event else { continue };
        if let Some(control) = key_to_control(key) {
          let quitting = matches!(control, Control::Quit);
          if tx.send(control).is_err() || quitting {
            break;
          }
        }
      }
    });
    Ok(Self { help: false })
  }

  fn render(
    &self,
    args: &Args,
    engine: &ReplayEngine,
    stats: &LoadStats,
    playing: bool,
    speed: f64,
    interface_url: &str,
  ) -> Result<()> {
    let mut stdout = io::stdout();
    queue!(stdout, MoveTo(0, 0), Clear(ClearType::All))?;
    writeln!(stdout, "CrashPilot vision-filter replay")?;
    writeln!(stdout, "Log: {}", args.log_file.display())?;
    writeln!(stdout, "Interface: {interface_url}")?;
    writeln!(
      stdout,
      "State: {}   Speed: {:>6.3}x   Time: {} / {}",
      if playing { "PLAYING" } else { "PAUSED " },
      speed,
      format_time(engine.position_seconds()),
      format_time(engine.duration_seconds())
    )?;
    writeln!(
      stdout,
      "Event: {} / {} ({})   Sources now: {} raw, {} tracked",
      engine.cursor,
      engine.events.len(),
      engine.last_event_label(),
      engine.model.latest_raw_packets().len(),
      engine.model.latest_tracked_packets().len()
    )?;
    writeln!(
      stdout,
      "Loaded: {} raw, {} tracked, {} referee; {} ignored, {} decode errors",
      stats.raw, stats.tracked, stats.referee, stats.ignored, stats.decode_errors
    )?;
    writeln!(stdout)?;
    if self.help {
      writeln!(stdout, "Playback controls")?;
      writeln!(stdout, "  Space / p       Play or pause")?;
      writeln!(stdout, "  + / -           Double or halve playback speed")?;
      writeln!(
        stdout,
        "  Left / Right    Seek backward or forward 1 second"
      )?;
      writeln!(
        stdout,
        "  Shift+arrows    Seek backward or forward 10 seconds"
      )?;
      writeln!(stdout, "  , / .           Previous or next log event")?;
      writeln!(stdout, "  Home / End      Jump to start or end")?;
      writeln!(stdout, "  r                Restart and play")?;
      writeln!(stdout, "  ?                Hide this help")?;
      writeln!(stdout, "  q / Esc / Ctrl-C Quit")?;
    } else {
      writeln!(
        stdout,
        "Space play/pause  +/- speed  ←/→ seek  ,/. step  ? help  q quit"
      )?;
    }
    stdout.flush()?;
    Ok(())
  }
}

impl Drop for Terminal {
  fn drop(&mut self) {
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), Show, LeaveAlternateScreen);
  }
}

fn key_to_control(key: KeyEvent) -> Option<Control> {
  if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
    return Some(Control::Quit);
  }
  match key.code {
    KeyCode::Char(' ') | KeyCode::Char('p') => Some(Control::TogglePlayback),
    KeyCode::Char('+') | KeyCode::Char('=') => Some(Control::Faster),
    KeyCode::Char('-') => Some(Control::Slower),
    KeyCode::Left => Some(Control::Seek(
      if key.modifiers.contains(KeyModifiers::SHIFT) {
        -10.0
      } else {
        -1.0
      },
    )),
    KeyCode::Right => Some(Control::Seek(
      if key.modifiers.contains(KeyModifiers::SHIFT) {
        10.0
      } else {
        1.0
      },
    )),
    KeyCode::Char(',') => Some(Control::Previous),
    KeyCode::Char('.') => Some(Control::Next),
    KeyCode::Home => Some(Control::Start),
    KeyCode::End => Some(Control::End),
    KeyCode::Char('r') => Some(Control::Restart),
    KeyCode::Char('?') => Some(Control::ToggleHelp),
    KeyCode::Char('q') | KeyCode::Esc => Some(Control::Quit),
    _ => None,
  }
}

struct InterfaceProcess {
  child: Child,
  _temp_dir: TempDir,
}

impl InterfaceProcess {
  fn start(interface_port: u16, websocket_port: u16) -> Result<Self> {
    let temp_dir = tempfile::tempdir().context("create temporary interface directory")?;
    let binary_path = temp_dir.path().join("crashpilot-interface");
    let config_path = temp_dir.path().join("interface.toml");
    fs::write(&binary_path, EMBEDDED_INTERFACE)
      .with_context(|| format!("write embedded interface to {}", binary_path.display()))?;
    let mut permissions = fs::metadata(&binary_path)?.permissions();
    permissions.set_mode(0o755);
    fs::set_permissions(&binary_path, permissions)?;
    fs::write(
      &config_path,
      format!(
        "[server]\nhost = \"127.0.0.1\"\nport = {interface_port}\n\n\
         [crashpilot]\nws_url = \"ws://127.0.0.1:{websocket_port}/ws\"\n\
         reconnect_delay_ms = 250\nhandshake_timeout_ms = 10000\nwrite_timeout_ms = 2000\n"
      ),
    )?;
    let child = Command::new(&binary_path)
      .current_dir(temp_dir.path())
      .env("CRASHPILOT_CONFIG", &config_path)
      .stdin(Stdio::null())
      .stdout(Stdio::null())
      .stderr(Stdio::null())
      .spawn()
      .context("start embedded CrashPilot interface")?;
    Ok(Self {
      child,
      _temp_dir: temp_dir,
    })
  }

  fn check_running(&mut self) -> Result<()> {
    if let Some(status) = self.child.try_wait()? {
      bail!("embedded interface exited early with {status}; is its HTTP port already in use?");
    }
    Ok(())
  }
}

impl Drop for InterfaceProcess {
  fn drop(&mut self) {
    let _ = self.child.kill();
    let _ = self.child.wait();
  }
}

fn load_log(path: &Path) -> Result<(Vec<ReplayEvent>, ReplayCache, LoadStats)> {
  let mut reader =
    LogReader::open(path).with_context(|| format!("open SSL log {}", path.display()))?;
  let mut cache = ReplayCache::create()?;
  let mut payload_offset = 0_u64;
  let mut events = Vec::new();
  let mut stats = LoadStats::default();
  while let Some(message) = reader.next_message().context("read SSL log message")? {
    let kind = match message.message_id {
      MessageId::Vision2014 => ReplayKind::Raw,
      MessageId::VisionTracker2020 => ReplayKind::Tracked,
      MessageId::Referee2013 => ReplayKind::Referee,
      _ => {
        stats.ignored += 1;
        continue;
      }
    };
    let decodes = match kind {
      ReplayKind::Raw => SslWrapperPacket::decode(message.payload.as_slice()).is_ok(),
      ReplayKind::Tracked => TrackerWrapperPacket::decode(message.payload.as_slice()).is_ok(),
      ReplayKind::Referee => Referee::decode(message.payload.as_slice()).is_ok(),
    };
    if !decodes {
      stats.decode_errors += 1;
      continue;
    }
    match kind {
      ReplayKind::Raw => stats.raw += 1,
      ReplayKind::Tracked => stats.tracked += 1,
      ReplayKind::Referee => stats.referee += 1,
    }
    cache.file.write_all(&message.payload)?;
    events.push(ReplayEvent {
      timestamp_ns: message.timestamp_ns,
      kind,
      payload_offset,
      payload_len: message.payload.len(),
    });
    payload_offset += message.payload.len() as u64;
  }
  events.sort_by_key(|event| event.timestamp_ns);
  if events.is_empty() {
    bail!("log contains no decodable Vision2014, VisionTracker2020, or referee messages");
  }
  if stats.raw + stats.tracked == 0 {
    bail!("log contains no decodable raw or tracked vision packets");
  }
  if stats.raw == 0 || stats.tracked == 0 {
    eprintln!(
      "warning: log contains {} raw and {} tracked packets; comparison needs both",
      stats.raw, stats.tracked
    );
  }
  Ok((events, cache, stats))
}

fn format_time(seconds: f64) -> String {
  let total_ms = (seconds.max(0.0) * 1000.0).round() as u64;
  let minutes = total_ms / 60_000;
  let seconds = (total_ms / 1_000) % 60;
  let millis = total_ms % 1_000;
  format!("{minutes:02}:{seconds:02}.{millis:03}")
}

fn open_browser(url: &str) {
  let _ = Command::new("xdg-open")
    .arg(url)
    .stdin(Stdio::null())
    .stdout(Stdio::null())
    .stderr(Stdio::null())
    .spawn();
}

#[derive(Clone)]
struct ReplayWebsocket {
  latest: watch::Sender<Option<Vec<u8>>>,
}

impl ReplayWebsocket {
  async fn bind(port: u16) -> Result<Self> {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port))
      .await
      .with_context(|| format!("bind replay WebSocket to 127.0.0.1:{port}"))?;
    let (latest, _) = watch::channel::<Option<Vec<u8>>>(None);
    let publisher = Self { latest };
    let clients = publisher.clone();
    tokio::spawn(async move {
      loop {
        let Ok((stream, _peer)) = listener.accept().await else {
          continue;
        };
        let Ok(socket) = tokio_tungstenite::accept_async(stream).await else {
          continue;
        };
        let (mut outgoing, mut incoming) = socket.split();
        let mut updates = clients.latest.subscribe();
        tokio::spawn(async move {
          let initial = updates.borrow().clone();
          if let Some(payload) = initial
            && outgoing
              .send(WebsocketMessage::Binary(Bytes::from(payload)))
              .await
              .is_err()
          {
            return;
          }
          while updates.changed().await.is_ok() {
            let payload = updates.borrow_and_update().clone();
            let Some(payload) = payload else { continue };
            if outgoing
              .send(WebsocketMessage::Binary(Bytes::from(payload)))
              .await
              .is_err()
            {
              break;
            }
          }
        });
        tokio::spawn(async move {
          while let Some(message) = incoming.next().await {
            if message.is_err() {
              break;
            }
          }
        });
      }
    });
    Ok(publisher)
  }

  fn publish(&self, engine: &ReplayEngine) {
    self
      .latest
      .send_replace(Some(engine.encoded_interface_packet()));
  }
}

#[tokio::main]
async fn main() -> Result<()> {
  let args = Args::parse();
  if !args.speed.is_finite() || args.speed <= 0.0 {
    bail!("--speed must be a finite number greater than zero");
  }
  let (events, cache, stats) = load_log(&args.log_file)?;
  let config_path = args.config.to_string_lossy();
  let mut config = config::load_or_create_config(&config_path).map_err(|error| {
    anyhow::anyhow!(
      "load CrashPilot config {}: {}",
      args.config.display(),
      error
    )
  })?;
  let websocket_port = args.websocket_port.unwrap_or(config.server.websocket_port);
  config.server.websocket_host = Ipv4Addr::LOCALHOST;
  config.server.websocket_port = websocket_port;

  let websocket = ReplayWebsocket::bind(websocket_port).await?;

  let mut interface = InterfaceProcess::start(args.interface_port, websocket_port)?;
  tokio::time::sleep(Duration::from_millis(400)).await;
  interface.check_running()?;
  let interface_url = format!("http://127.0.0.1:{}", args.interface_port);
  if !args.no_browser {
    open_browser(&interface_url);
  }

  let mut engine = ReplayEngine::new(events, cache, &config);
  engine.rebuild_to(engine.initial_cursor());
  websocket.publish(&engine);

  let (control_tx, mut control_rx) = mpsc::unbounded_channel();
  let mut terminal = Terminal::start(control_tx)?;
  let mut playing = !args.paused;
  let mut speed = args.speed.clamp(MIN_SPEED, MAX_SPEED);
  terminal.render(&args, &engine, &stats, playing, speed, &interface_url)?;

  loop {
    let control = if playing && engine.cursor < engine.events.len() {
      let delay = engine.delay_to_next(speed);
      tokio::select! {
        control = control_rx.recv() => control,
        _ = tokio::time::sleep(delay) => {
          engine.apply_next();
          websocket.publish(&engine);
          if engine.cursor == engine.events.len() {
            playing = false;
          }
          terminal.render(&args, &engine, &stats, playing, speed, &interface_url)?;
          continue;
        }
      }
    } else {
      control_rx.recv().await
    };

    let Some(control) = control else { break };
    let mut state_changed = false;
    match control {
      Control::TogglePlayback => {
        if engine.cursor == engine.events.len() {
          engine.rebuild_to(engine.initial_cursor());
          state_changed = true;
        }
        playing = !playing;
      }
      Control::Faster => speed = (speed * 2.0).min(MAX_SPEED),
      Control::Slower => speed = (speed * 0.5).max(MIN_SPEED),
      Control::Seek(delta) => {
        engine.seek_relative(delta);
        state_changed = true;
      }
      Control::Next => {
        playing = false;
        state_changed = engine.apply_next();
      }
      Control::Previous => {
        playing = false;
        engine.step_back();
        state_changed = true;
      }
      Control::Start => {
        engine.rebuild_to(engine.initial_cursor());
        state_changed = true;
      }
      Control::End => {
        engine.move_to(engine.events.len());
        playing = false;
        state_changed = true;
      }
      Control::Restart => {
        engine.rebuild_to(engine.initial_cursor());
        playing = true;
        state_changed = true;
      }
      Control::ToggleHelp => terminal.help = !terminal.help,
      Control::Quit => break,
    }
    if state_changed {
      websocket.publish(&engine);
    }
    terminal.render(&args, &engine, &stats, playing, speed, &interface_url)?;
  }

  Ok(())
}
