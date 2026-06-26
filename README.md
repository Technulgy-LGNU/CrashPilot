# CrashPilot

CrashPilot is the main Rust controller for our RoboCup Small Size League (SSL)
team. It sits between SSL-Vision, the SSL GameController, the operator
interface, and the robots. Every control tick it collects the latest world and
operator state, updates an internal model of the match, chooses robot commands,
and sends those commands to the configured robots over UDP.

The controller is built around the `core_dump` protobuf types used by the team.
Incoming SSL packets, interface commands, robot feedback, and outgoing robot
commands are all encoded as protobuf messages.

## What CrashPilot Does

- Receives SSL-Vision raw geometry and detection packets over multicast.
- Receives SSL-Vision tracked packets over multicast.
- Receives SSL referee packets from the GameController multicast stream.
- Optionally connects to the SSL GameController team TCP protocol.
- Hosts a WebSocket endpoint for the local operator interface.
- Receives robot feedback packets over UDP and tracks robot heartbeats.
- Builds a normalized `WorldState` from vision, referee, interface, and robot
  feedback data.
- Runs manual, game, or test control logic.
- Adapts AI decisions from the `artificial_incompetence` crate into low-level
  robot commands.
- Sends `CpRobot` command packets to every reachable configured robot.
- Optionally exposes Prometheus metrics and pushes robot command logs to Loki.

## Repository Layout

```text
.
|-- src/
|   |-- main.rs                         # CLI entry point
|   |-- lib.rs                          # CrashPilot runtime and control loop
|   |-- config.rs                       # TOML config loading and defaults
|   |-- communication.rs                # Communication task wiring
|   |-- communication/
|   |   |-- ssl_communication.rs        # SSL-Vision and referee multicast input
|   |   |-- ssl_gc_handler.rs           # SSL GameController team TCP protocol
|   |   |-- interface.rs                # WebSocket interface server
|   |   |-- robot_receiver.rs           # Robot feedback UDP input
|   |   |-- robot_sender.rs             # Robot command UDP output
|   |   |-- create_multicast_socket.rs  # Multicast socket setup
|   |   |-- udp_listener.rs             # Generic protobuf UDP listener
|   |   `-- loki.rs                     # Optional Loki publisher
|   |-- game_logic.rs                   # Mode dispatcher
|   |-- game_logic/
|   |   |-- types.rs                    # WorldState, Robot, BallData, referee state
|   |   |-- mode_manual.rs              # Direct operator command mode
|   |   |-- mode_game.rs                # Match/game strategy mode
|   |   |-- mode_test.rs                # Robot hardware test mode
|   |   |-- ai_handler.rs               # AI command adapter
|   |   |-- ball_placement.rs
|   |   |-- defend.rs
|   |   `-- kick_off.rs
|   |-- helpers/                        # Robot data and geometry helpers
|   |-- metrics.rs                      # Optional Prometheus endpoint
|   |-- interface.rs                    # Optional embedded interface launcher
|   `-- utils.rs                        # Shared runtime structs and socket setup
|-- crates/
|   |-- artificial_incompetence/        # AI model, inference, and training code
|   `-- strategy_presets/               # Strategy preset helpers
|-- grafana/                            # Example Grafana dashboards
|-- config.toml                         # Local runtime configuration
|-- Cargo.toml
`-- README.md
```

## Runtime Architecture

CrashPilot is centered on the `CrashPilot` struct in `src/lib.rs`.

```text
SSL-Vision raw multicast --\
SSL-Vision tracked ---------\
SSL GameController ref ------+--> Events --> recv --> update_data --> update_logic --> send
Interface WebSocket --------/
Robot UDP feedback --------/
                                                               |             |
                                                               |             |--> UDP CpRobot to robots
                                                               |             `--> WebSocket snapshot to UI
                                                               `--> WorldState + AI state
```

Communication tasks write the newest packets into a shared `Events` value. The
main loop drains that latest snapshot on every tick, so slow or bursty producers
do not build an unbounded queue of old world states.

The main loop currently runs every 8 ms:

```rust
let mut tick = interval(Duration::from_millis(4));
```

Each tick performs:

1. `recv()` drains latest incoming events from `EventShare`.
2. `interpret()` stores packets in `PacketBuffer`, updates selected team/side,
   updates robot feedback, and applies operator commands.
3. `update_data()` converts tracked vision into `WorldState` and the AI-facing
   game state.
4. `update_logic()` dispatches to manual, game, or test logic.
5. `send()` sends robot UDP commands and publishes a WebSocket snapshot.

## Configuration

CrashPilot loads `config.toml` from the current working directory. If the file
does not exist, `config::load_or_create_config("config.toml")` writes a default
configuration.

The main config sections are:

```toml
[ssl]
ssl_vision_raw_ip = "224.5.23.2"
ssl_vision_raw_port = 10006
ssl_vision_tracked_ip = "224.5.23.2"
ssl_vision_tracked_port = 10010
ssl_gc_ip = "224.5.23.1"
ssl_gc_port = 10003
ssl_gc_msg_ip = "127.0.0.1"
ssl_gc_msg_port = 10008

[server]
robot_socket_host = "0.0.0.0"
robot_socket_port = 8192
robots_port = 1024
robot_receive_port = 2048
websocket_host = "0.0.0.0"
websocket_port = 4096

[logging]
prometheus_host = "0.0.0.0"
prometheus_port = 9090
loki_host = "10.0.64.2"
loki_port = 3100

[robots]
0 = { ip = "10.0.64.100", substitution_pos = { x = 400, y = 0 } }
1 = { ip = "10.0.64.101", substitution_pos = { x = 800, y = 0 } }
4 = { ip = "10.0.64.104", substitution_pos = { x = 2000, y = 0 } }
```

Important ports:

| Setting | Purpose |
| --- | --- |
| `ssl_vision_raw_ip:ssl_vision_raw_port` | SSL-Vision raw detection and geometry multicast input |
| `ssl_vision_tracked_ip:ssl_vision_tracked_port` | SSL-Vision tracker multicast input |
| `ssl_gc_ip:ssl_gc_port` | SSL GameController referee multicast input |
| `ssl_gc_msg_ip:ssl_gc_msg_port` | Optional GameController team TCP endpoint |
| `robot_socket_host:robot_socket_port` | Local UDP socket used to send robot commands |
| `robots_port` | Destination UDP port on each robot |
| `robot_receive_port` | Local UDP port for robot feedback |
| `websocket_host:websocket_port` | Operator interface WebSocket endpoint |

Robot IDs are configured as TOML table keys under `[robots]`. Each robot needs
an IP address and a `substitution_pos`, used during timeout/substitution
positioning.

## Building

Install a recent Rust toolchain with edition 2024 support, then build:

```sh
cargo build
```

Build with optional integrations:

```sh
cargo build --features interface
cargo build --features ssl_game_controller
cargo build --features prometheus
cargo build --features loki
cargo build --features "interface ssl_game_controller prometheus loki"
```

The `interface` feature embeds and starts the `crashpilot-interface` binary from
the repository root. That file must be a compiled executable-compatible binary
for the target system.

## Running

Run with default AI:

```sh
cargo run
```

Run with an AI checkpoint:

```sh
cargo run -- --ai-checkpoint path/to/model-or-checkpoint
```

`--ai-checkpoint` and `--ai-model` are aliases. The path may point to a
`model.safetensors` file, a checkpoint directory, or a run directory containing
`checkpoint_*` directories.

Embedded callers can also use:

```sh
CRASHPILOT_AI_CHECKPOINT=path/to/checkpoint cargo run
```

Run with common competition integrations:

```sh
cargo run --features "interface ssl_game_controller prometheus"
```

## Feature Flags

| Feature | Effect |
| --- | --- |
| `interface` | Starts the embedded `crashpilot-interface` binary alongside the controller. |
| `ssl_game_controller` | Enables the SSL GameController team TCP protocol handler and goalie requests. |
| `prometheus` | Starts an HTTP metrics server with `/metrics` and `/health`. |
| `loki` | Publishes outbound robot command logs to Loki. |

## Communication Details

### SSL-Vision and Referee Input

`src/communication/ssl_communication.rs` creates multicast sockets for:

- raw SSL-Vision packets as `SslWrapperPacket`
- tracked SSL-Vision packets as `TrackerWrapperPacket`
- referee packets as `Referee`

Each listener decodes protobuf UDP datagrams and stores only the latest packet
of each type in `Events`.

### Interface WebSocket

`src/communication/interface.rs` hosts a WebSocket server at
`server.websocket_host:server.websocket_port`.

Incoming binary WebSocket messages are decoded as `InterfaceWrapperCp` and used
for:

- robot commands in manual mode
- selected controller mode
- test selection and selected test robots
- team color and field side
- goalkeeper ID
- optional manual use of referee halt/stop commands

Outgoing messages are encoded as `CpInterfaceWrapper` and include:

- latest raw vision packet
- latest tracked vision packet
- latest referee packet, if available
- latest command packet for every robot

Outbound WebSocket publishing keeps only the newest snapshot. If a client is
slow, it skips stale frames instead of delaying the controller.

### Robot Feedback Input

`src/communication/robot_receiver.rs` binds
`server.robot_socket_host:server.robot_receive_port` and expects UDP protobuf
`RobotCp` feedback messages from known robot IPs.

When feedback is received from a configured robot IP:

- the robot heartbeat timestamp is updated
- the decoded feedback packet is stored in `Events.rf`
- fields such as `has_ball`, battery voltage, kicker readiness, and acting
  status become available to game logic and metrics

### Robot Command Output

`src/communication/robot_sender.rs` sends one `CpRobot` protobuf packet to each
configured robot at:

```text
robot.ip:server.robots_port
```

Sending is best-effort per robot. A failure for one robot does not stop sends to
other robots.

A robot is considered reachable only if its latest heartbeat is less than
100 ms old. If the heartbeat is older, the sender reports the robot as
unreachable and does not send a command packet to it.

## World Model

`src/game_logic/types.rs` defines the internal match model:

- `WorldState` stores own robots, opponent robots, ball data, referee data,
  interface commands, active goalie, defenders, and current phase.
- `Robot` stores robot position, velocity, orientation, angular velocity,
  distance to the ball, distance to teammates/opponents, and wall distances.
- `BallData` stores current ball position/velocity and kicked-ball stop
  prediction when the tracker provides it.
- `RefMachine` maps SSL referee commands into a smaller internal state machine.
- `GamePhase` describes controller phases such as halted, stopped, kickoff,
  penalty, free kick, running, timeout, and ball placement.

Coordinates and velocities are handled in SSL field units:

- positions: millimeters
- linear velocities: millimeters per second
- orientations: degrees
- angular velocities: degrees per second

Field dimensions come from SSL-Vision geometry when available. Before geometry
arrives, `FieldSetup::default()` assumes a 9000 mm by 6000 mm field.

## Controller Modes

The active mode comes from `InterfaceCommandCp.mode`.

### Manual Mode

Implemented in `src/game_logic/mode_manual.rs`.

Manual mode copies per-robot commands received from the WebSocket interface
directly into the outgoing robot command packets. If manual GameController
control is enabled by the interface and the referee command is halt or stop,
the robot command state is overridden with that referee state.

Use this mode for operator-driven movement and direct robot debugging.

### Game Mode

Implemented in `src/game_logic/mode_game.rs`.

Game mode is the main autonomous match mode. It uses `WorldState.phase` to
decide how to command the team:

- halted and stopped phases set all robots to halt/stop
- kickoff, penalty, and free-kick phases keep the goalie in goalie state
- timeout sends robots to configured substitution positions
- ball placement assigns the closest own robot to place the ball
- running phase sets the goalie, handles goalie possession, and otherwise calls
  the AI handler

When the goalie has the ball, CrashPilot tries to chip to the farthest teammate.
If no good teammate target exists, it uses the goal-shooting helper.

With the `ssl_game_controller` feature enabled, a changed goalkeeper selection
causes CrashPilot to send a `desired_keeper` request to the GameController team
protocol.

### Test Mode

Implemented in `src/game_logic/mode_test.rs`.

Test mode provides hardware and behavior tests selected from the interface:

- `TestNone`: halt selected robots
- `TestBallControl`: run dribbler/ball-control behavior
- `TestDribbler`: move selected robots to changing target positions with ball
  positioning
- `TestKicker`: with one robot, approach the ball; with two robots, alternate
  kick and receive behavior based on ball possession

If no robot IDs are selected by the interface, test mode applies to all
configured robots.

## AI Integration

The AI implementation lives in `crates/artificial_incompetence`.

CrashPilot stores AI-facing state in
`artificial_incompetence::types::GameState`. `update_ai_data()` normalizes robot
and ball data by field dimensions before inference.

`src/game_logic/ai_handler.rs` calls:

```rust
let commands = cp.ai.predict(&cp.ai_data, 1f32);
```

It then converts AI `RobotCommand` values into low-level `CpCommand` tasks:

| AI command | Robot task/action |
| --- | --- |
| `Pos` | `TaskPos` |
| `Kick` | `TaskKick` |
| `Chip` | kick/chip command path |
| `RecKick` / `RecPass` | `TaskRecKick` |
| `Steal` | `TaskSteal` |
| `Dribble` | `TaskDribble` |
| `PosBall` | `TaskPosBall` |
| `KickGoal` | best-angle shot helper |
| `PassTo` | kick toward selected teammate |
| `GoalWall` | add robot to defenders |
| `GoalieGuard` | `TaskBlock` |
| `Hold` | zero speed |

The goalie is excluded from normal AI command application when a goalie ID is
known.

## Observability

### Prometheus

Enable with:

```sh
cargo run --features prometheus
```

The metrics server listens on:

```text
http://<logging.prometheus_host>:<logging.prometheus_port>
```

Endpoints:

- `/metrics`: Prometheus text exposition
- `/health`: simple health check

Metrics include registered robots, feedback presence, battery voltage, current,
kicker readiness, ball possession, error/acting flags, packet counters, send
success/failure counters, and tracked robot velocities.

Example Grafana dashboards are stored in `grafana/`.

### Loki

Enable with:

```sh
cargo run --features loki
```

Outbound robot command packets are batched and pushed to:

```text
http://<logging.loki_host>:<logging.loki_port>/loki/api/v1/push
```

Logs are labeled with:

- `app="CrashPilot"`
- `direction="outbound"`
- `robot_id="<id>"`

## GameController Team Protocol

Enable with:

```sh
cargo run --features ssl_game_controller
```

CrashPilot connects to `ssl.ssl_gc_msg_ip:ssl.ssl_gc_msg_port`, registers as:

```text
Robocup Junior SSL Team
```

The handler supports:

- desired keeper requests
- advantage choice
- substitution requests
- ping

Replies from the GameController team protocol are stored in
`Events.gc_team_messages`.

## Development Commands

Format:

```sh
cargo fmt
```

Check:

```sh
cargo check
```

Test:

```sh
cargo test
```

Build optimized:

```sh
cargo build --release
```

Cross-build examples depend on the local cross toolchain and target setup. This
repository has previously been used with:

```sh
cross build --target aarch64-unknown-linux-gnu --release
```

## Current Implementation Notes

- The controller loop comment says `~500 Hz`, but the configured 8 ms interval
  is approximately 125 Hz.
- `WorldState::update_states()` maps referee commands into `GamePhase`, then
  currently forces `self.phase = GamePhase::Running` at the end. That means the
  live game-mode behavior always executes the running branch until that testing
  override is removed.
- Robot UDP sending depends on recent feedback heartbeats. If robots are not
  sending `RobotCp` feedback to `robot_receive_port`, CrashPilot will mark them
  unreachable and skip command sends.
- The outbound WebSocket and event handling intentionally keep the latest state
  rather than a full history. This is good for real-time control, but it means
  packet-by-packet replay needs a separate logger.
- `config.toml` may contain operational comments or extra keys. The Rust config
  structs deserialize only the fields defined in `src/config.rs`.

## Typical Match Startup Checklist

1. Confirm network interface and multicast routing are correct for SSL-Vision
   and the GameController.
2. Confirm `config.toml` robot IPs match the robots on the field.
3. Confirm each robot sends feedback to `server.robot_receive_port`.
4. Start SSL-Vision and the SSL GameController.
5. Start CrashPilot with the required features.
6. Open/connect the operator interface to the WebSocket endpoint.
7. Select team color, field side, goalie, and controller mode in the interface.
8. Verify robot heartbeats, vision tracking, and referee state before enabling
   autonomous game behavior.
