# CrashPilot

This is the controller for the new Robocup SSL team
consisting out of three junior teams:
- Team Faabs (Germany)
- LNX (Slovakia)
- ZG24 (Croatia)

The software is written in rust, using the protobuff api bindings
used by the ssl league.

## Observability

- Prometheus metrics are served from the address configured in `config.logging.prometheus_host` / `config.logging.prometheus_port`
- A ready-to-import Grafana dashboard is available at `grafana/crashpilot-robot-dashboard.json`
- A dedicated Loki dashboard for outbound robot messages is available at `grafana/crashpilot-loki-dashboard.json`


# How to run
The Crashpilot should run on every system without any problems.
Just run following command:
```shell
# Without Loki and Interface
cargo run --release

# With Interface
cargo run --release --features interface

# With Loki && Interface
cargo run --release --features interface loki
```
