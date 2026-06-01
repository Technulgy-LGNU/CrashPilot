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
