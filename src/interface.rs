use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::thread;

use webinterface_assets::embedded_assets;
use webinterface_core::{InterfaceConfig, InterfaceHost};
use webinterface_crashpilot_bridge::{
  CrashPilotAdapter, LegacyBridgeConfig, run_legacy_controller,
};
use webinterface_protocol::{SessionKind, SessionLifecycle};

/// Start the Rust operator-interface host and compatibility bridge on a
/// dedicated runtime. The returned thread handle keeps the host lifecycle
/// owned by the `CrashPilot` instance instead of using process-global state.
pub fn spawn_interface(
  crashpilot_websocket_url: String,
) -> std::io::Result<thread::JoinHandle<()>> {
  thread::Builder::new()
    .name("crashpilot-interface-owner".into())
    .spawn(move || {
      let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("failed to create CrashPilot interface runtime");
      runtime.block_on(async move {
        let bind_address = std::env::var("FAABS_INTERFACE_BIND")
          .ok()
          .and_then(|value| value.parse::<SocketAddr>().ok())
          .unwrap_or_else(|| SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 8080));
        let (_guard, handle) = InterfaceHost::start(InterfaceConfig {
          bind_address,
          assets: embedded_assets(),
          ..InterfaceConfig::default()
        })
        .expect("failed to start Rust CrashPilot interface");
        let session = handle.create_session(
          "CrashPilot live",
          SessionKind::LiveMatch,
          true,
          vec!["crashpilot".into()],
          1,
        );
        handle
          .update_session(session.id, SessionLifecycle::Running, None)
          .expect("failed to start CrashPilot interface session");
        let adapter = CrashPilotAdapter::register(&handle, session.id)
          .expect("failed to register CrashPilot interface adapter");
        println!("CrashPilot interface: {}", handle.http_url());
        if let Err(error) = run_legacy_controller(
          adapter,
          LegacyBridgeConfig {
            websocket_url: crashpilot_websocket_url,
            ..LegacyBridgeConfig::default()
          },
        )
        .await
        {
          eprintln!("CrashPilot interface bridge stopped: {error}");
        }
      });
    })
}
