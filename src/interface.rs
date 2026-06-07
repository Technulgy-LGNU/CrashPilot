// Embed frontend (crashpilot-interface) binary
#[cfg(feature = "interface")]
static GO_BINARY: &[u8] = include_bytes!("../crashpilot-interface");

/// Starts the Crashpilot interface, has to be in the repository as a compiled binary
#[cfg(feature = "interface")]
pub fn spawn_interface() {
  tokio::spawn(async move {
    let path = "./crashpilot-interface";

    fs::write(path, GO_BINARY).expect("Failed to write binary file");

    let mut perms = fs::metadata(path)
      .expect("Failed to read metadata")
      .permissions();

    perms.set_mode(0o755);

    fs::set_permissions(path, perms).expect("Failed to set executable permissions");

    Command::new(path)
      .spawn()
      .expect("Failed to spawn binary")
      .wait()
      .expect("Failed to wait on binary");
  });
}
