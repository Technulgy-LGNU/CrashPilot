#[tokio::main]
async fn main() {
  let mut cp = CrashPilot::CrashPilot::default().await;

  cp.run().await
}
