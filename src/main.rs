#[tokio::main]
async fn main() {
  let mut cp = crashpilot::CrashPilot::default().await;

  cp.run().await
}
