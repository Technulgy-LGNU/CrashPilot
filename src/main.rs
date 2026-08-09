#[tokio::main]
pub async fn main() {
  println!("Starting Crashpilot V2");

  let mut cp = crashpilot::CrashPilot::default().await;
  
  cp.run().await;
}
