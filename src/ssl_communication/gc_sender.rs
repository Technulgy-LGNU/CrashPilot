
pub enum GCTask {
  Substitution,
  Goalie,
}

pub fn gc_sender(robot_id: u16, task: GCTask) -> anyhow::Result<()> {
  Ok(())
}
