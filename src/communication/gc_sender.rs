pub enum GCTask {
  Substitution,
  Goalie,
}

pub fn gc_sender(_robot_id: u16, _task: GCTask) -> anyhow::Result<()> {
  Ok(())
}
