mod data;
mod gae;
mod ppo;
mod reward;
mod stages;
mod trainer;
mod transition;

pub use ppo::UpdateResult;
pub use reward::RewardMode;
pub use stages::*;
use std::path::Path;
use tch::Device;
pub use trainer::*;

pub fn train(
  epochs: usize,
  worlds: usize,
  model_path: &Path,
  dev: Device,
) -> TrainResult<Vec<TrainingReport>> {
  let opts = TrainOptions {
    updates: epochs,
    worlds,
    model_path: Some(model_path.to_path_buf()),
    device: dev,
    ..Default::default()
  };
  train_all_stages(opts)
}

pub fn train_stage(
  stage: TrainingStage,
  epochs: usize,
  worlds: usize,
  model_path: &Path,
  dev: Device,
) -> TrainResult<TrainingReport> {
  let opts = TrainOptions {
    updates: epochs,
    worlds,
    model_path: Some(model_path.to_path_buf()),
    device: dev,
    ..Default::default()
  };
  train_single_stage(stage, opts)
}
