use std::path::{Path, PathBuf};

#[cfg(feature = "train")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "train")]
use std::fs;
#[cfg(feature = "train")]
use tch::Device;
#[cfg(feature = "train")]
use tch::nn::VarStore;

#[cfg(feature = "train")]
use crate::{RewardMode, TrainOptions, TrainResult, Trainer, TrainingStage, UpdateResult};

#[derive(Debug, Clone)]
pub struct ModelLoader {
  checkpoint_dir: PathBuf,
}

impl ModelLoader {
  pub fn new<P: Into<PathBuf>>(checkpoint_dir: P) -> Self {
    Self {
      checkpoint_dir: checkpoint_dir.into(),
    }
  }

  pub fn checkpoint_dir(&self) -> &Path {
    &self.checkpoint_dir
  }

  #[cfg(feature = "train")]
  pub fn fresh_trainer(
    &self,
    device: Device,
    worlds: usize,
    learning_rate: f64,
    reward_mode: RewardMode,
  ) -> Trainer {
    let vs = VarStore::new(device);
    Trainer::new_with_reward(vs, worlds, learning_rate, reward_mode)
  }

  #[cfg(feature = "train")]
  pub fn load_model_from_safetensors<P: AsRef<Path>>(
    &self,
    model_path: P,
    device: Device,
    worlds: usize,
    learning_rate: f64,
    reward_mode: RewardMode,
  ) -> TrainResult<Trainer> {
    let mut trainer = self.fresh_trainer(device, worlds, learning_rate, reward_mode);
    trainer.vs.load(model_path)?;
    Ok(trainer)
  }

  #[cfg(feature = "train")]
  pub fn save_model_safetensors<P: AsRef<Path>>(
    &self,
    trainer: &Trainer,
    model_path: P,
  ) -> TrainResult<()> {
    trainer.vs.save(model_path)?;
    Ok(())
  }

  #[cfg(feature = "train")]
  pub fn save_checkpoint(
    &self,
    trainer: &Trainer,
    metadata: &CheckpointMetadata,
  ) -> TrainResult<PathBuf> {
    let dir = self
      .checkpoint_dir
      .join(format!("checkpoint_{:09}", metadata.update));
    fs::create_dir_all(&dir)?;

    trainer.vs.save(dir.join("model.safetensors"))?;
    fs::write(dir.join("meta.json"), serde_json::to_vec_pretty(metadata)?)?;

    Ok(dir)
  }

  #[cfg(feature = "train")]
  pub fn load_checkpoint<P: AsRef<Path>>(
    &self,
    checkpoint_dir: P,
    device: Device,
  ) -> TrainResult<(Trainer, CheckpointMetadata)> {
    let checkpoint_dir = checkpoint_dir.as_ref();
    let metadata: CheckpointMetadata =
      serde_json::from_slice(&fs::read(checkpoint_dir.join("meta.json"))?)?;

    let mut trainer = self.fresh_trainer(
      device,
      metadata.worlds,
      metadata.learning_rate,
      metadata.stage.reward_mode(),
    );
    trainer.vs.load(checkpoint_dir.join("model.safetensors"))?;

    Ok((trainer, metadata))
  }

  #[cfg(feature = "train")]
  pub fn latest_checkpoint(&self) -> TrainResult<Option<PathBuf>> {
    let Ok(entries) = fs::read_dir(&self.checkpoint_dir) else {
      return Ok(None);
    };

    let mut checkpoints = entries
      .filter_map(Result::ok)
      .map(|entry| entry.path())
      .filter(|path| {
        path.is_dir()
          && path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name.starts_with("checkpoint_"))
      })
      .collect::<Vec<_>>();

    checkpoints.sort();
    Ok(checkpoints.pop())
  }
}

#[cfg(feature = "train")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CheckpointMetadata {
  pub stage: TrainingStage,
  pub update: usize,
  pub worlds: usize,
  pub rollout_steps: usize,
  pub learning_rate: f64,
  pub loss: Option<f64>,
  pub policy_loss: Option<f64>,
  pub value_loss: Option<f64>,
  pub entropy: Option<f64>,
}

#[cfg(feature = "train")]
impl CheckpointMetadata {
  pub fn from_training(
    stage: TrainingStage,
    update: usize,
    opts: &TrainOptions,
    trainer: &Trainer,
    stats: Option<UpdateResult>,
  ) -> Self {
    Self {
      stage,
      update,
      worlds: opts.worlds,
      rollout_steps: opts.rollout_steps,
      learning_rate: trainer.learning_rate,
      loss: stats.map(|s| s.loss),
      policy_loss: stats.map(|s| s.policy_loss),
      value_loss: stats.map(|s| s.value_loss),
      entropy: stats.map(|s| s.entropy),
    }
  }
}
