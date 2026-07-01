use crate::GameStateExt;
use crate::grid::GridSpec;
use crate::modules::coach::Coach;
use core_dump::types::{Ai, Commands, GameState};
use std::env;
use std::error::Error;
use std::path::{Path, PathBuf};
use tch::nn::VarStore;
use tch::{Device, no_grad};

pub type InferenceResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

pub struct ArtificialIncompetence {
  _vs: VarStore,
  coach: Coach,
  deterministic: bool,
}

impl ArtificialIncompetence {
  pub fn new(device: Device) -> Self {
    let vs = VarStore::new(device);
    let coach = Coach::new(&vs.root(), GridSpec::default_ssl());

    Self {
      _vs: vs,
      coach,
      deterministic: true,
    }
  }

  pub fn load<P: AsRef<Path>>(path: P, device: Device) -> InferenceResult<Self> {
    let model_path = resolve_model_path(path.as_ref())?;
    let mut ai = Self::new(device);
    ai._vs.load(model_path)?;
    Ok(ai)
  }

  pub fn load_auto<P: AsRef<Path>>(path: P) -> InferenceResult<Self> {
    let device = Device::cuda_if_available();

    let model_path = resolve_model_path(path.as_ref())?;
    let mut ai = Self::new(device);
    ai._vs.load(model_path)?;
    Ok(ai)
  }

  pub fn load_cpu<P: AsRef<Path>>(path: P) -> InferenceResult<Self> {
    Self::load(path, Device::Cpu)
  }

  pub fn from_env_or_default() -> InferenceResult<Self> {
    let device = device_from_env();

    if let Ok(path) = env::var("CRASHPILOT_AI_CHECKPOINT") {
      return Self::load(path, device);
    }

    if let Ok(path) = env::var("CRASHPILOT_AI_MODEL") {
      return Self::load(path, device);
    }

    Ok(Self::new(device))
  }

  pub fn set_deterministic(&mut self, deterministic: bool) {
    self.deterministic = deterministic;
  }
}

impl Default for ArtificialIncompetence {
  fn default() -> Self {
    {
      Self::from_env_or_default().unwrap_or_else(|err| {
        panic!("failed to initialize ArtificialIncompetence: {err}");
      })
    }
  }
}

impl Ai for ArtificialIncompetence {
  fn predict(&mut self, state: &GameState, _dt: f32) -> Commands {
    {
      let batch = <GameState as GameStateExt>::encode_multiple(&[*state], self._vs.device());
      let (_, _, _, mut plans) = no_grad(|| self.coach.act(&batch, self.deterministic));

      plans.pop().unwrap_or_default()
    }
  }
}

fn resolve_model_path(path: &Path) -> InferenceResult<PathBuf> {
  if path.is_file() {
    return Ok(path.to_path_buf());
  }

  let direct_model = path.join("model.safetensors");
  if direct_model.is_file() {
    return Ok(direct_model);
  }

  latest_checkpoint(path)?
    .map(|checkpoint| checkpoint.join("model.safetensors"))
    .ok_or_else(|| {
      format!(
        "no model.safetensors or checkpoint_* directory found in {}",
        path.display()
      )
      .into()
    })
}

fn latest_checkpoint(path: &Path) -> InferenceResult<Option<PathBuf>> {
  let Ok(entries) = std::fs::read_dir(path) else {
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
        && path.join("model.safetensors").is_file()
    })
    .collect::<Vec<_>>();

  checkpoints.sort();
  Ok(checkpoints.pop())
}

fn device_from_env() -> Device {
  match env::var("CRASHPILOT_AI_DEVICE") {
    Ok(device) if device.eq_ignore_ascii_case("cuda") => Device::Cuda(0),
    Ok(device) if device.to_ascii_lowercase().starts_with("cuda:") => device
      .split_once(':')
      .and_then(|(_, index)| index.parse::<usize>().ok())
      .map(Device::Cuda)
      .unwrap_or(Device::Cuda(0)),
    _ => Device::Cpu,
  }
}
