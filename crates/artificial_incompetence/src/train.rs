mod data;
mod gae;
mod ppo;
mod reward;
mod trainer;
mod transition;

use tch::nn::Path;
use tch::Device;
pub use trainer::*;

pub fn train(epochs: usize, worlds: usize, model_path: &Path, dev: Device) {
  // let var_store = VarStore::fill_safetensors();
}
