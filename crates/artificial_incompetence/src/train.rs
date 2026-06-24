mod ppo;
mod data;
mod transition;
mod reward;
mod gae;
mod trainer;

use tch::Device;
use tch::nn::{Path, VarStore};
pub use trainer::*;



pub fn train(epochs: usize, worlds: usize, model_path: &Path, dev: Device) {
    let var_store = VarStore::fill_safetensors()

}