mod ppo;
mod data;
mod transition;
mod reward;

use std::cell::UnsafeCell;
use std::sync::Arc;
use tch::nn::{Optimizer, VarStore};
use crate::modules::coach::Coach;
use crate::types::{Ai, Commands, RobotCommand};


pub type Data = Arc<UnsafeCell<Vec<Commands>>>;

pub struct Trainer {
    pub data: Data,
    pub vs: VarStore,
    pub policy: Coach,
    pub opt: Optimizer,

}



pub struct ArtificialTrainer {
    pub id: usize,
    pub data: Data,
}

impl Ai for ArtificialTrainer {
  fn predict(&mut self, _state: &crate::types::GameState, _dt: f32) -> Commands {
    // state is already submitted in the first step during training!

    let mut data = unsafe { &**self.data.get() };

    data[self.id]
  }
}
