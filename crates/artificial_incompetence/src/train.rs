use crate::types::{Ai, Commands};
mod ppo;

use std::cell::UnsafeCell;
use std::sync::Arc;

pub struct ArtificialTrainer {
  pub id: usize,
  pub data: Arc<UnsafeCell<Vec<Commands>>>,
}

impl Ai for ArtificialTrainer {
  fn predict(&mut self, _state: &crate::types::GameState, _dt: f32) -> Commands {
    // state is already submitted in the first step during training!

    let mut data = unsafe { &**self.data.get() };

    data[self.id]
  }
}
