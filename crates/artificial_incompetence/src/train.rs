use std::cell::UnsafeCell;
use std::sync::Arc;
use crate::types::{Ai, Commands, RobotCommand};

pub struct ArtificialTrainer {
    pub id: usize,
    pub data: Arc<UnsafeCell<Vec<Commands>>>
}

impl Ai for ArtificialTrainer {
    fn predict(&mut self, _state: &crate::types::GameState, _dt: f32) -> Commands {
        //TODO: we need somehow get state and dt to the trainer.

        let mut data = unsafe { &**self.data.get() };

        data[self.id]
    }
}