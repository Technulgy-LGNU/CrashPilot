use crate::types::{Ai, Commands, GameState};

pub struct ArtificialIncompetence {
  // pub actor: Actor,
  // internal state, if needed
}

impl Default for ArtificialIncompetence {
  fn default() -> Self {
    // todo!()
    ArtificialIncompetence {}
  }
}

impl Ai for ArtificialIncompetence {
  fn predict(&mut self, state: &GameState, dt: f32) -> Commands {
    todo!()
  }
}
