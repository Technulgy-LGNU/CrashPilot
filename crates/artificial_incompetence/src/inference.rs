use crate::types::{Ai, Commands, GameState};

#[derive(Default)]
pub struct ArtificialIncompetence {
    // internal state, if needed
}

impl Ai for ArtificialIncompetence {
    fn predict(&mut self, state: &GameState, dt: f32) -> Commands {
        todo!()
    }
}
