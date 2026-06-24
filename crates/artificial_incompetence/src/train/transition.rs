use simhark::WorldState;
use tch::Tensor;
use crate::GameState;

#[derive(Debug)]
pub struct Transition {
    pub obs: Vec<GameState>,
    pub command_type: Tensor, // [R]
    pub target_robot: Tensor, // [N, R]
    pub target_zone: Tensor,  // [N, R]
    pub power_bin: Tensor,    // [N, R]
    pub log_prob: Tensor,     // [N, R] log-prob under the OLD policy
    pub value: Vec<f64>,           // critic estimate V(s_t)
    pub reward: Vec<f64>,
}
