use crate::modules::mlp::MLP;

pub struct Critic {
    own_encoder: MLP,
    opp_encoder: MLP,
    ball_encoder: MLP,
}