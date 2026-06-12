use tch::nn::Linear;
use crate::modules::attention::MultiheadAttention;
use crate::modules::mlp::MLP;

struct Actor {
    own_encoder: MLP,
    opp_encoder: MLP,
    ball_encoder: MLP,
    
    attn: MultiheadAttention,
    
    policy_head: Linear,
    policy_tail: Linear,
    
    action_head: Linear,
    pass_query: Linear,
    zone_query: Linear,
}