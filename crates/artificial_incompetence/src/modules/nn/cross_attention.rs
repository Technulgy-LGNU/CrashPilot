use crate::modules::nn::multi_head_attention::MultiHeadAttention;
use tch::{Tensor, nn};

pub struct CrossAttentionBlock {
  query_norm: nn::LayerNorm,
  context_norm: nn::LayerNorm,
  attn: MultiHeadAttention,
}

impl CrossAttentionBlock {
  pub fn new(vs: &nn::Path, d_model: i64, num_heads: i64) -> Self {
    let query_norm = nn::layer_norm(vs / "query_norm", vec![d_model], Default::default());
    let context_norm = nn::layer_norm(vs / "context_norm", vec![d_model], Default::default());
    let attn = MultiHeadAttention::new(&(vs / "attn"), d_model, num_heads);

    Self {
      query_norm,
      context_norm,
      attn,
    }
  }

  pub fn forward(&self, query: &Tensor, context: &Tensor, context_mask: &Tensor) -> Tensor {
    self.attn.forward(
      &query.apply(&self.query_norm),
      &context.apply(&self.context_norm),
      &context.apply(&self.context_norm),
      context_mask,
    )
  }
}
