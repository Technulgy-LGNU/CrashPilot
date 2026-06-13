use tch::{nn, Tensor};
use crate::modules::nn::feed_forward::FeedForward;
use crate::modules::nn::multi_head_attention::MultiHeadAttention;


pub struct TransformerEncoder {
    blocks: Vec<TransformerEncoderBlock>,
}

impl TransformerEncoder {
    pub fn new(
        vs: &nn::Path<'_>,
        num_layers: i64,
        d_model: i64,
        hidden_dim: i64,
        num_heads: i64,
    ) -> Self {
        let mut blocks = Vec::new();
        for i in 0..num_layers {
            blocks.push(TransformerEncoderBlock::new(
                &(vs / format!("block_{i}")),
                d_model,
                hidden_dim,
                num_heads,
            ));
        }
        Self { blocks }
    }

    pub fn forward(&self, tokens: &Tensor, mask: &Tensor) -> Tensor {
        let mut x = tokens.shallow_clone();

        for block in &self.blocks {
            x = block.forward(&x, mask);
        }

        x
    }
}

pub struct TransformerEncoderBlock {
    attn_norm: nn::LayerNorm,
    attn: MultiHeadAttention,
    ff_norm: nn::LayerNorm,
    ff: FeedForward,
}

impl TransformerEncoderBlock {
    pub fn new(vs: &nn::Path, d_model: i64, hidden_dim: i64, num_heads: i64) -> Self {
        let attn_norm = nn::layer_norm(vs / "attn_norm", vec![d_model], Default::default());
        let attn = MultiHeadAttention::new(&(vs / "attn"), d_model, num_heads);
        let ff_norm = nn::layer_norm(vs / "ff_norm", vec![d_model], Default::default());
        let ff = FeedForward::new(&(vs / "ff"), d_model, hidden_dim);

        Self {
            attn_norm,
            attn,
            ff_norm,
            ff,
        }
    }

    pub fn forward(&self, tokens: &Tensor, mask: &Tensor) -> Tensor {
        let normed_tokens = tokens.apply(&self.attn_norm);
        
        let x = tokens + self.attn.forward(
            &normed_tokens,
            &normed_tokens,
            &normed_tokens,
            mask,
        );

        let y = x.apply(&self.ff_norm);

        x + self.ff.forward(&y)
    }
}
