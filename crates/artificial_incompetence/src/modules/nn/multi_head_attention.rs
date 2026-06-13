use tch::{nn, Kind, Tensor};
use crate::config::NEG_INF;

pub struct MultiHeadAttention {
    q_proj: nn::Linear,
    k_proj: nn::Linear,
    v_proj: nn::Linear,
    o_proj: nn::Linear,
    d_model: i64,
    num_heads: i64,
    head_dim: i64,
}

impl MultiHeadAttention {
    pub fn new(vs: &nn::Path, d_model: i64, num_heads: i64) -> Self {
        assert!(d_model % num_heads == 0);
        let q_proj = nn::linear(vs / "q_proj", d_model, d_model, Default::default());
        let k_proj = nn::linear(vs / "k_proj", d_model, d_model, Default::default());
        let v_proj = nn::linear(vs / "v_proj", d_model, d_model, Default::default());
        let o_proj = nn::linear(vs / "o_proj", d_model, d_model, Default::default());

        Self {
            q_proj,
            k_proj,
            v_proj,
            o_proj,
            d_model,
            num_heads,
            head_dim: d_model / num_heads,
        }
    }

    pub fn forward(
        &self,
        query: &Tensor,
        key: &Tensor,
        value: &Tensor,
        key_mask: &Tensor, // [B, K]
    ) -> Tensor {
        let b = query.size()[0];
        let q_len = query.size()[1];
        let k_len = key.size()[1];

        let q = query
            .apply(&self.q_proj)
            .view([b, q_len, self.num_heads, self.head_dim])
            .transpose(1, 2);

        let k = key
            .apply(&self.k_proj)
            .view([b, k_len, self.num_heads, self.head_dim])
            .transpose(1, 2);

        let v = value
            .apply(&self.v_proj)
            .view([b, k_len, self.num_heads, self.head_dim])
            .transpose(1, 2);

        let scale = (self.head_dim as f64).sqrt();
        let scores = q.matmul(&k.transpose(-2, -1)) / scale;

        let expanded_mask = key_mask
            .unsqueeze(1)
            .unsqueeze(1)
            .expand([b, self.num_heads, q_len, k_len], true);

        let neg = Tensor::full_like(&scores, NEG_INF);
        let scores = expanded_mask.where_self(&scores, &neg);

        let attn = scores.softmax(-1, Kind::Float);

        attn.matmul(&v)
            .transpose(1, 2)
            .contiguous()
            .view([b, q_len, self.d_model])
            .apply(&self.o_proj)
    }
}
