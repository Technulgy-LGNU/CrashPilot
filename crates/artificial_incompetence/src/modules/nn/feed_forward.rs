use tch::{nn, Tensor};

pub struct FeedForward {
    l1: nn::Linear,
    l2: nn::Linear,
}

impl FeedForward {
    pub fn new(vs: &nn::Path, d_model: i64, hidden_dim: i64) -> Self {
        let l1 = nn::linear(vs / "l1", d_model, hidden_dim, Default::default());
        let l2 = nn::linear(vs / "l2", hidden_dim, d_model, Default::default());

        Self { l1, l2 }
    }

    pub fn forward(&self, xs: &Tensor) -> Tensor {
        xs.apply(&self.l1)
            .gelu("none")
            .apply(&self.l2)
    }
}
