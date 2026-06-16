use tch::{Tensor, nn};

pub struct PointerHead {
  query_proj: nn::Linear,
}

impl PointerHead {
  pub fn new(vs: &nn::Path, in_dim: i64, key_dim: i64) -> Self {
    let query_proj = nn::linear(vs / "query_proj", in_dim, key_dim, Default::default());

    Self { query_proj }
  }

  pub fn forward(&self, query: &Tensor, keys: &Tensor) -> Tensor {
    query
      .apply(&self.query_proj)
      .matmul(&keys.transpose(-1, -2))
  }
}

pub struct CategoricalHead {
  linear: nn::Linear,
}

impl CategoricalHead {
  pub fn new(vs: &nn::Path, in_dim: i64, out_dim: i64) -> Self {
    let linear = nn::linear(vs / "linear", in_dim, out_dim, Default::default());

    Self { linear }
  }

  pub fn forward(&self, xs: &Tensor) -> Tensor {
    xs.apply(&self.linear)
  }
}

pub struct ValueHead {
  l1: nn::Linear,
  l2: nn::Linear,
}

impl ValueHead {
  pub fn new(vs: &nn::Path, in_dim: i64, hidden_dim: i64) -> Self {
    let l1 = nn::linear(vs / "l1", in_dim, hidden_dim, Default::default());
    let l2 = nn::linear(vs / "l2", hidden_dim, 1, Default::default());

    Self { l1, l2 }
  }

  pub fn forward(&self, xs: &Tensor) -> Tensor {
    xs.apply(&self.l1).gelu("none").apply(&self.l2)
  }
}
