use tch::nn;

#[derive(Debug)]
pub struct MLP {
  head: nn::Linear,
  n1: nn::LayerNorm,
  hidden: nn::Linear,
  n2: nn::LayerNorm,
}

impl MLP {
  pub fn new(vs: &nn::Path, in_dim: i64, hidden_dim: i64, out_dim: i64) -> Self {
    let head = nn::linear(vs / "head", in_dim, hidden_dim, Default::default());
    let n1 = nn::layer_norm(vs / "n1", vec![hidden_dim], Default::default());
    let hidden = nn::linear(vs / "hidden", hidden_dim, out_dim, Default::default());
    let n2 = nn::layer_norm(vs / "n2", vec![out_dim], Default::default());

    MLP {
      head,
      n1,
      hidden,
      n2,
    }
  }
}

impl nn::Module for MLP {
  fn forward(&self, xs: &tch::Tensor) -> tch::Tensor {
    let xs = self.head.forward(xs);
    let xs = self.n1.forward(&xs);
    let xs = xs.gelu("none");
    let xs = self.hidden.forward(&xs);
    let xs = self.n2.forward(&xs);

    xs.gelu("none")
  }
}
