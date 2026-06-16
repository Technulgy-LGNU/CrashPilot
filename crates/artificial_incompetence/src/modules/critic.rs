use crate::ai_types::MultiBatch;
use crate::config::{BALL_FEATURES, ROBOT_FEATURES};
use crate::grid::GridSpec;
use crate::modules::grid::GridZoneEncoder;
use crate::modules::mlp::MLP;
use crate::modules::nn::{TransformerEncoder, ValueHead};
use tch::nn::Module;
use tch::{Kind, Tensor, nn};

pub struct Critic {
  own_encoder: MLP,
  opp_encoder: MLP,
  ball_encoder: MLP,
  grid_encoder: GridZoneEncoder,
  encoder: TransformerEncoder,
  cls: Tensor,
  value_head: ValueHead,
  d_model: i64,
}

impl Critic {
  pub fn new(
    vs: &nn::Path,
    grid_spec: GridSpec,
    d_model: i64,
    hidden_dim: i64,
    num_heads: i64,
    num_layers: i64,
  ) -> Self {
    let own_encoder = MLP::new(&(vs / "own_encoder"), ROBOT_FEATURES, d_model, d_model);
    let opp_encoder = MLP::new(&(vs / "opp_encoder"), ROBOT_FEATURES, d_model, d_model);
    let ball_encoder = MLP::new(&(vs / "ball_encoder"), BALL_FEATURES, d_model, d_model);
    let grid_encoder = GridZoneEncoder::new(&(vs / "grid_encoder"), grid_spec, d_model);
    let encoder = TransformerEncoder::new(
      &(vs / "encoder"),
      num_layers,
      d_model,
      hidden_dim,
      num_heads,
    );
    let cls = vs.randn("cls", &[1, 1, d_model], 0.0, 0.02);
    let value_head = ValueHead::new(&(vs / "value_head"), d_model, hidden_dim);

    Self {
      own_encoder,
      opp_encoder,
      ball_encoder,
      grid_encoder,
      encoder,
      cls,
      value_head,
      d_model,
    }
  }

  pub fn forward(&self, batch: &MultiBatch) -> Tensor {
    let own_e = self.own_encoder.forward(&batch.own);
    let opp_e = self.opp_encoder.forward(&batch.opp);
    let ball_e = self.ball_encoder.forward(&batch.ball).unsqueeze(1);

    let b = batch.own.size()[0];
    let device = batch.own.device();

    let zone_e = self.grid_encoder.forward(b, device);
    let zone_mask = self.grid_encoder.mask(b, device);

    let cls = self.cls.expand([b, 1, self.d_model], true);
    let cls_mask = Tensor::ones([b, 1], (Kind::Bool, device));
    let ball_mask = Tensor::ones([b, 1], (Kind::Bool, device));

    let tokens = Tensor::cat(&[cls, own_e, opp_e, ball_e, zone_e], 1);
    let mask = Tensor::cat(
      &[
        cls_mask,
        batch.own_mask.shallow_clone(),
        batch.opp_mask.shallow_clone(),
        ball_mask,
        zone_mask,
      ],
      1,
    );

    let enc = self.encoder.forward(&tokens, &mask);
    self.value_head.forward(&enc.select(1, 0))
  }
}
