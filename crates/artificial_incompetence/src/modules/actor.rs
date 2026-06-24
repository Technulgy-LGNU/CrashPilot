use crate::ai_types::{MultiBatch, NUM_COMMANDS};
use crate::config::{BALL_FEATURES, NEG_INF, NUM_POWER_BINS, ROBOT_FEATURES};
use crate::grid::GridSpec;
use crate::mask::{build_action_masks, Masks};
use crate::modules::grid::GridZoneEncoder;
use crate::modules::mlp::MLP;
use crate::modules::nn::{CategoricalHead, CrossAttentionBlock, PointerHead};
use tch::nn::Module;
use tch::{nn, Kind, Tensor};

pub struct Actor {
  own_encoder: MLP,
  opp_encoder: MLP,
  ball_encoder: MLP,
  grid_encoder: GridZoneEncoder,
  cross_attn: CrossAttentionBlock,
  body: PolicyBody,
  command_head: CategoricalHead,
  teammate_head: PointerHead,
  zone_head: PointerHead,
  power_head: CategoricalHead,
}

impl Actor {
  pub fn new(
    vs: &nn::Path,
    grid_spec: GridSpec,
    d_model: i64,
    hidden_dim: i64,
    num_heads: i64,
  ) -> Self {
    let own_encoder = MLP::new(&(vs / "own_encoder"), ROBOT_FEATURES, d_model, d_model);
    let opp_encoder = MLP::new(&(vs / "opp_encoder"), ROBOT_FEATURES, d_model, d_model);
    let ball_encoder = MLP::new(&(vs / "ball_encoder"), BALL_FEATURES, d_model, d_model);
    let grid_encoder = GridZoneEncoder::new(&(vs / "grid_encoder"), grid_spec, d_model);
    let cross_attn = CrossAttentionBlock::new(&(vs / "cross_attn"), d_model, num_heads);
    let body = PolicyBody::new(&(vs / "body"), 2 * d_model, hidden_dim);
    let command_head =
      CategoricalHead::new(&(vs / "command_head"), hidden_dim, NUM_COMMANDS as i64);
    let teammate_head = PointerHead::new(&(vs / "teammate_head"), hidden_dim, d_model);
    let zone_head = PointerHead::new(&(vs / "zone_head"), hidden_dim, d_model);
    let power_head = CategoricalHead::new(&(vs / "power_head"), hidden_dim, NUM_POWER_BINS);

    Self {
      own_encoder,
      opp_encoder,
      ball_encoder,
      grid_encoder,
      cross_attn,
      body,
      command_head,
      teammate_head,
      zone_head,
      power_head,
    }
  }

  pub fn forward_raw(&self, batch: &MultiBatch) -> ActorRawOutput {
    let own_e = self.own_encoder.forward(&batch.own);
    let opp_e = self.opp_encoder.forward(&batch.opp);
    let ball_e = self.ball_encoder.forward(&batch.ball).unsqueeze(1);

    let b = batch.own.size()[0];
    let device = batch.own.device();

    let zone_e = self.grid_encoder.forward(b, device);
    let zone_mask = self.grid_encoder.mask(b, device);
    let ball_mask = Tensor::ones([b, 1], (Kind::Bool, device));

    let context = Tensor::cat(
      &[
        own_e.shallow_clone(),
        opp_e.shallow_clone(),
        ball_e,
        zone_e.shallow_clone(),
      ],
      1,
    );

    let context_mask = Tensor::cat(
      &[
        batch.own_mask.shallow_clone(),
        batch.opp_mask.shallow_clone(),
        ball_mask,
        zone_mask,
      ],
      1,
    );

    let attended = self.cross_attn.forward(&own_e, &context, &context_mask);
    let robot_hidden = self
      .body
      .forward(&Tensor::cat(&[own_e.shallow_clone(), attended], -1));

    let command_logits = self.command_head.forward(&robot_hidden);
    let teammate_logits = self.teammate_head.forward(&robot_hidden, &own_e);
    let zone_logits = self.zone_head.forward(&robot_hidden, &zone_e);
    let power_logits = self.power_head.forward(&robot_hidden);

    ActorRawOutput {
      command_logits,
      teammate_logits,
      zone_logits,
      power_logits,
      robot_hidden,
      zone_tokens: zone_e,
    }
  }

  pub fn forward(&self, batch: &MultiBatch) -> ActorOutput {
    let raw = self.forward_raw(batch);
    let masks = build_action_masks(batch);

    ActorOutput {
      command_logits: masked_logits(&raw.command_logits, &masks.action_mask),
      teammate_logits: masked_logits(&raw.teammate_logits, &masks.teammate_mask),
      zone_logits: raw.zone_logits,
      power_logits: raw.power_logits,
      robot_hidden: raw.robot_hidden,
      zone_tokens: raw.zone_tokens,
      masks,
    }
  }
}

pub struct ActorRawOutput {
  command_logits: Tensor,
  teammate_logits: Tensor,
  zone_logits: Tensor,
  power_logits: Tensor,
  robot_hidden: Tensor,
  zone_tokens: Tensor,
}

pub struct ActorOutput {
  pub command_logits: Tensor,
  pub teammate_logits: Tensor,
  pub zone_logits: Tensor,
  pub power_logits: Tensor,
  pub robot_hidden: Tensor,
  pub zone_tokens: Tensor,
  pub masks: Masks,
}

pub struct PolicyBody {
  mlp: MLP,
}

impl PolicyBody {
  pub fn new(vs: &nn::Path<'_>, in_dim: i64, hidden_dim: i64) -> Self {
    Self {
      mlp: MLP::new(&(vs / "mlp"), in_dim, hidden_dim, hidden_dim),
    }
  }

  pub fn forward(&self, xs: &Tensor) -> Tensor {
    self.mlp.forward(xs)
  }
}

pub fn masked_logits(logits: &Tensor, mask: &Tensor) -> Tensor {
  let neg = Tensor::full_like(logits, NEG_INF);
  logits.where_self(&mask.to_kind(Kind::Bool), &neg)
}
