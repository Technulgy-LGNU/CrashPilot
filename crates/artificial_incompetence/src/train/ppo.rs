use crate::Trainer;
use crate::ai_types::{MultiBatch, SampledRobotAction};
use tch::{Kind, Tensor};

pub struct UpdateResult {
  loss: f64,
  policy_loss: f64,
  value_loss: f64,
  entropy: f64,
}

impl Trainer {
  pub(crate) fn ppo_update(
    &mut self,
    batch: &MultiBatch,
    action_batch: &SampledRobotAction,
    old_log_prob: &Tensor,
    returns: &Tensor,
    advantages: &Tensor,
    clip_eps: f64,
    value_coef: f64,
    entropy_coef: f64,
    max_grad_norm: f64,
  ) -> UpdateResult {
    let (new_log_prob, entropy, value) = self.policy.evaluate_actions(batch, action_batch);

    let active = &batch.own_mask;
    let logp_new = new_log_prob.masked_select(active);
    let logp_old = old_log_prob.masked_select(active);
    let adv = advantages.masked_select(active);
    let ent = entropy.masked_select(active);

    let ratio = (&logp_new - &logp_old).exp();
    let surr1 = &ratio * &adv;
    let surr2 = ratio.clamp(1.0 - clip_eps, 1.0 + clip_eps) * &adv;
    let policy_loss = -Tensor::minimum(&surr1, &surr2).mean(Kind::Float);

    let value_loss: Tensor = 0.5
      * (value.squeeze_dim(-1) - returns)
        .pow_tensor_scalar(2.0)
        .mean(Kind::Float);

    let entropy_loss = -ent.mean(Kind::Float);
    let loss: Tensor = &policy_loss + value_coef * &value_loss + entropy_coef * &entropy_loss;

    self.opt.zero_grad();
    loss.backward();
    self.opt.clip_grad_norm(max_grad_norm);
    self.opt.step();

    let loss = loss.double_value(&[]);
    let policy_loss = policy_loss.double_value(&[]);
    let value_loss = value_loss.double_value(&[]);
    let entropy = ent.mean(Kind::Float).double_value(&[]);

    UpdateResult {
      loss,
      policy_loss,
      value_loss,
      entropy,
    }
  }
}
