mod ppo;
mod data;
mod buffer;
mod transition;
mod reward;
mod gae;

use std::cell::UnsafeCell;
use std::mem;
use std::mem::MaybeUninit;
use std::sync::Arc;
use simhark::WorldState;
use tch::{Device, Tensor};
use tch::nn::{Optimizer, OptimizerConfig, VarStore};
use crate::ai_types::SampledRobotAction;
use crate::GameState;
use crate::grid::GridSpec;
use crate::modules::coach::Coach;
use crate::train::buffer::RolloutBuffer;
use crate::train::data::{empty_world_state, Data, RootData};
use crate::train::reward::compute_reward;
use crate::train::transition::Transition;
use crate::types::{Ai, Commands, RobotCommand};




pub struct Trainer {
    data: RootData,
    pub vs: VarStore,
    pub policy: Coach,
    pub opt: Optimizer,
    pub buf: Vec<Transition>,
    pub dev: Device,
    old_state: Vec<WorldState>,
    values: Vec<f64>,
    sampled: SampledRobotAction,
    log_prob: Tensor,

}

impl Trainer {
    pub fn new(vs: VarStore, num_worlds: usize) -> Self {
        let policy = Coach::new(&vs.root(), GridSpec::default_ssl());

        let dev = vs.device();

        let opt = tch::nn::Adam::default().build(&vs, 1e-3).unwrap();

        let mut old_state = Vec::with_capacity(num_worlds);

        for i in 0..num_worlds {
            old_state.push(empty_world_state(i))

        }


        Self {
            data: RootData::new(num_worlds),
            vs,
            policy,
            opt,
            buf: Vec::new(),
            dev,
            old_state,
            values: Vec::new(),
            sampled: SampledRobotAction::default()
            log_prob: Tensor::new(),
        }
    }

    pub fn get_trainer(&self, id: usize) -> ArtificialTrainer {
        ArtificialTrainer {
            id,
            data: self.data.read(),
        }
    }

    pub fn step(&mut self, states: &[GameState], dt: f32) {
        let batch = GameState::encode_multiple(states, self.dev);

        let (sampled, log_prob, value, plans) = tch::no_grad(|| self.policy.act(&batch, false));

        self.data.set_from(&plans);

        self.values = (0..states.len()).map(|i| value.double_value(&[i as i64, 0])).collect::<Vec<_>>();
        self.log_prob = log_prob;

    }

    pub fn finish_step(&mut self, states: Vec<WorldState>) {
        let mut rewards = Vec::with_capacity(states.len());

        for (id, state) in states.iter().enumerate() {
            let old = &self.old_state[id];
            let commands = self.data.get(id);
            rewards.push(compute_reward(old, state, commands))
        }

        let obs = mem::replace(&mut self.old_state, states);

        self.buf.push(Transition {
            obs,
            command_type: self.sampled.command_type.shallow_clone(),
            target_robot: self.sampled.target_robot.shallow_clone(),
            target_zone: self.sampled.target_zone.shallow_clone(),
            power_bin: self.sampled.power_bin.shallow_clone(),
            log_prob: self.log_prob.shallow_clone(),
            value: mem::take(&mut self.values),
            reward: rewards,
        })
    }

    pub fn finish_rollout(&mut self, states: &[GameState]) {
        let last_values = tch::no_grad(|| {
            let batch = GameState::encode_multiple(states, self.dev);
            let v = self.policy.critic.forward(&batch);
            (0..states.len()).map(|i| v.double_value(&[i as i64, 0])).collect::<Vec<_>>()
        });

    }
}



pub struct ArtificialTrainer {
    pub id: usize,
    pub data: Data,
}

impl Ai for ArtificialTrainer {
  fn predict(&mut self, _state: &crate::types::GameState, _dt: f32) -> Commands {
    // state is already submitted in the first step during training!
    self.data.get(self.id)
  }
}
