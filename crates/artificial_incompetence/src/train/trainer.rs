use std::mem;
use simhark::WorldState;
use tch::{Device, Kind, Tensor};
use tch::nn::{Optimizer, OptimizerConfig, VarStore};
use crate::ai_types::SampledRobotAction;
use crate::config::MAX_ROBOTS_PER_TEAM;
use crate::GameState;
use crate::grid::GridSpec;
use crate::modules::coach::Coach;
use crate::train::data::{empty_world_state, Data, RootData};
use crate::train::gae::compute_gae;
use crate::train::ppo::UpdateResult;
use crate::train::reward::compute_reward;
use crate::train::transition::Transition;
use crate::types::{Ai, Commands};

pub struct Trainer {
    data: RootData,
    pub vs: VarStore,
    pub policy: Coach,
    pub opt: Optimizer,
    pub buf: Vec<Transition>,
    pub dev: Device,
    old_state: Vec<GameState>,
    old_sim_state: Vec<WorldState>,
    values: Vec<f64>,
    sampled: SampledRobotAction,
    log_prob: Tensor,

}

impl Trainer {
    pub fn new(vs: VarStore, num_worlds: usize) -> Self {
        let policy = Coach::new(&vs.root(), GridSpec::default_ssl());

        let dev = vs.device();

        let opt = tch::nn::Adam::default().build(&vs, 1e-3).unwrap();

        let mut old_sim_state = Vec::with_capacity(num_worlds);

        for i in 0..num_worlds {
            old_sim_state.push(empty_world_state(i))

        }


        let mut old_state = Vec::with_capacity(num_worlds);

        for i in 0..num_worlds {
            old_state.push(GameState::default())
        }


        Self {
            data: RootData::new(num_worlds),
            vs,
            policy,
            opt,
            buf: Vec::new(),
            dev,
            old_state,
            old_sim_state,
            values: Vec::new(),
            sampled: SampledRobotAction::default(),
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
        self.sampled = sampled;
        self.log_prob = log_prob;

    }

    pub fn finish_step(&mut self, sim_states: Vec<WorldState>, states: Vec<GameState>) {
        let mut rewards = Vec::with_capacity(states.len());

        for (id, state_sim) in sim_states.iter().enumerate() {
            let old_sim = &self.old_sim_state[id];
            let old = self.old_state[id];

            let state = states[id];

            let commands = self.data.get(id);
            rewards.push(compute_reward(old_sim, state_sim, old, state, commands))
        }

        let obs = mem::replace(&mut self.old_state, states);
        self.old_sim_state = sim_states;

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

    pub fn finish_rollout(&mut self, states: &[GameState]) -> UpdateResult {
        let n = states.len();

        let last_values = tch::no_grad(|| {
            let batch = GameState::encode_multiple(states, self.dev);
            let v = self.policy.critic.forward(&batch);
            (0..n).map(|i| v.double_value(&[i as i64, 0])).collect::<Vec<_>>()
        });

        let t_len = self.buf.len();

        let mut advantages = vec![0.0; t_len * n];
        let mut returns = vec![0.0; t_len * n];

        for e in 0..n {
            let rewards = self.buf.iter().map(|t| t.reward[e]);
            let values = self.buf.iter().map(|t| t.value[e]);

            let (adv, ret) = compute_gae(rewards, values, last_values[e], 0.99, 0.75);

            //TODO: we could potentially fuse this into the compute_gae
            for t in 0..t_len {
                advantages[t * n + e] = adv[t];
                returns[t * n + e] = ret[t];
            }
        }

        let rows = (t_len * n) as i64;

        let mut obs = Vec::with_capacity(t_len * n);

        for item in &self.buf {
            obs.extend_from_slice(&item.obs);
        }

        let batch = GameState::encode_multiple(&obs, self.dev);

        let stack = |f: &dyn Fn(&Transition) -> Tensor| {
            Tensor::stack(&self.buf.iter().map(f).collect::<Vec<_>>(), 0)
                .view([rows, MAX_ROBOTS_PER_TEAM])
        };
        let action_batch = SampledRobotAction {
            command_type: stack(&|t| t.command_type.shallow_clone()),
            target_robot: stack(&|t| t.target_robot.shallow_clone()),
            target_zone: stack(&|t| t.target_zone.shallow_clone()),
            power_bin: stack(&|t| t.power_bin.shallow_clone()),
        };
        let old_log_prob = stack(&|t| t.log_prob.shallow_clone());

        let returns = Tensor::from_slice(&returns)
            .to_kind(Kind::Float)
            .to_device(self.dev);

        let adv = Tensor::from_slice(&advantages)
            .to_kind(Kind::Float)
            .to_device(self.dev);

        let adv_mean = adv.mean(Kind::Float);

        let adv_std = (&adv - &adv_mean)
            .pow_tensor_scalar(2.0)
            .mean(Kind::Float)
            .sqrt();

        let adv = (&adv - &adv_mean) / (adv_std + 1e-8);
        let advantages = adv.unsqueeze(-1).expand([rows, MAX_ROBOTS_PER_TEAM], true);

        let stats = self.ppo_update(
            &batch,
            &action_batch,
            &old_log_prob,
            &returns,
            &advantages,
            0.2,
            0.5,
            0.01,
            0.5,
        );


        stats


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
