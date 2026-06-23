use crate::ai_types::{CommandType, MultiBatch, SampledRobotAction};
use crate::config::{MAX_ROBOTS_PER_TEAM, NUM_POWER_BINS};
use crate::grid::GridSpec;
use crate::modules::actor::{Actor, ActorOutput};
use crate::modules::critic::Critic;
use crate::types::{Commands, RobotCommand};
use tch::{Kind, Tensor, nn};

pub struct Coach {
  actor: Actor,
  pub critic: Critic,
  grid_spec: GridSpec,
}

impl Coach {
  pub(crate) fn new(vs: &nn::Path, grid_spec: GridSpec) -> Self {
    let actor = Actor::new(&(vs / "actor"), grid_spec, 128, 256, 4);
    let critic = Critic::new(&(vs / "critic"), grid_spec, 128, 256, 4, 2);

    Self {
      actor,
      critic,
      grid_spec,
    }
  }

  pub fn act(
    &self,
    batch: &MultiBatch,
    deterministic: bool,
  ) -> (SampledRobotAction, Tensor, Tensor, Vec<Commands>) {
    let out = self.actor.forward(batch);

    let command_type = sample_categorical_from_logits(&out.command_logits, deterministic);
    let target_robot = sample_categorical_from_logits(&out.teammate_logits, deterministic);
    let target_zone = sample_categorical_from_logits(&out.zone_logits, deterministic);
    let power_bin = sample_categorical_from_logits(&out.power_logits, deterministic);

    let sampled = SampledRobotAction {
      command_type,
      target_robot,
      target_zone,
      power_bin,
    };

    let log_prob = self.log_prob_of(batch, &sampled);
    let value = self.critic.forward(batch);
    let plans = self.decode_team_plans(batch, &sampled, Some(&out));

    (sampled, log_prob, value, plans)
  }

  fn log_prob_of(&self, batch: &MultiBatch, action: &SampledRobotAction) -> Tensor {
    let out = self.actor.forward(batch);

    let mut logp = categorical_log_prob(&out.command_logits, &action.command_type);

    let zone_like = action
      .command_type
      .eq(CommandType::Pos as i64)
      .logical_or(&action.command_type.eq(CommandType::Dribble as i64))
      .logical_or(&action.command_type.eq(CommandType::PosBall as i64));

    let power_like = action
      .command_type
      .eq(CommandType::Kick as i64)
      .logical_or(&action.command_type.eq(CommandType::Chip as i64))
      .logical_or(&action.command_type.eq(CommandType::RecKick as i64))
      .logical_or(&action.command_type.eq(CommandType::Kickoff as i64))
      .logical_or(&action.command_type.eq(CommandType::FreeKick as i64));

    let teammate_like = action.command_type.eq(CommandType::PassTo as i64);

    logp += categorical_log_prob(&out.teammate_logits, &action.target_robot)
      * teammate_like.to_kind(Kind::Float);

    logp +=
      categorical_log_prob(&out.zone_logits, &action.target_zone) * zone_like.to_kind(Kind::Float);

    logp +=
      categorical_log_prob(&out.power_logits, &action.power_bin) * power_like.to_kind(Kind::Float);

    logp * batch.own_mask.to_kind(Kind::Float)
  }

  pub fn evaluate_actions(
    &self,
    batch: &MultiBatch,
    action: &SampledRobotAction,
  ) -> (Tensor, Tensor, Tensor) {
    let log_prob = self.log_prob_of(batch, action);
    let out = self.actor.forward(batch);

    let zone_like = action
      .command_type
      .eq(CommandType::Pos as i64)
      .logical_or(&action.command_type.eq(CommandType::Dribble as i64))
      .logical_or(&action.command_type.eq(CommandType::PosBall as i64));

    let power_like = action
      .command_type
      .eq(CommandType::Kick as i64)
      .logical_or(&action.command_type.eq(CommandType::Chip as i64))
      .logical_or(&action.command_type.eq(CommandType::RecKick as i64))
      .logical_or(&action.command_type.eq(CommandType::Kickoff as i64))
      .logical_or(&action.command_type.eq(CommandType::FreeKick as i64));

    let teammate_like = action.command_type.eq(CommandType::PassTo as i64);

    let mut ent = categorical_entropy(&out.command_logits);
    ent += categorical_entropy(&out.teammate_logits) * teammate_like.to_kind(Kind::Float);
    ent += categorical_entropy(&out.zone_logits) * zone_like.to_kind(Kind::Float);
    ent += categorical_entropy(&out.power_logits) * power_like.to_kind(Kind::Float);
    ent *= batch.own_mask.to_kind(Kind::Float);

    let value = self.critic.forward(batch);
    (log_prob, ent, value)
  }

  fn decode_robot_command(
    &self,
    cmd_type: CommandType,
    zone_idx: i64,
    power_idx: i64,
    robot_idx: i64,
  ) -> RobotCommand {
    let zone = self.grid_spec.zone_center(zone_idx);
    let power = power_from_bin(power_idx);

    match cmd_type {
      CommandType::Pos => RobotCommand::Pos(zone),
      CommandType::Kick => RobotCommand::Kick(power),
      CommandType::Chip => RobotCommand::Chip(power),
      CommandType::RecKick => RobotCommand::RecKick(power),
      CommandType::Steal => RobotCommand::Steal,
      CommandType::Dribble => RobotCommand::Dribble(zone),
      CommandType::PosBall => RobotCommand::PosBall(zone),
      CommandType::Kickoff => RobotCommand::Kickoff(power),
      CommandType::FreeKick => RobotCommand::FreeKick(power),
      CommandType::KickGoal => RobotCommand::KickGoal,
      CommandType::PassTo => RobotCommand::PassTo(robot_idx as u8),
      CommandType::RecPass => RobotCommand::RecPass,
      CommandType::GoalWall => RobotCommand::GoalWall,
      CommandType::GoalieGuard => RobotCommand::GoalieGuard,
      CommandType::Hold => RobotCommand::Hold,
    }
  }

  fn decode_team_plans(
    &self,
    batch: &MultiBatch,
    sampled: &SampledRobotAction,
    actor_out: Option<&ActorOutput>,
  ) -> Vec<Commands> {
    let owned;
    let out = match actor_out {
      Some(v) => v,
      None => {
        owned = self.actor.forward(batch);
        &owned
      }
    };

    let _probs = out.command_logits.softmax(-1, Kind::Float);
    let bsz = batch.own.size()[0];
    let mut plans = Vec::new();

    for b in 0..bsz {
      let mut commands = [None; MAX_ROBOTS_PER_TEAM as usize];

      for i in 0..MAX_ROBOTS_PER_TEAM {
        if batch.own_mask.int64_value(&[b, i]) == 0 {
          continue;
        }

        let cmd_idx = sampled.command_type.int64_value(&[b, i]);
        let zone_idx = sampled.target_zone.int64_value(&[b, i]);
        let power_idx = sampled.power_bin.int64_value(&[b, i]);
        let target_robot = sampled.target_robot.int64_value(&[b, i]);

        let cmd_type = CommandType::from_i64(cmd_idx);
        let cmd = self.decode_robot_command(cmd_type, zone_idx, power_idx, target_robot);
        commands[i as usize] = Some(cmd);
      }

      let commands = self.resolve_team_consistency(commands);
      plans.push(commands);
    }

    plans
  }

  fn resolve_team_consistency(&self, commands: Commands) -> Commands {
    let mut updated = commands.clone();

    for (_, cmd) in commands.iter().enumerate() {
      if let Some(cmd) = cmd {
        if let RobotCommand::PassTo(dst) = cmd {
          let dst = *dst as usize;
          if updated.len() > dst {
            if updated[dst] != Some(RobotCommand::RecPass) {
              updated[dst] = Some(RobotCommand::RecPass);
            }
          }
        }
      }
    }

    updated
  }
}

fn power_from_bin(idx: i64) -> f32 {
  (idx as f32 + 1.0) / NUM_POWER_BINS as f32
}

fn categorical_entropy(logits: &Tensor) -> Tensor {
  let p = logits.softmax(-1, Kind::Float);
  let logp = logits.log_softmax(-1, Kind::Float);
  -(p * logp).sum_dim_intlist([-1].as_ref(), false, Kind::Float)
}


fn sample_categorical_from_logits(logits: &Tensor, deterministic: bool) -> Tensor {
  if deterministic {
    logits.argmax(-1, false)
  } else {
    logits
        .softmax(-1, Kind::Float)
        .multinomial(1, true)
        .squeeze_dim(-1)
  }
}

fn categorical_log_prob(logits: &Tensor, actions: &Tensor) -> Tensor {
  let logp = logits.log_softmax(-1, Kind::Float);
  logp.gather(-1, &actions.unsqueeze(-1), false)
      .squeeze_dim(-1)
}

