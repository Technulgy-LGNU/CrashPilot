use crate::ai_types::{Batch, MultiBatch};
use crate::config::{MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES};
use core_dump::types::{BallState, GameState, RobotState};
use std::mem;
use tch::{Kind, Tensor};

pub trait GameStateExt {
  fn encode(&self, dev: tch::Device) -> Batch;
  fn encode_multiple(states: &[GameState], dev: tch::Device) -> MultiBatch;
}

pub trait RobotStateExt {
  fn encode(&self, team_sign: f32) -> Tensor;
  fn encode_with_active(&self, team_sign: f32, active: bool) -> Tensor;
  fn encode_empty(team_sign: f32) -> Tensor;
}

pub trait BallStateExt {
  fn encode(&self) -> Tensor;
}

impl GameStateExt for GameState {
  fn encode(&self, dev: tch::Device) -> Batch {
    let own = Tensor::zeros(
      [MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES],
      (Kind::Float, tch::Device::Cpu),
    );
    let mut own_mask = [false; MAX_ROBOTS_PER_TEAM as usize];

    let mut own_goalie_mask = [false; MAX_ROBOTS_PER_TEAM as usize];

    for (i, robot) in self
      .own_robots
      .iter()
      .enumerate()
      .take(MAX_ROBOTS_PER_TEAM as usize)
    {
      let features = if let Some(robot) = robot {
        own_mask[i] = true;

        if robot.is_goalie {
          own_goalie_mask[i] = true;
        }

        robot.encode(1.0)
      } else {
        RobotState::encode_empty(1.0)
      };

      own.get(i as i64).copy_(&features);
    }

    let own = own.to_device(dev);

    let own_mask = Tensor::from_slice(&own_mask)
      .to_kind(Kind::Bool)
      .to_device(dev);
    let own_goalie_mask = Tensor::from_slice(&own_goalie_mask)
      .to_kind(Kind::Bool)
      .to_device(dev);

    let opp = Tensor::zeros(
      [MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES],
      (Kind::Float, tch::Device::Cpu),
    );
    let mut opp_mask = [false; MAX_ROBOTS_PER_TEAM as usize];

    for (i, robot) in self
      .opp_robots
      .iter()
      .enumerate()
      .take(MAX_ROBOTS_PER_TEAM as usize)
    {
      let features = if let Some(robot) = robot {
        opp_mask[i] = true;
        robot.encode(-1.0)
      } else {
        RobotState::encode_empty(-1.0)
      };

      opp.get(i as i64).copy_(&features);
    }

    let opp = opp.to_device(dev);

    let opp_mask = Tensor::from_slice(&opp_mask)
      .to_kind(Kind::Bool)
      .to_device(dev);

    let ball = self.ball.encode().to_device(dev);

    Batch {
      own,
      own_mask,
      own_goalie_mask,
      opp,
      opp_mask,
      ball,
    }
  }

  fn encode_multiple(states: &[GameState], dev: tch::Device) -> MultiBatch {
    let mut states = states
      .iter()
      .map(|state| state.encode(dev))
      .collect::<Vec<_>>();

    let empty = Tensor::new();

    let mut temp = Vec::with_capacity(states.len());
    for state in &mut states {
      temp.push(mem::replace(&mut state.own, empty.shallow_clone()));
    }

    let own = Tensor::stack(&temp, 0);

    temp.clear();
    for state in &mut states {
      temp.push(mem::replace(&mut state.own_mask, empty.shallow_clone()));
    }

    let own_mask = Tensor::stack(&temp, 0);

    temp.clear();
    for state in &mut states {
      temp.push(mem::replace(
        &mut state.own_goalie_mask,
        empty.shallow_clone(),
      ));
    }

    let own_goalie_mask = Tensor::stack(&temp, 0);

    temp.clear();
    for state in &mut states {
      temp.push(mem::replace(&mut state.opp, empty.shallow_clone()));
    }

    let opp = Tensor::stack(&temp, 0);

    temp.clear();
    for state in &mut states {
      temp.push(mem::replace(&mut state.opp_mask, empty.shallow_clone()));
    }

    let opp_mask = Tensor::stack(&temp, 0);

    temp.clear();
    for state in &mut states {
      temp.push(mem::replace(&mut state.ball, empty.shallow_clone()));
    }

    let ball = Tensor::stack(&temp, 0);

    MultiBatch {
      own,
      own_mask,
      own_goalie_mask,
      opp,
      opp_mask,
      ball,
    }
  }
}

impl RobotStateExt for RobotState {
  fn encode(&self, team_sign: f32) -> Tensor {
    self.encode_with_active(team_sign, true)
  }

  fn encode_with_active(&self, team_sign: f32, active: bool) -> Tensor {
    Tensor::from_slice(&[
      self.pos.x,
      self.pos.y,
      self.vel.x,
      self.vel.y,
      self.heading.sin(),
      self.heading.cos(),
      self.angular_vel,
      team_sign,
      self.id as f32 / (MAX_ROBOTS_PER_TEAM - 1) as f32,
      self.is_goalie as u8 as f32,
      self.vel.norm(),
      active as u8 as f32,
    ])
    .to_kind(Kind::Float)
  }

  fn encode_empty(team_sign: f32) -> Tensor {
    Self::default().encode_with_active(team_sign, false)
  }
}

impl BallStateExt for BallState {
  fn encode(&self) -> Tensor {
    Tensor::from_slice(&[
      self.pos.x,
      self.pos.y,
      self.vel.x,
      self.vel.y,
      self.stop_pos.x,
      self.stop_pos.y,
      self.stop_time,
    ])
    .to_kind(Kind::Float)
  }
}
