use crate::ai_types::{Batch, MultiBatch};
use crate::config::{MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES};
use crate::types::{BallState, GameState, RobotState};
use std::mem;
use tch::{Kind, Tensor};

impl GameState {
  pub fn encode(&self, dev: tch::Device) -> Batch {
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
      own_mask[i] = true;

      let features = if let Some(robot) = robot {
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
      opp_mask[i] = true;

      let features = if let Some(robot) = robot {
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

    let own_goalie_mask = Tensor::from_slice(&own_goalie_mask)
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

  pub fn encode_multiple(states: &[GameState], dev: tch::Device) -> MultiBatch {
    let mut temp = Vec::with_capacity(states.len());

    let mut states = states
      .iter()
      .map(|state| state.encode(dev))
      .collect::<Vec<_>>();

    for state in &mut states {
      temp.push(mem::replace(&mut state.own, Tensor::new()));
    }

    let own = Tensor::stack(&temp, 0);

    for state in &mut states {
      temp.push(mem::replace(&mut state.own_mask, Tensor::new()));
    }

    let own_mask = Tensor::stack(&temp, 0);

    for state in &mut states {
      temp.push(mem::replace(&mut state.own_goalie_mask, Tensor::new()));
    }

    let own_goalie_mask = Tensor::stack(&temp, 0);

    for state in &mut states {
      temp.push(mem::replace(&mut state.opp, Tensor::new()));
    }

    let opp = Tensor::stack(&temp, 0);

    for state in &mut states {
      temp.push(mem::replace(&mut state.opp_mask, Tensor::new()));
    }

    let opp_mask = Tensor::stack(&temp, 0);

    for state in &mut states {
      temp.push(mem::replace(&mut state.ball, Tensor::new()));
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

impl RobotState {
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

impl BallState {
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
