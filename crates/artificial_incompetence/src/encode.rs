use tch::{IndexOp, Kind, Tensor};
use crate::ai_types::Batch;
use crate::config::{MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES};
use crate::types::GameState;



impl GameState {
    fn encode(&self, dev: tch::Device) -> Batch {
        let own = Tensor::zeros([MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES], (Kind::Float, dev));
        let mut own_mask = [false; MAX_ROBOTS_PER_TEAM as usize];

        let mut own_goalie_mask = [false; MAX_ROBOTS_PER_TEAM as usize];

        for (i, robot) in self.own_robots.iter().enumerate().take(MAX_ROBOTS_PER_TEAM as usize) {
            if let Some(robot) = robot {
                own_mask[i] = true;

                if robot.is_goalie {
                    own_goalie_mask[i] = true;
                }

                let features = robot.encode();
                own.get(i as i64).copy_(&features);
            }
        }

        let own_mask = Tensor::from_slice(&own_mask).to_kind(Kind::Bool).to_device(dev);
        let own_goalie_mask = Tensor::from_slice(&own_goalie_mask).to_kind(Kind::Bool).to_device(dev);



        todo!()

    }
}