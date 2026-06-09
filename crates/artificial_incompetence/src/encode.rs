use tch::{IndexOp, Kind, Tensor};
use crate::ai_types::Batch;
use crate::config::{MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES};
use crate::types::GameState;



impl GameState {
    fn encode(&self, dev: tch::Device) -> Batch {
        let own = Tensor::zeros([MAX_ROBOTS_PER_TEAM, ROBOT_FEATURES], (Kind::Float, dev));
        let own_mask = Tensor::zeros(MAX_ROBOTS_PER_TEAM, (Kind::Bool, dev));

        let own_goalie_mask = Tensor::zeros(MAX_ROBOTS_PER_TEAM, (Kind::Bool, dev));

        for (i, robot) in self.own_robots.iter().enumerate().take(MAX_ROBOTS_PER_TEAM as usize) {
            if let Some(robot) = robot {
                Tensor::from_slice()

                own_mask.i(i).fill_()
                if robot.is_goalie {
                    own_goalie_mask.i((i,)).fill_(true);
                }
            }
        }



        todo!()

    }
}