use tch::{IndexOp, Kind, Tensor};
use crate::ai_types::{CommandType, MultiBatch, NUM_COMMANDS};
use crate::config::{MAX_ROBOTS_PER_TEAM, NUM_ZONES};

pub struct Masks {
    pub action_mask: Tensor,
    pub teammate_mask: Tensor,
}

pub fn estimate_has_ball(batch: &MultiBatch, ball_radius: f32) -> Tensor {
    let own_xy = batch.own.narrow(-1, 0, 2);
    let ball_xy = batch.ball.narrow(-1, 0, 2).unsqueeze(1);

    let diff = own_xy - ball_xy;
    let dist = (&diff * &diff)
        .pow_tensor_scalar(2.0)
        .sum_dim_intlist([-1].as_ref(), false, Kind::Float)
        .sqrt();

    dist.lt(ball_radius as f64).logical_and(&batch.own_mask)
}

fn build_action_masks(batch: &MultiBatch) -> Masks {
    let own = &batch.own;
    let own_mask = &batch.own_mask;
    let own_goalie_mask = &batch.own_goalie_mask;
    let base_zone_mask = &Tensor::new();

    let b = own.size()[0];
    let device = own.device();

    let mut action_mask =
        Tensor::zeros([b, MAX_ROBOTS_PER_TEAM, NUM_COMMANDS as i64], (Kind::Bool, device));
    let mut teammate_mask =
        Tensor::zeros([b, MAX_ROBOTS_PER_TEAM, MAX_ROBOTS_PER_TEAM], (Kind::Bool, device));

    let has_ball = estimate_has_ball(batch, 0.18);


    for i in 0..MAX_ROBOTS_PER_TEAM {
        let active = own_mask.narrow(1, i, 1).squeeze_dim(1);
        let inactive = active.logical_not();

        let goalie = batch.own_goalie_mask.narrow(1, i, 1).squeeze_dim(1);
        let field_player = active.logical_and(&goalie.logical_not());
        let has_ball_i = has_ball.narrow(1, i, 1).squeeze_dim(1);

        let mut tm_row = teammate_mask.narrow(1, i, 1).squeeze_dim(1);
        tm_row.copy_(own_mask);
        let _ = tm_row.narrow(1, i, 1).fill_(0);


        let has_ball = field_player.logical_and(&has_ball_i);


        let setup_mask = |cmd_type, val| {
            action_mask
                .narrow(1, i, 1)
                .narrow(2, cmd_type as i64, 1)
                .copy_(val.unsqueeze(-1))
        };

        setup_mask(CommandType::Hold, &Tensor::ones(b, (Kind::Bool, device)));
        setup_mask(CommandType::Pos, &active);
        setup_mask(CommandType::Kick, &has_ball_i);
        setup_mask(CommandType::Chip, &has_ball_i);
        setup_mask(CommandType::RecKick, &has_ball_i);
        setup_mask(CommandType::Steal, &has_ball_i);
        setup_mask(CommandType::Dribble, &has_ball_i);
        setup_mask(CommandType::PosBall, &has_ball_i);
        setup_mask(CommandType::Kickoff, &has_ball_i);
        setup_mask(CommandType::KickGoal, &has_ball_i);
        setup_mask(CommandType::PassTo, &has_ball_i);
        setup_mask(CommandType::RecPass, &field_player);
        setup_mask(CommandType::GoalWall, &field_player);
        setup_mask(CommandType::GoalWall, &field_player);
        setup_mask(CommandType::GoalieGuard, &field_player); //TODO: should this be field_player?
    }

    Masks {
        action_mask,
        teammate_mask,
    }
}