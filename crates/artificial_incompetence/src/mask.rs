use tch::{IndexOp, Kind, Tensor};
use crate::ai_types::{CommandType, MultiBatch, NUM_COMMANDS};
use crate::config::{MAX_ROBOTS_PER_TEAM, NUM_ZONES};


pub struct Masks {
    pub action_mask: Tensor,
    pub teammate_mask: Tensor,
}

pub fn estimate_has_ball(batch: &MultiBatch, ball_radius: f32) -> Tensor {
    let own_xy = batch.own.i((.., .., 0..2));
    let ball_xy = batch.ball.i((.., .., 0..2)).unsqueeze(1);

    let diff = own_xy - ball_xy;
    let dist = (&diff * &diff)
        .sum_dim_intlist(&[-1i64], false, Kind::Float)
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

    let zone_mask = base_zone_mask
        .unsqueeze(1)
        .expand([b, MAX_ROBOTS_PER_TEAM, NUM_ZONES], true);

    let has_ball = estimate_has_ball(batch, 0.18);

    for i in 0..MAX_ROBOTS_PER_TEAM {
        let mut row = teammate_mask.i((.., i, ..));
        row.copy_(own_mask);

        let mut diag = teammate_mask.i((.., i, i));
        diag.copy_(&Tensor::zeros([b], (Kind::Bool, device)));
    }

    for i in 0..MAX_ROBOTS_PER_TEAM {
        let active = own_mask.i((.., i));
        let goalie = own_goalie_mask.i((.., i));
        let field_player = active.logical_and(&goalie.logical_not());

        // let zone_ok = zone_mask.i((.., i, ..)).any_dim(-1, false);
        let teammate_ok = teammate_mask.i((.., i, ..)).any_dim(-1, false);
        let has_ball_i = has_ball.i((.., i));

        // pub enum RobotCommand {
        //     Pos(Vec2<f32>), //Vec2 is calculated by the active zone logit (any zone is allowed)
        //     Kick(f32), // similar to pos, but with a power parameter, it is calculated by the active kick power logit (any is allowed)
        //     Chip(f32),
        //     RecKick(f32),
        //     Steal,
        //     Dribble(Vec2<f32>),
        //     PosBall(Vec2<f32>),
        //     Kickoff(f32),
        //     FreeKick(f32),
        //     KickGoal,
        //     PassTo(u8),
        //     RecPass,
        //     GoalWall,
        //     GoalieGuard,
        //     Hold,

        {
            let mut dst = action_mask.i((.., i, CommandType::Pos as i64));
            dst.copy_(&active.logical_and(&zone_ok));
        }

        {
            let mut dst = action_mask.i((.., i, CommandType::Hold as i64));
            dst.copy_(&active);
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::Pos as i64));
            dst.copy_(&active.logical_and(&zone_ok));
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::Ball as i64));
            dst.copy_(&field_player);
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::PassToRobot as i64));
            dst.copy_(&field_player.logical_and(&has_ball_i).logical_and(&teammate_ok));
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::ShootGoal as i64));
            dst.copy_(&field_player.logical_and(&has_ball_i));
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::ReceivePass as i64));
            dst.copy_(&field_player);
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::BuildWall as i64));
            dst.copy_(&field_player.logical_and(&zone_ok));
        }
        {
            let mut dst = action_mask.i((.., i, CommandType::GoalieGuard as i64));
            dst.copy_(&goalie);
        }
    }


    Masks {
        action_mask,
        teammate_mask,
    }
}