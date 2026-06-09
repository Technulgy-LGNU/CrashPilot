enum CommandType {
    Pos,
    Kick,
    Chip,
    RecKick,
    Steal,
    Dribble,
    PosBall,
    Kickoff,
    FreeKick,
    KickGoal,
    PassTo,
    RecPass,
    GoalWall,
    GoalieGuard,
    Hold,
}

pub const COMMANDS: &[CommandType] = &[
    CommandType::Pos,
    CommandType::Kick,
    CommandType::Chip,
    CommandType::RecKick,
    CommandType::Steal,
    CommandType::Dribble,
    CommandType::PosBall,
    CommandType::Kickoff,
    CommandType::FreeKick,
    CommandType::KickGoal,
    CommandType::PassTo,
    CommandType::RecPass,
    CommandType::GoalWall,
    CommandType::GoalieGuard,
    CommandType::Hold,
];

pub const NUM_COMMANDS: usize = COMMANDS.len();

pub struct RawRobotCommand {
    cmd: CommandType,
    target_robot: Option<u8>,
    target_zone: Option<u8>,
    pwr: f32,
    score: f32,
}

pub type RawCommands = [RawRobotCommand; 8];


pub struct SampleCommand {
    ty: tch::Tensor,
    target_robot: tch::Tensor,
    target_zone: tch::Tensor,
}


pub struct Batch {
    own: tch::Tensor,
    own_mask: tch::Tensor,
    own_goalie_mask: tch::Tensor,
    opp: tch::Tensor,
    opp_mask: tch::Tensor,
    ball: tch::Tensor,
    zones: tch::Tensor,
    zone_mask: tch::Tensor,
}



