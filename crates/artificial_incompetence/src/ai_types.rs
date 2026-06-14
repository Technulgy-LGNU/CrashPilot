
#[repr(i64)]
pub enum CommandType {
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


impl CommandType {
    pub fn from_i64(v: i64) -> Self {
        match v {
            0 => Self::Pos,
            1 => Self::Kick,
            2 => Self::Chip,
            3 => Self::RecKick,
            4 => Self::Steal,
            5 => Self::Dribble,
            6 => Self::PosBall,
            7 => Self::Kickoff,
            8 => Self::FreeKick,
            9 => Self::KickGoal,
            10 => Self::PassTo,
            11 => Self::RecPass,
            12 => Self::GoalWall,
            13 => Self::GoalieGuard,
            14 => Self::Hold,
            _ => Self::Hold,
        }
    }

    pub fn uses_zone(self) -> bool {
        matches!(self, Self::Pos | Self::Dribble | Self::PosBall)
    }

    pub fn uses_power(self) -> bool {
        matches!(
            self,
            Self::Kick | Self::Chip | Self::RecKick | Self::Kickoff | Self::FreeKick
        )
    }

    pub fn uses_teammate(self) -> bool {
        matches!(self, Self::PassTo)
    }
}

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
    pub own: tch::Tensor,
    pub own_mask: tch::Tensor,
    pub own_goalie_mask: tch::Tensor,
    pub opp: tch::Tensor,
    pub opp_mask: tch::Tensor,
    pub ball: tch::Tensor,
    // pub zones: tch::Tensor,
    // pub zone_mask: tch::Tensor,
}


pub struct MultiBatch {
    pub own: tch::Tensor,
    pub own_mask: tch::Tensor,
    pub own_goalie_mask: tch::Tensor,
    pub opp: tch::Tensor,
    pub opp_mask: tch::Tensor,
    pub ball: tch::Tensor,
    // pub zones: tch::Tensor,
    // pub zone_mask: tch::Tensor,
}



