


struct BallState {
    pos: Vec2<f32>,
    vel: Vec2<f32>,
    stop_pos: Vec2<f32>,
    stop_time: f32,
}

struct RobotState {
    id: u8,
    pos: Vec2<f32>,
    vel: Vec2<f32>,
    heading: f32,
    angular_vel: f32,
    is_goalie: bool,
    is_lnx: bool, // ignored for opponents
}

type Robots = [Option<RobotState>; 8];


struct GameState {
    own_robots: RobotState,
    opp_robots: RobotState,
    ball: BallState,
}


enum RobotCommand {
    Pos(Vec2<f32>),
    Kick(f32),
    Chip(f32),
    RecKick(f32),
    Steal,
    Dribble(Vec2<f32>),
    PosBall(Vec2<f32>),
    Kickoff(f32),
    FreeKick(f32),
    KickGoal,
    PassTo(u8),
    RecPass,
    GoalWall,
    GoalieGuard,
    Hold,
}


type Commands = [Option<RobotCommand>; 8];


fn predict(state: &GameState, dt: f32) -> Commands {
    todo!()
}