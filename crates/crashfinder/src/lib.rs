//?
//? This crate hosts the path finding algorithm for our 2027-Now
//? robots.
//? We use A* an ORCA implementation for real time object avoidance
//? When driving the algorithm tries to avoid robots, but some flags
//? may allow interaction with other robots
//? Also at the end position, the algorithm will not move away from
//? robots.
//?
//? The "avoidance_zone" is primarly used for the ball, because in certain
//? szenarious you need to avoid the ball

// Constants to define behaviour and tune the crashpilot
const MAX_ACCEL_MM_S2: f32 = 4_000f32;
const MAX_DECCEL_MM_S2: f32 = 4_000f32;
const MAX_SPEED_MM_S: f32 = 2_000f32;
const MIN_SPEED_MM_S: f32 = 200f32;

pub mod types;
pub mod crashfinder;
