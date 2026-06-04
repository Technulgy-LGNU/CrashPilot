use core_dump::vec::types::{Axis, Vec2};
use crate::quadratic::QuadraticResult;

struct PhysicsParams {
    ball_radius: f32,
    acc_slide: f32,
    acc_roll: f32,
    k_switch: f32,
    chip_damping_xy_first_hop: f32,
    chip_damping_xy_other_hops: f32,
    chip_damping_z: f32,
    inertia_distribution: f32,
}


struct BallState {
    pos: Vec2<f32>,
    vel: Vec2<f32>,
}

struct BallTrajectory {
    initial_pos: Vec2<f32>,
    initial_vel: Vec2<f32>,
    start_timestamp: u64,
    physics_params: PhysicsParams,
    t_switch: f32,
    pos_switch: Vec2<f32>,
    vel_switch: Vec2<f32>,
    acc_slide: Vec2<f32>,
    acc_roll: Vec2<f32>,
}

enum Coord {
    X(f32),
    Y(f32),
}

impl Coord {
    fn value(&self) -> f32 {
        match self {
            Coord::X(v) => *v,
            Coord::Y(v) => *v,
        }
    }

    fn to_axis(&self) -> Axis {
        match self {
            Coord::X(_) => Axis::X,
            Coord::Y(_) => Axis::Y,
        }
    }
}



impl BallTrajectory {
    fn new(pos: Vec2<f32>, vel: Vec2<f32>, start_timestamp: u64, physics_params: PhysicsParams, spin: Option<Vec2<f32>>) -> Self {
        let spin = spin.unwrap_or_default();

        let contact_vel = vel + (spin * -physics_params.ball_radius);

        let acc_slide;
        let t_switch;

        if contact_vel.length() < 0.01 {
            acc_slide = vel.scale_to(physics_params.acc_slide);
            t_switch = 0.0;
        } else {
            acc_slide = contact_vel.scale_to(physics_params.acc_slide);
            let inertia = physics_params.inertia_distribution;
            //let acc_slide_spin = acc_slide * (1.0 / (physics_params.ball_radius * inertia));
            let f = 1.0 / (1.0 + 1.0 / inertia);
            let slide_vel = (spin * physics_params.ball_radius + contact_vel * -1.0) * f;

            if acc_slide.x.abs() > acc_slide.y.abs() {
                t_switch = slide_vel.x / acc_slide.x;
            } else {
                t_switch = slide_vel.y / acc_slide.y;
            }
        }

        let vel_switch = vel + acc_slide * t_switch;
        let pos_switch = pos + vel * t_switch + acc_slide * (0.5 * t_switch * t_switch);
        let acc_roll = vel_switch.scale_to(physics_params.acc_roll);

        Self {
            initial_pos: pos,
            initial_vel: vel,
            start_timestamp,
            physics_params,
            t_switch,
            pos_switch,
            vel_switch,
            acc_slide,
            acc_roll,
        }
    }


    fn stops_at(&self) -> f32 {
        let t_stop = -self.vel_switch.length() / self.physics_params.acc_roll;
        self.t_switch + t_stop
    }

    fn state_at_time(&self, t: f32) ->  Option<BallState> {
        let t_rest = self.stops_at();

        if t < 0.0 {
            // Before start
            return None;
        }


        if t > t_rest {
            // we stopped
            let slide_part = self.pos_switch + self.vel_switch  * (t_rest - self.t_switch);
            let roll_part = self.acc_roll * (0.5 * (t_rest - self.t_switch).powf(2.0));
            let final_pos = slide_part + roll_part;



            return Some(BallState {
                pos: final_pos,
                vel: Vec2::zero()
            });
        }


        let pos;
        let vel;

        if t < self.t_switch {
            // Sliding phase
            pos = self.initial_pos + self.initial_vel * t + self.acc_slide * (0.5 * t * t);
            vel = self.initial_vel + self.acc_slide * t;
        } else {
            // Rollingphase
            let t2 = t - self.t_switch;
            pos = self.pos_switch + self.vel_switch * t2 + self.acc_roll * (0.5 * t2 * t2);
            vel = self.vel_switch + self.acc_roll * t2
        }

        Some(BallState {
            pos,
            vel,
        })
    }

    fn find_intersection(&self, coord: Coord) -> Option<(f32, BallState)> {
        let t_rest = self.stops_at();
        let axis = coord.to_axis();
        let target = coord.value();

        let p0 = self.initial_pos.get(axis);
        let v0 = self.initial_vel.get(axis);
        let ps = self.pos_switch.get(axis);
        let vs = self.vel_switch.get(axis);
        let as_ = self.acc_slide.get(axis);
        let ar = self.acc_roll.get(axis);

        for t in QuadraticResult::from_coefficients(0.5 * as_, v0, p0 - target) {
            if t >= 0.0 && t < self.t_switch {
                if let Some(state) = self.state_at_time(t) {
                    return Some((t, state));
                }
            }
        }

        for t2 in QuadraticResult::from_coefficients(0.5 * ar, vs, ps - target) {
            let t = self.t_switch + t2;
            if t >= self.t_switch && t <= t_rest {
                if let Some(state) = self.state_at_time(t) {
                    return Some((t, state));
                }
            }
        }

        None
    }
}
