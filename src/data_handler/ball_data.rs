use crate::proto::{CpBall, CpVector2, InterfaceCommandCp, SslDetectionBall, TrackedBall, Vector3};

pub enum VisionBalls {
  Raw(Vec<SslDetectionBall>),
  Tracked(Vec<TrackedBall>)
}
/// Convert a tracked ball into a CpBall
/// Also does only select the ball who is in the designated test area, if test mode is enabled
pub fn convert_ball(balls: VisionBalls, interface_command: InterfaceCommandCp) -> CpBall {
  // The correct ball, that gets passed on
  let mut correct_ball: TrackedBall = Default::default();
  // Converts all balls to the TrackedBall, so the function works with the default vision
  let mut balls_generic: Vec<TrackedBall> = vec![];

  // Matches the raw vision balls
  match balls {
    VisionBalls::Raw(balls) => {
      for ball in balls {
        balls_generic.push(TrackedBall {
          pos: Vector3 {
            x: ball.x,
            y: ball.y,
            z: 0.0,
          },
          vel: None,
          visibility: None,
        });
      }
    },
    VisionBalls::Tracked(balls) => {
      balls_generic = balls;
    }
  }

  // Test field check, filters for the a specific part of the field
  if interface_command.enable_testfield {
    let mut correct_balls: Vec<&TrackedBall> = vec![];
    // Correct Balls

    // Switch between the test areas
    // We different between the four areas with their omen
    match interface_command.testfield {
      // -x || +y
      0 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x < 0.0 && ball.pos.y > 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // +x || +y
      1 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x > 0.0 && ball.pos.y > 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // +x || -y
      2 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x > 0.0 && ball.pos.y < 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      // -x || -y
      3 => {
        correct_balls = balls_generic.iter()
          .filter(|ball| ball.pos.x < 0.0 && ball.pos.y < 0.0)
          .collect::<Vec<&TrackedBall>>();
      },
      _ => (),
    }
    if !correct_balls.is_empty() {
      correct_ball = *correct_balls[0];
    }
  } else {
    correct_ball = balls_generic[0];
  }
  CpBall {
    pos: CpVector2 {
      x: correct_ball.pos.x as i32,
      y: correct_ball.pos.y as i32,
    },
    vel: Option::from(CpVector2 {
      x: correct_ball.vel.unwrap_or_default().x as i32,
      y: correct_ball.vel.unwrap_or_default().y as i32,
    }),
  }
}
