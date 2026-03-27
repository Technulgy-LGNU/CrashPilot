@0xf86852cd805ff738;

struct Ball {
  # The position (x, y, height) [m] in the ssl-vision coordinate system
  pos @0 :Vector3;

  # The velocity [m/s] in the ssl-vision coordinate system
  vel @1 :Vector3;
}

struct KickedBall {
  # The initial position [m] from which the ball was kicked
  pos @0 :Vector2;

  # The initial velocity [m/s] with which the ball was kicked
  vel @1 :Vector3;

  # The unix timestamp [s] when the kick was performed
  start_timestamp @2 :Float64;

  # The predicted unix timestamp [s] when the ball comes to a stop
  stop_timestamp @3 :Float64;

  # The predicted position [m] at which the ball will come to a stop
  stop_pos @4 :Vector2;

  # The robot that kicked the ball
  robot_id @5 :RobotId;
}

struct TrackedRobot {
  robot_id @0 :UInt8;

  # The position [m] in the ssl-vision coordinate system
  pos @1 :Vector2;

  # The orientation [rad]
  orientation @2 :Float32;

  # The velocity [m/s]
  vel @3 :Vector2;
}

struct Vector2 {
  x @0 :Float32;  # meters
  y @1 :Float32;  # meters
}

struct Vector3 {
  x @0 :Float32;  # meters
  y @1 :Float32;  # meters
  z @2 :Float32;  # meters (height)
}