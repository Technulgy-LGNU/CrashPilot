@0xd05346a7597a775f;

struct Robot_CP {
    robot_id @0: UInt8;

    battery_voltage @1: Float32;

    kicker_ready @2: Bool;
    has_ball @3: Bool;

    error_msg @4: Text;
}