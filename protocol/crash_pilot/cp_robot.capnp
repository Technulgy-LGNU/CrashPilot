@0xcc185f5c4f5f4131;

using Ball = import "../common.capnp";
using Robot = import "../common.capnp";
using Vector2 = import "../common.capnp";

struct CP_Robot {
    robot_id @0 :UInt8;
    timestamp @1 :Float32;

    ball @3 :Ball;

    robots_blue @4 :List(Robot);
    robots_yellow @5 :List(Robot);

    struct Task {
        gc_command @6 :GC_Command;
        cp_command @7 :CP_Command;

        pos @8 :Vector2;
        orientation @9 :Float32;
    }
}

enum GC_Command {
    # Don't move
    HALT @0;
    # Max speed 1.5m/s & 0.5m distance from ball
    STOP @1;
    # Normal start (Kickoff, etc)
    NORMAL_START @2;
    # Just starts the game again
    FORCED_START @3;
}

enum CP_Command {
    # Same as GC HALT
    HALT @0;
    # Drive to the position
    POS @1;
    # Kick
    KICK @3;
    # Recieves the ball
    REC_KICK @4;
}