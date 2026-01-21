#pragma once

#define GEAR_RATIO 0.75
#define WHEEL_DIAMETER 3.25
#define DISTANCE_TO_SIDE_OF_DRIVETRAIN 0 //dummy
#define HORIZONTAL_WHEEL_DIAMETER 0 //dummy
#define VERTICAL_WHEEL_DIAMETER 0 //dummy
#define HORIZONTAL_TO_CENTER_DISTANCE 0 //dummy
#define VERTICAL_TO_CENTER_DISTANCE 0 //dummy

enum Button {
    A,
    B,
    X,
    Y,
    Right,
    Left,
    Up,
    Down,
    R1,
    R2,
    L1,
    L2
};

enum OdometryType {
    ZeroTrackerOdometry,
    OneTrackerOdometry,
    TwoTrackerOdometry
};

enum IntegralMethod {
    Decay,
    Reset
};

enum Auton {
    Null,
    RunRightQual,
    RunLeftQual,
    RunRightElim,
    RunLeftElim,
    RunSoloAWP,
    RunAutonSkills
};

enum Driver {
    Nikhil,
    Thomas,
    Shaurya,
    Noob
};

