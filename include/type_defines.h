#pragma once

#define INERTIAL_SCALE 358.06
#define GEAR_RATIO 0.75
#define LEFT_VELOCITY 1
#define RIGHT_VELOCITY 1
#define HORIZONTAL_WHEEL_DIAMETER 2
#define VERTICAL_WHEEL_DIAMETER 3.25
#define HORIZONTAL_TO_CENTER_DISTANCE 0 //0.375 //IDK if this is accurate
#define VERTICAL_TO_CENTER_DISTANCE 0 //dummy

enum Auton {
    Null,
    RunRightQual,
    RunLeftQual,
    RunRightElim,
    RunLeftElim,
    RunSoloAWP,
    RunAutonSkills
};

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

enum Driver {
    Nikhil,
    Thomas,
    Shaurya,
    Noob
};

enum OdometryType {
    ZeroTrackerOdometry,
    OneTrackerOdometry,
    TwoTrackerOdometry
};

enum PIDType {
    Drive,
    Bearing,
    DriveTurn,
    Turn,
    Swing
};

enum IntegralMethod {
    Decay,
    Reset
};
