#include "drivetrain.h"

static rotation DummyRotation = rotation(PORT6);

//Must always initialize references, enums, and class objects with no default constructor

Drivetrain::Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, inertial& Inertial, double gearRatio, double wheelDiameter):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(DummyRotation), //unused member
    VerticalTracker(DummyRotation), //unused member
    Inertial(Inertial),
    gearRatio(gearRatio),
    wheelDiameter(wheelDiameter),
    odometryType(ZeroTrackerOdometry), //enum
    odometry(0, 0) //class object with no default constructor
{};

Drivetrain::Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, inertial& Inertial, double gearRatio, double wheelDiameter, double distanceToSideOfDrivetrain):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(DummyRotation), //unused member
    VerticalTracker(DummyRotation), //unused member
    Inertial(Inertial),
    gearRatio(gearRatio),
    wheelDiameter(wheelDiameter),
    distanceToSideOfDrivetrain(distanceToSideOfDrivetrain),
    odometryType(ZeroTrackerOdometry), //enum
    odometry(0, 0) //class object with no default constructor
{};

Drivetrain::Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, rotation& HorizontalTracker, inertial& Inertial, double gearRatio,  double wheelDiameter, double distanceToSideOfDrivetrain, double horizontalWheelDiameter, double horizontalToCenterDistance):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(HorizontalTracker),
    VerticalTracker(DummyRotation), //unused member
    Inertial(Inertial),
    gearRatio(gearRatio),
    wheelDiameter(wheelDiameter),
    distanceToSideOfDrivetrain(distanceToSideOfDrivetrain),
    horizontalWheelDiameter(horizontalWheelDiameter),
    horizontalToCenterDistance(horizontalToCenterDistance),
    odometryType(OneTrackerOdometry),
    odometry(horizontalToCenterDistance, 0)
{};

Drivetrain::Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, rotation& HorizontalTracker, rotation& VerticalTracker, inertial& Inertial, double gearRatio,  double wheelDiameter, double distanceToSideOfDrivetrain, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(HorizontalTracker),
    VerticalTracker(VerticalTracker),
    Inertial(Inertial),
    gearRatio(gearRatio),
    wheelDiameter(wheelDiameter),
    distanceToSideOfDrivetrain(distanceToSideOfDrivetrain),
    horizontalWheelDiameter(horizontalWheelDiameter),
    verticalWheelDiameter(verticalWheelDiameter),
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance),
    odometryType(TwoTrackerOdometry),
    odometry(horizontalToCenterDistance, verticalToCenterDistance)
{};

void Drivetrain::calibrateInertial() {
    Inertial.calibrate();
};

void Drivetrain::resetInertialHeading() {
    Inertial.resetHeading();
};

void Drivetrain::resetHorizontalTrackerPosition() {
    if (this->odometryType == OneTrackerOdometry || this->odometryType == TwoTrackerOdometry) {
        HorizontalTracker.resetPosition();
    }
}

void Drivetrain::resetVerticalTrackerPosition() {
    if (this->odometryType == ZeroTrackerOdometry || this->odometryType == OneTrackerOdometry) {
        LeftDrive.resetPosition();
        RightDrive.resetPosition();
    }

    else if (odometryType == TwoTrackerOdometry) {
        VerticalTracker.resetPosition();
    }
}

double Drivetrain::getHorizontalTrackerPositionInches() {
    if(this->odometryType == OneTrackerOdometry || this->odometryType == TwoTrackerOdometry) {
        return HorizontalTracker.position(turns) * M_PI * this->horizontalWheelDiameter;
    }

    else {
        return 0;
    }
}

double Drivetrain::getVerticalTrackerPositionInches() {
    if(this->odometryType == ZeroTrackerOdometry || this->odometryType == OneTrackerOdometry) {
        return (LeftDrive.position(turns) + RightDrive.position(turns))/2 * M_PI * this->wheelDiameter;
    }

    else if(this->odometryType == TwoTrackerOdometry) {
        return VerticalTracker.position(turns) * M_PI * this->verticalWheelDiameter;
    }

    else {
        return 0;
    }
}

void Drivetrain::setCoordinates(double initialXCoordinate, double initialYCoordinate, double initialGlobalOrientation) {
    resetHorizontalTrackerPosition();
    resetVerticalTrackerPosition();
    Inertial.setHeading(initialGlobalOrientation, deg);
    odometry.setPosition(initialXCoordinate, initialYCoordinate, initialGlobalOrientation);

    positionTrackingTask = task(Drivetrain::trackPositionStatic, this); //tells the task that the trackPositionStatic is part of the Drivetrain class not some other class
                                                                        //", this" tells it which object in the drivetrain class to run the task on (on this one)
}

int Drivetrain::trackPositionStatic(void* instance) {
    return static_cast<Drivetrain*>(instance)->trackPosition(); //the nullptr is cast to the Drivetrain class, then calls the function trackPosition
}

int Drivetrain::trackPosition() {
    while(true) {
        this->odometry.updatePosition(getHorizontalTrackerPositionInches(), getVerticalTrackerPositionInches(), Inertial.heading(degrees));
        wait(5, msec);
    }

    return 0;
}

//==== PID Functions ====//

//drive a straight line distance
void Drivetrain::driveDistance(double targetDistance) {
    PID drivePID(targetDistance, driveDistanceOutputConstants.kp, driveDistanceOutputConstants.ki, driveDistanceOutputConstants.kd, driveDistanceLogicConstants.startIntegral, driveDistanceSettleConstants.deadband, driveDistanceSettleConstants.stallCounter, driveDistanceSettleConstants.counterLimit, driveDistanceControlOutputConstants.minimumMotorOutput, driveDistanceControlOutputConstants.maximumMotorOutput, driveDistanceControlOutputConstants.slewRate, Decay);
    PID bearingPID(0, bearingOutputConstants.kp, bearingOutputConstants.ki, bearingOutputConstants.kd, bearingLogicConstants.startIntegral, bearingSettleConstants.deadband, bearingSettleConstants.stallCounter, bearingSettleConstants.counterLimit, bearingControlOutputConstants.minimumMotorOutput, bearingControlOutputConstants.maximumMotorOutput, bearingControlOutputConstants.slewRate, Reset);

    double startHeading = Inertial.heading(); //for bearing PID

    double driveOutput = 0;
    double bearingOutput = 0;

    //find these 2 using the ratio code in drivercontrol
    const double leftVelocity = 1;
    const double rightVelocity = 1;

    LeftDrive.resetPosition();
    RightDrive.resetPosition();

    do {
        drivePID.error = drivePID.startError - (RightDrive.position(turns) + LeftDrive.position(turns))/2 * (this->wheelDiameter) * M_PI * (this->gearRatio);
        bearingPID.error = getTurnError(Inertial.heading(), startHeading);

        driveOutput = drivePID.output();
        bearingOutput = bearingPID.output();
        
        if((LeftDrive.velocity(pct) == 0) != (RightDrive.velocity(pct) == 0)) {
            driveOutput = 0;
            bearingOutput = 0;
        }

        LeftDrive.spin(forward, driveOutput + bearingOutput, pct);
        RightDrive.spin(forward, driveOutput - bearingOutput, pct);

        wait(20, msec);
    } while(!drivePID.settled());
    
    std::cout << "DrivePID is done!" << std::endl;
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
};

//drive a straight line distance slowly
void Drivetrain::driveDistanceSlow(double targetDistance) {
    PID drivePID(targetDistance, driveDistanceSlowOutputConstants.kp, driveDistanceSlowOutputConstants.ki, driveDistanceSlowOutputConstants.kd, driveDistanceSlowLogicConstants.startIntegral, driveDistanceSlowSettleConstants.deadband, driveDistanceSlowSettleConstants.stallCounter, driveDistanceSlowSettleConstants.counterLimit, driveDistanceSlowControlOutputConstants.minimumMotorOutput, driveDistanceSlowControlOutputConstants.maximumMotorOutput, driveDistanceControlOutputConstants.slewRate, Decay);
    PID bearingPID(0, bearingSlowOutputConstants.kp, bearingSlowOutputConstants.ki, bearingSlowOutputConstants.kd, bearingSlowLogicConstants.startIntegral, bearingSlowSettleConstants.deadband, bearingSlowSettleConstants.stallCounter, bearingSlowSettleConstants.counterLimit, bearingSlowControlOutputConstants.minimumMotorOutput, bearingSlowControlOutputConstants.maximumMotorOutput, bearingSlowControlOutputConstants.slewRate, Reset);

    double startHeading = Inertial.heading(); //for bearing PID

    double driveOutput = 0;
    double bearingOutput = 0;

    //find these 2 using the ratio code in drivercontrol
    const double leftVelocity = 1;
    const double rightVelocity = 1;

    LeftDrive.resetPosition();
    RightDrive.resetPosition();

    do {
        drivePID.error = drivePID.startError - (RightDrive.position(turns) + LeftDrive.position(turns))/2 * (this->wheelDiameter) * M_PI * (this->gearRatio);
        bearingPID.error = getTurnError(Inertial.heading(), startHeading);
        
        driveOutput = drivePID.output();
        bearingOutput = bearingPID.output();
        
        if((LeftDrive.velocity(pct) == 0) != (RightDrive.velocity(pct) == 0)) {
            driveOutput = 0;
            bearingOutput = 0;
        }

        LeftDrive.spin(forward, driveOutput + bearingOutput, pct);
        RightDrive.spin(forward, driveOutput - bearingOutput, pct);

        wait(20, msec);
    } while(!drivePID.settled());
    
    std::cout << "DrivePID is done!" << std::endl;
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
};

//turn to a global heading
void Drivetrain::turnToHeading(double targetHeading) {
    PID turnPID(targetHeading, turnToHeadingOutputConstants.kp, turnToHeadingOutputConstants.ki, turnToHeadingOutputConstants.kd, turnToHeadingLogicConstants.startIntegral, turnToHeadingSettleConstants.deadband, turnToHeadingSettleConstants.stallCounter, turnToHeadingSettleConstants.counterLimit, turnToHeadingControlOutputConstants.minimumMotorOutput, turnToHeadingControlOutputConstants.maximumMotorOutput, driveDistanceControlOutputConstants.slewRate, Decay);

    double turnOutput;

    do {
        turnPID.error = getTurnError(Inertial.heading(), targetHeading);

        turnOutput = turnPID.output();

        if((LeftDrive.velocity(pct) == 0) != (RightDrive.velocity(pct) == 0)) {
            turnPID.motorOutput = 0;
        }

        LeftDrive.spin(forward, turnOutput, pct);
        RightDrive.spin(reverse, turnOutput, pct);

        wait(20, msec);
    } while(!turnPID.settled());


    std::cout << "TurnPID is done!" << std::endl;
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
};

//==== Driver Control Functions ====//
void Drivetrain::runArcadeDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed) {
    const controller::button SlowButton = Controller.getButton(SlowButtonID);

    static double totalCount = 0;
    static double totalRatio = 0;

    double buttonScaleSpeed = 1;
    
    if(SlowButton.pressing()) {
        buttonScaleSpeed = 0.1;
    }
    else {
        buttonScaleSpeed = 1;
    }

    double driveSpeed = (Controller.Axis3.position()) * scaleDriveSpeed * buttonScaleSpeed;
    double turnSpeed = (Controller.Axis1.position()) * scaleTurnSpeed * buttonScaleSpeed;
    
    LeftDrive.setStopping(coast);
    RightDrive.setStopping(coast);

    LeftDrive.setVelocity((driveSpeed + turnSpeed), pct);
    RightDrive.setVelocity((driveSpeed - turnSpeed), pct);

    if(RightDrive.velocity(pct) > 0) {
        totalCount++;
        totalRatio += LeftDrive.velocity(pct)/RightDrive.velocity(pct);
    }

    //std::cout << totalRatio/totalCount << std::endl;

    LeftDrive.spin(forward);
    RightDrive.spin(forward);
};

void Drivetrain::runTankDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed) {
    const controller::button SlowButton = Controller.getButton(SlowButtonID);

    static double totalCount = 0;
    static double totalRatio = 0;

    double buttonScaleSpeed = 1;
    
    if(SlowButton.pressing()) {
        buttonScaleSpeed = 0.1;
    }
    else {
        buttonScaleSpeed = 1;
    }

    double leftSpeed = (Controller.Axis3.position()) * scaleDriveSpeed * buttonScaleSpeed;
    double rightSpeed = (Controller.Axis2.position()) * scaleTurnSpeed * buttonScaleSpeed;
    
    LeftDrive.setStopping(coast);
    RightDrive.setStopping(coast);

    LeftDrive.setVelocity((leftSpeed), pct);
    RightDrive.setVelocity((rightSpeed), pct);

    if(RightDrive.velocity(pct) > 0) {
        totalCount++;
        totalRatio += LeftDrive.velocity(pct)/RightDrive.velocity(pct);
    }

    //std::cout << totalRatio/totalCount << std::endl;

    LeftDrive.spin(forward);
    RightDrive.spin(forward);
};