#include "drivetrain.h"

static rotation DummyRotation = rotation(PORT6);

//Must always initialize references, enums, and class objects with no default constructor

Drivetrain::Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double verticalWheelDiameter):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(DummyRotation), //unused member
    VerticalTracker(DummyRotation), //unused member
    Inertial(Inertial),
    inertialScale(inertialScale),
    gearRatio(gearRatio),
    leftVelocity(leftVelocity),
    rightVelocity(rightVelocity),
    verticalWheelDiameter(verticalWheelDiameter),
    odometryType(ZeroTrackerOdometry), //enum
    previousChaining(false),
    previousDirection(1),
    odometry(0, 0) //class object with no default constructor
{};

Drivetrain::Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, rotation& HorizontalTracker, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(HorizontalTracker),
    VerticalTracker(DummyRotation), //unused member
    Inertial(Inertial),
    inertialScale(inertialScale),
    gearRatio(gearRatio),
    leftVelocity(leftVelocity),
    rightVelocity(rightVelocity),
    horizontalWheelDiameter(horizontalWheelDiameter),
    verticalWheelDiameter(verticalWheelDiameter),
    horizontalToCenterDistance(horizontalToCenterDistance),
    odometryType(OneTrackerOdometry),
    previousChaining(false),
    previousDirection(1),
    odometry(horizontalToCenterDistance, 0) //distance from the center to the side of drivetrain doesn't matter since the odometry's y offset is calculated by taking the avg. of the 2 sides of the drivetrain, and the width of the drivetrain base doesn't matter
{};

Drivetrain::Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, rotation& HorizontalTracker, rotation& VerticalTracker, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance):
    Controller(Controller),
    LeftDrive(LeftDrive),
    RightDrive(RightDrive),
    HorizontalTracker(HorizontalTracker),
    VerticalTracker(VerticalTracker),
    Inertial(Inertial),
    inertialScale(inertialScale),
    gearRatio(gearRatio),
    leftVelocity(leftVelocity),
    rightVelocity(rightVelocity),
    horizontalWheelDiameter(horizontalWheelDiameter),
    verticalWheelDiameter(verticalWheelDiameter),
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance),
    odometryType(TwoTrackerOdometry),
    previousChaining(false),
    previousDirection(1),
    odometry(horizontalToCenterDistance, verticalToCenterDistance)
{};

void Drivetrain::calibrateInertial() {
    Inertial.calibrate();
};

//==== Inertial Scale Purpose ====//
/*
    Step 1: Use setInertialHeading to set the heading based on what the sensor thinks it is b/c of inertialScale
    Step 2: Calculate the inertial's rotation in degrees
    Step 3: Multiply that back by inertialScale
    Step 4: Use a wraparound function that returns a double [0, 360)
    Step 5: Return this value

    How it works:
    1. The acutal heading the robot is facing at is the (the value) H
    2. You set the inertial to H / (360 / inertialScale)
    3. You do X degrees of real world rotations (for some double X), which is the inertial thinks is X * (inertialScale / 360)
    4. Therefore, the inertial now thinks your total rotation is (H / (360 / inertialScale)) + (X * (inertialScale / 360)) = (X + H) / (360 / inertialScale)
    5. Now, you multiply by (360 / inertialScale) so that you can report the real world heading as (X + H), which is the heading used in all the calculations
    6. Use a wraparound function so that you report a double in the range [0, 360)
*/

//All inertial calculations will use the Inertial's rotation b/c rotation doesn't have limits so you can wraparound by using fmods, and since it doesn't have limits, you can always multiply by inertial scale

void Drivetrain::setInertialHeading(double heading) {
    Inertial.setRotation(heading / (360 / inertialScale), degrees);
};

double Drivetrain::getInertialHeading() {
    return fmod(fmod(Inertial.rotation(degrees) * (360 / inertialScale), 360) + 360, 360); //the wraparound function
};

void Drivetrain::resetHorizontalTrackerPosition() {
    if(this->odometryType == OneTrackerOdometry || this->odometryType == TwoTrackerOdometry) {
        HorizontalTracker.resetPosition();
    }
}

void Drivetrain::resetVerticalTrackerPosition() {
    if(this->odometryType == ZeroTrackerOdometry || this->odometryType == OneTrackerOdometry) {
        LeftDrive.resetPosition();
        RightDrive.resetPosition();
    }

    else if(odometryType == TwoTrackerOdometry) { //use else if for enums
        VerticalTracker.resetPosition();
    }
}

double Drivetrain::getHorizontalTrackerPositionInches() {
    if(this->odometryType == OneTrackerOdometry || this->odometryType == TwoTrackerOdometry) {
        return (HorizontalTracker.position(degrees)/360) * M_PI * this->horizontalWheelDiameter; //in inches
    }

    else {
        return 0;
    }
}

double Drivetrain::getVerticalTrackerPositionInches() {
    if(this->odometryType == ZeroTrackerOdometry || this->odometryType == OneTrackerOdometry) {
        return (LeftDrive.getPosition() + RightDrive.getPosition())/(2 * 360) * M_PI * this->verticalWheelDiameter * this->gearRatio; //in inches
    }

    else if(this->odometryType == TwoTrackerOdometry) {
        return (VerticalTracker.position(degrees)/360) * M_PI * this->verticalWheelDiameter; //in inches
    }

    else {
        return 0;
    }
}

void Drivetrain::setCoordinates(double initialXCoordinate, double initialYCoordinate, double initialGlobalOrientation) {
    this->resetHorizontalTrackerPosition();
    this->resetVerticalTrackerPosition();
    this->setInertialHeading(initialGlobalOrientation);
    odometry.setPosition(initialXCoordinate, initialYCoordinate, initialGlobalOrientation);

    positionTrackingTask = task(Drivetrain::trackPositionStatic, this); //tells the task that the trackPositionStatic is part of the Drivetrain class not some other class
                                                                        //", this" tells it which object in the drivetrain class to run the task on (on this one)
}

int Drivetrain::trackPositionStatic(void* instance) {
    return static_cast<Drivetrain*>(instance)->trackPosition(); //the nullptr is cast to the Drivetrain class, then calls the function trackPosition
}

int Drivetrain::trackPosition() {
    while(true) {
        this->odometry.updatePosition(this->getHorizontalTrackerPositionInches(), this->getVerticalTrackerPositionInches(), this->getInertialHeading());
        
        vex::wait(5, msec);
    }

    return 0;
}

//==== PID Functions ====//

//drive a straight line distance
void Drivetrain::driveDistance(double targetDistance, bool exit) {
    double initialHeading = odometry.globalOrientation; //for bearing PID

    double headingDelta = 0;

    double initialX = odometry.xCoordinate; //needed b/c driveDistance is relative to your starting position
    double initialY = odometry.yCoordinate; //needed b/c driveDistance is relave to your starting position

    double xDelta = 0; //initializes the change in global x-coordinate between previous and current loop
    double yDelta = 0; //initializes the change in global y-coordinate between previous and current loop

    double distanceTraveled = 0; //initializes the total distance traveled

    PID drivePID(targetDistance, driveDistanceOutputConstants.kp, driveDistanceOutputConstants.ki, driveDistanceOutputConstants.kd, driveDistanceLogicConstants.startIntegral, driveDistanceSettleConstants.deadband, driveDistanceSettleConstants.stallCounter, driveDistanceSettleConstants.counterLimit, driveDistanceControlOutputConstants.minimumMotorOutput, driveDistanceControlOutputConstants.maximumMotorOutput, driveDistanceControlOutputConstants.accelerationSlew, driveDistanceControlOutputConstants.minDecelerationSlew, driveDistanceControlOutputConstants.maxDecelerationSlew, Decay);
    PID bearingPID(0, bearingOutputConstants.kp, bearingOutputConstants.ki, bearingOutputConstants.kd, bearingLogicConstants.startIntegral, bearingSettleConstants.deadband, bearingSettleConstants.stallCounter, bearingSettleConstants.counterLimit, bearingControlOutputConstants.minimumMotorOutput, bearingControlOutputConstants.maximumMotorOutput, bearingControlOutputConstants.accelerationSlew, bearingControlOutputConstants.minDecelerationSlew, bearingControlOutputConstants.maxDecelerationSlew, Reset);

    double driveOutput = 0;
    double bearingOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    do {
        headingDelta = getTurnError(odometry.globalOrientation, initialHeading);

        xDelta = odometry.xCoordinate - initialX ; //measures the change in global x-coordinate
        yDelta = odometry.yCoordinate - initialY; //measures the change in global y-coordinate

        distanceTraveled = xDelta * sin(degreesToRadians(initialHeading)) + yDelta * cos(degreesToRadians(initialHeading)); //adds on to distanceTraveled using the projection formula

        drivePID.error = drivePID.startError - distanceTraveled;
        bearingPID.error = headingDelta;

        if(!exit) {
            driveOutput = 12 * getSign(drivePID.startError);
            driveOutput = 12 * getSign(drivePID.startError);
        }

        driveOutput = drivePID.output();
        bearingOutput = bearingPID.output();

        leftOutput = percentToVolt(driveOutput + bearingOutput);
        rightOutput = percentToVolt(driveOutput - bearingOutput);

        if(this->leftVelocity < this->rightVelocity) {
            rightOutput *= this->leftVelocity / this->rightVelocity;
        }

        if(this->rightVelocity < this->leftVelocity) {
            leftOutput *= this->rightVelocity / this->leftVelocity;
        }

        LeftDrive.spin(forward, leftOutput, voltageUnits::volt);
        RightDrive.spin(forward, rightOutput, voltageUnits::volt);

        vex::wait(20, msec);
    } while(exit ? !drivePID.settled() : std::abs(drivePID.error) > drivePID.deadband);
    
    std::cout << "DrivePID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        previousChaining = false;
    }

    else {
        LeftDrive.stop(coast);
        RightDrive.stop(coast);

        previousChaining = true;

        previousDirection = getSign(drivePID.startError);
    }
};

void Drivetrain::driveToPoint(double finalX, double finalY, bool exit) {
    double targetDistance = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
    double targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //wraps the heading to [0, 360), which is necessary b/c atan2 returns a value in the ragne [-pi, pi), this is the heading facing the target
                                                                                                                                                   //atan is flipped b/c in normal coordinate systems, 0 degrees faces +X, but for our robot, 0 degrees faces +Y

    PID drivePID(targetDistance, driveDistanceOutputConstants.kp, driveDistanceOutputConstants.ki, driveDistanceOutputConstants.kd, driveDistanceLogicConstants.startIntegral, driveDistanceSettleConstants.deadband, driveDistanceSettleConstants.stallCounter, driveDistanceSettleConstants.counterLimit, driveDistanceControlOutputConstants.minimumMotorOutput, driveDistanceControlOutputConstants.maximumMotorOutput, driveDistanceControlOutputConstants.accelerationSlew, driveDistanceControlOutputConstants.minDecelerationSlew, driveDistanceControlOutputConstants.maxDecelerationSlew, Decay);
    PID driveTurnPID(targetHeading, driveTurnOutputConstants.kp, driveTurnOutputConstants.ki, driveTurnOutputConstants.kd, driveTurnLogicConstants.startIntegral, driveTurnSettleConstants.deadband, driveTurnSettleConstants.stallCounter, driveTurnSettleConstants.counterLimit, driveTurnControlOutputConstants.minimumMotorOutput, driveTurnControlOutputConstants.maximumMotorOutput, driveTurnControlOutputConstants.accelerationSlew, driveTurnControlOutputConstants.minDecelerationSlew, driveTurnControlOutputConstants.maxDecelerationSlew, Reset);

    if(std::abs(getTurnError(odometry.globalOrientation, targetHeading)) > 90) {
        driveTurnPID.error = getTurnError(odometry.globalOrientation, fmod((targetHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
    }

    else {
        driveTurnPID.error = getTurnError(odometry.globalOrientation, targetHeading);
    }

    bool locked = false;

    double driveOutput = 0;
    double driveTurnOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    do {
        drivePID.error = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
        targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //updates the heading facing the target
        driveTurnPID.error = getTurnError(odometry.globalOrientation, targetHeading);

        driveOutput = drivePID.output() * cos(degreesToRadians(driveTurnPID.error)); //must calculate before recalculating driveTurnPID.error if going backward b/c that means that initial error is (90, 270), which means cosine will be negative
                                                                                     //also helps reduce forward/backward movement when heading is less correct, applies more power when facing in right direction

        if(std::abs(getTurnError(odometry.globalOrientation, targetHeading)) > 90) { //checks if the absolute value heading delta of the heading delta is greater than 90, meaning the target is behind the robot
            driveTurnPID.error = getTurnError(odometry.globalOrientation, fmod((targetHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
        }

        driveTurnOutput = driveTurnPID.output();

        if(drivePID.error < driveTurnPID.deadband) { //if heading is already close to enough to facing the target or the drivePID is <3 inches away from target, stop turning
            driveTurnOutput = 0;
            locked = true;
        }

        if(locked) {
            driveTurnOutput = 0;
        }

        leftOutput = percentToVolt(driveOutput + driveTurnOutput);
        rightOutput = percentToVolt(driveOutput - driveTurnOutput);

        if(this->leftVelocity < this->rightVelocity) {
            rightOutput *= this->leftVelocity / this->rightVelocity;
        }

        if(this->rightVelocity < this->leftVelocity) {
            leftOutput *= this->rightVelocity / this->leftVelocity;
        }

        LeftDrive.spin(forward, leftOutput, voltageUnits::volt);
        RightDrive.spin(forward, rightOutput, voltageUnits::volt);

        vex::wait(20, msec);
    } while(!drivePID.settled());

    std::cout << "DriveToPointPID is done!" << std::endl;
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
};

//turn to a global heading
void Drivetrain::turnToHeading(double targetHeading, bool exit) {
    PID turnPID(targetHeading, turnToHeadingOutputConstants.kp, turnToHeadingOutputConstants.ki, turnToHeadingOutputConstants.kd, turnToHeadingLogicConstants.startIntegral, turnToHeadingSettleConstants.deadband, turnToHeadingSettleConstants.stallCounter, turnToHeadingSettleConstants.counterLimit, turnToHeadingControlOutputConstants.minimumMotorOutput, turnToHeadingControlOutputConstants.maximumMotorOutput, turnToHeadingControlOutputConstants.accelerationSlew, turnToHeadingControlOutputConstants.minDecelerationSlew,turnToHeadingControlOutputConstants.maxDecelerationSlew, Decay);

    double turnOutput;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    if(previousChaining) {
        if(previousDirection == 1) {
            if(getTurnError(odometry.globalOrientation, targetHeading) > 0) {
                RightDrive.spin(reverse, 12, voltageUnits::volt);
            }

            else {
                LeftDrive.spin(reverse, 12, voltageUnits::volt);
            }
        }

        else {
            if(getTurnError(odometry.globalOrientation, targetHeading) > 0) {
                LeftDrive.spin(forward, 12, voltageUnits::volt);
            }

            else {
                RightDrive.spin(reverse, 12, voltageUnits::volt);
            }
        }
    }

    do {
        turnPID.error = getTurnError(odometry.globalOrientation, targetHeading);

        turnOutput = turnPID.output();

        leftOutput = percentToVolt(turnOutput);
        rightOutput = percentToVolt(turnOutput);

        if(this->leftVelocity < this->rightVelocity) {
            rightOutput *= this->leftVelocity / this->rightVelocity;
        }

        if(this->rightVelocity < this->leftVelocity) {
            leftOutput *= this->rightVelocity / this->leftVelocity;
        }

        LeftDrive.spin(forward, leftOutput, voltageUnits::volt);
        RightDrive.spin(reverse, rightOutput, voltageUnits::volt);

        vex::wait(20, msec);
    } while(exit ? !turnPID.settled() : turnPID.error > turnPID.deadband);


    std::cout << "TurnPID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        previousChaining = false;
    }

    else {
        previousChaining = true;

        previousDirection = getSign(turnPID.startError);
    }
};

void Drivetrain::turnToPoint(double finalX, double finalY, bool exit) {
    double targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //wraps the heading to [0, 360), which is necessary b/c atan2 returns a value in the ragne [-pi, pi), this is the heading facing the target
                                                                                                                                                   //atan is flipped b/c in normal coordinate systems, 0 degrees faces +X, but for our robot, 0 degrees faces +Y

    turnToHeading(targetHeading, exit);
};

//==== AUTON MOVEMENTS ====//
void Drivetrain::leftSpin(directionType direction, int velocity, percentUnits units) {
    LeftDrive.spin(direction, velocity, units);
};

void Drivetrain::rightSpin(directionType direction, int velocity, percentUnits units) {
    RightDrive.spin(direction, velocity, units);
};

void Drivetrain::stop() {
    LeftDrive.stop();
    RightDrive.stop();
};

void Drivetrain::oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation) {
    for(int i = 0; i < numberOfOscillations; i++) {
        LeftDrive.spin(forward, speedOfOscillation, pct);
        RightDrive.spin(forward, speedOfOscillation, pct);
        wait(timeOfOscillation, msec);
        LeftDrive.stop(hold);
        RightDrive.stop(hold);
        LeftDrive.spin(reverse, speedOfOscillation, pct);
        RightDrive.spin(reverse, speedOfOscillation, pct);
        if(i < numberOfOscillations - 1) {
            wait(timeOfOscillation, msec);
        }
        LeftDrive.stop(hold);
        RightDrive.stop(hold);
    }

    LeftDrive.stop(hold);
    RightDrive.stop(hold);
};

//==== Driver Control Functions ====//
void Drivetrain::runArcadeDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed) {
    const controller::button SlowButton = Controller.getButton(SlowButtonID);

    static double totalCount = 0;
    static double total = 0;

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

    LeftDrive.spin(forward);
    RightDrive.spin(forward);

    if(RightDrive.velocity(pct) > 0) {
        totalCount++;
        total += LeftDrive.velocity(pct)/RightDrive.velocity(pct);
    }

    //std::cout << total/totalCount << std::endl;
};

void Drivetrain::runTankDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed) {
    const controller::button SlowButton = Controller.getButton(SlowButtonID);

    static double totalCount = 0;
    static double total = 0;

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

    LeftDrive.spin(forward);
    RightDrive.spin(forward);

    if(RightDrive.velocity(pct) > 0) {
        totalCount++;
        total += LeftDrive.velocity(pct)/RightDrive.velocity(pct);
    }

    //std::cout << total/totalCount << std::endl;
};