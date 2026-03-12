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
    drivePID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Decay),
    bearingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    driveTurnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    turnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    swingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    prevPIDType(Drive), //enum
    prevChaining(false),
    prevDirection(1),
    prevError(0),
    prevIntegral(0),
    prevMotorOutput(0),
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
    odometryType(OneTrackerOdometry), //enum
    drivePID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Decay),
    bearingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    driveTurnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    turnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    swingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    prevPIDType(Drive), //enum
    prevChaining(false),
    prevDirection(1),
    prevError(0),
    prevIntegral(0),
    prevMotorOutput(0),
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
    odometryType(TwoTrackerOdometry), //enum
    drivePID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Decay),
    bearingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    driveTurnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    turnPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    swingPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, Reset),
    prevPIDType(Drive), //enum
    prevChaining(false),
    prevDirection(1),
    prevError(0),
    prevIntegral(0),
    prevMotorOutput(0),
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
    Inertial.setRotation(heading / (360 / this->inertialScale), degrees);
};

double Drivetrain::getInertialHeading() {
    return fmod(fmod(Inertial.rotation(degrees) * (360 / this->inertialScale), 360) + 360, 360); //the wraparound function
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

    else if(this->odometryType == TwoTrackerOdometry) { //use else if for enums
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
    this->odometry.setPosition(initialXCoordinate, initialYCoordinate, initialGlobalOrientation);

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
void Drivetrain::driveDistanceEncoder(double targetDistance, bool exit) {
    double initialHeading = odometry.globalOrientation; //for bearing PID
    double headingDelta = 0;

    double initialEncoderAverage = getVerticalTrackerPositionInches(); //initial average of the left and right encoders, used to find the change in position by subtracting this value from the current average encoder value
    double distanceTraveled = 0; //initializes the total distance traveled

    this->drivePID.setError(targetDistance);
    this->drivePID.setOutputConstants(driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd);
    this->drivePID.setLogicConstants(driveLogicConstants.startIntegral);
    this->drivePID.setSettleConstants(driveSettleConstants.deadband, driveSettleConstants.stallCounter, driveSettleConstants.counterLimit);
    this->drivePID.setControlOutputConstants(driveControlOutputConstants.minMotorOutput, driveControlOutputConstants.maxMotorOutput, driveControlOutputConstants.accelerationSlew, driveControlOutputConstants.minDecelerationSlew, driveControlOutputConstants.maxDecelerationSlew);
    this->drivePID.setIntegralMethod(Decay);

    this->drivePID.setDerivative(0);
    this->drivePID.setMotorOutput(0);
    this->drivePID.setCounter(0);

    if(!prevChaining) {
        this->drivePID.setIntegral(0);
        this->drivePID.setPrevError(this->drivePID.startError);
        this->drivePID.setPrevMotorOutput(0);
    }

    else {
        this->drivePID.setIntegral(this->prevIntegral);
        this->drivePID.setPrevError(this->prevError);
        this->drivePID.setPrevMotorOutput(this->prevMotorOutput);
    }

    this->bearingPID.setError(0);
    this->bearingPID.setOutputConstants(bearingOutputConstants.kp, bearingOutputConstants.ki, bearingOutputConstants.kd);
    this->bearingPID.setLogicConstants(bearingLogicConstants.startIntegral);
    this->bearingPID.setSettleConstants(bearingSettleConstants.deadband, bearingSettleConstants.stallCounter, bearingSettleConstants.counterLimit);
    this->bearingPID.setControlOutputConstants(bearingControlOutputConstants.minMotorOutput, bearingControlOutputConstants.maxMotorOutput, bearingControlOutputConstants.accelerationSlew, bearingControlOutputConstants.minDecelerationSlew, bearingControlOutputConstants.maxDecelerationSlew);
    this->bearingPID.setIntegralMethod(Reset);

    this->bearingPID.setIntegral(0);
    this->bearingPID.setPrevError(this->bearingPID.startError);
    this->bearingPID.setDerivative(0);
    this->bearingPID.setMotorOutput(0);
    this->bearingPID.setCounter(0);
    this->bearingPID.setPrevMotorOutput(0);

    double driveOutput = 0;
    double bearingOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;

    do {
        headingDelta = getTurnError(this->odometry.globalOrientation, initialHeading);

        distanceTraveled = getVerticalTrackerPositionInches() - initialEncoderAverage; //calculates total distance traveled using change in encoder values
        
        this->drivePID.error = this->drivePID.startError - distanceTraveled;
        this->bearingPID.error = headingDelta;

        driveOutput = this->drivePID.output();
        bearingOutput = this->bearingPID.output();

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
    } while(exit ? !this->drivePID.settled() : std::abs(this->drivePID.error) > this->drivePID.deadband);
    
    std::cout << "DrivePID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->drivePID.startError);
        this->prevError = this->drivePID.error;
        this->prevIntegral = this->drivePID.integral + this->bearingPID.integral;
        this->prevMotorOutput = this->drivePID.motorOutput + this->bearingPID.motorOutput;

        this->prevChaining = true;
    }

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;
};

//drive a straight line distance
void Drivetrain::driveDistance(double targetDistance, bool exit) {
    double initialHeading = odometry.globalOrientation; //for bearing PID

    double headingDelta = 0;
    
    double initialX = odometry.xCoordinate; //needed b/c driveDistance is relative to your starting position
    double initialY = odometry.yCoordinate; //needed b/c driveDistance is relave to your starting position
    
    double xDelta = 0; //initializes the change in global x-coordinate between previous and current loop
    double yDelta = 0; //initializes the change in global y-coordinate between previous and current loop
    
    double distanceTraveled = 0; //initializes the total distance traveled

    this->drivePID.setError(targetDistance);
    this->drivePID.setOutputConstants(driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd);
    this->drivePID.setLogicConstants(driveLogicConstants.startIntegral);
    this->drivePID.setSettleConstants(driveSettleConstants.deadband, driveSettleConstants.stallCounter, driveSettleConstants.counterLimit);
    this->drivePID.setControlOutputConstants(driveControlOutputConstants.minMotorOutput, driveControlOutputConstants.maxMotorOutput, driveControlOutputConstants.accelerationSlew, driveControlOutputConstants.minDecelerationSlew, driveControlOutputConstants.maxDecelerationSlew);
    this->drivePID.setIntegralMethod(Decay);

    this->drivePID.setDerivative(0);
    this->drivePID.setMotorOutput(0);
    this->drivePID.setCounter(0);

    if(!prevChaining) {
        this->drivePID.setIntegral(0);
        this->drivePID.setPrevError(this->drivePID.startError);
        this->drivePID.setPrevMotorOutput(0);
    }

    else {
        this->drivePID.setIntegral(this->prevIntegral);
        this->drivePID.setPrevError(this->prevError);
        this->drivePID.setPrevMotorOutput(this->prevMotorOutput);
    }

    this->bearingPID.setError(0);
    this->bearingPID.setOutputConstants(bearingOutputConstants.kp, bearingOutputConstants.ki, bearingOutputConstants.kd);
    this->bearingPID.setLogicConstants(bearingLogicConstants.startIntegral);
    this->bearingPID.setSettleConstants(bearingSettleConstants.deadband, bearingSettleConstants.stallCounter, bearingSettleConstants.counterLimit);
    this->bearingPID.setControlOutputConstants(bearingControlOutputConstants.minMotorOutput, bearingControlOutputConstants.maxMotorOutput, bearingControlOutputConstants.accelerationSlew, bearingControlOutputConstants.minDecelerationSlew, bearingControlOutputConstants.maxDecelerationSlew);
    this->bearingPID.setIntegralMethod(Reset);

    this->bearingPID.setIntegral(0);
    this->bearingPID.setPrevError(this->bearingPID.startError);
    this->bearingPID.setDerivative(0);
    this->bearingPID.setMotorOutput(0);
    this->bearingPID.setCounter(0);
    this->bearingPID.setPrevMotorOutput(0);

    double driveOutput = 0;
    double bearingOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;

    do {
        headingDelta = getTurnError(this->odometry.globalOrientation, initialHeading);

        xDelta = this->odometry.xCoordinate - initialX ; //measures the change in global x-coordinate
        yDelta = this->odometry.yCoordinate - initialY; //measures the change in global y-coordinate

        distanceTraveled = xDelta * sin(degreesToRadians(initialHeading)) + yDelta * cos(degreesToRadians(initialHeading)); //adds on to distanceTraveled using the projection formula
        
        this->drivePID.error = this->drivePID.startError - distanceTraveled;
        this->bearingPID.error = headingDelta;

        driveOutput = this->drivePID.output();
        bearingOutput = this->bearingPID.output();

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
    } while(exit ? !this->drivePID.settled() : std::abs(this->drivePID.error) > this->drivePID.deadband);
    
    std::cout << "DrivePID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->drivePID.startError);
        this->prevError = this->drivePID.error;
        this->prevIntegral = this->drivePID.integral + this->bearingPID.integral;
        this->prevMotorOutput = this->drivePID.motorOutput + this->bearingPID.motorOutput;

        this->prevChaining = true;
    }

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;
};

void Drivetrain::driveToPoint(double finalX, double finalY, bool exit) {
    double targetDistance = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
    double targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //wraps the heading to [0, 360), which is necessary b/c atan2 returns a value in the ragne [-pi, pi), this is the heading facing the target
                                                                                                                                                   //atan is flipped b/c in normal coordinate systems, 0 degrees faces +X, but for our robot, 0 degrees faces +Y

    this->drivePID.setError(targetDistance);
    this->drivePID.setOutputConstants(driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd);
    this->drivePID.setLogicConstants(driveLogicConstants.startIntegral);
    this->drivePID.setSettleConstants(driveSettleConstants.deadband, driveSettleConstants.stallCounter, driveSettleConstants.counterLimit);
    this->drivePID.setControlOutputConstants(driveControlOutputConstants.minMotorOutput, driveControlOutputConstants.maxMotorOutput, driveControlOutputConstants.accelerationSlew, driveControlOutputConstants.minDecelerationSlew, driveControlOutputConstants.maxDecelerationSlew);
    this->drivePID.setIntegralMethod(Decay);

    this->drivePID.setDerivative(0);
    this->drivePID.setMotorOutput(0);
    this->drivePID.setCounter(0);

    if(!prevChaining) {
        this->drivePID.setIntegral(0);
        this->drivePID.setPrevError(this->drivePID.startError);
        this->drivePID.setPrevMotorOutput(0);
    }

    else {
        this->drivePID.setIntegral(this->prevIntegral);
        this->drivePID.setPrevError(this->prevError);
        this->drivePID.setPrevMotorOutput(this->prevMotorOutput);
    }

    this->driveTurnPID.setError(0);
    this->driveTurnPID.setOutputConstants(driveTurnOutputConstants.kp, driveTurnOutputConstants.ki, driveTurnOutputConstants.kd);
    this->driveTurnPID.setLogicConstants(driveTurnLogicConstants.startIntegral);
    this->driveTurnPID.setSettleConstants(driveTurnSettleConstants.deadband, driveTurnSettleConstants.stallCounter, driveTurnSettleConstants.counterLimit);
    this->driveTurnPID.setControlOutputConstants(driveTurnControlOutputConstants.minMotorOutput, driveTurnControlOutputConstants.maxMotorOutput, driveTurnControlOutputConstants.accelerationSlew, driveTurnControlOutputConstants.minDecelerationSlew, driveTurnControlOutputConstants.maxDecelerationSlew);
    this->driveTurnPID.setIntegralMethod(Reset);

    this->driveTurnPID.setDerivative(0);
    this->driveTurnPID.setMotorOutput(0);
    this->driveTurnPID.setCounter(0);

    if(!prevChaining) {
        this->driveTurnPID.setIntegral(0);
        this->driveTurnPID.setPrevError(this->driveTurnPID.startError);
        this->driveTurnPID.setPrevMotorOutput(0);
    }

    else {
        this->driveTurnPID.setIntegral(this->prevIntegral);
        this->driveTurnPID.setPrevError(this->prevError);
        this->driveTurnPID.setPrevMotorOutput(this->prevMotorOutput);
    }

    if(std::abs(getTurnError(this->odometry.globalOrientation, targetHeading)) > 90) {
        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, fmod((targetHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
    }

    else {
        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, targetHeading);
    }

    double driveTurnDeadband = radiansToDegrees(atan2(this->driveTurnSettleConstants.deadband, std::abs(this->drivePID.error))); //the heading error at which the robot is close enough to facing the target that it can stop turning
    bool locked = false;

    double driveOutput = 0;
    double driveTurnOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;

    do {
        this->drivePID.error = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
        targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //updates the heading facing the target
        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, targetHeading);

        driveOutput = this->drivePID.output() * cos(degreesToRadians(this->driveTurnPID.error)); //must calculate before recalculating driveTurnPID.error if going backward b/c that means that initial error is (90, 270), which means cosine will be negative
                                                                                     //also helps reduce forward/backward movement when heading is less correct, applies more power when facing in right direction

        if(std::abs(getTurnError(this->odometry.globalOrientation, targetHeading)) > 90) { //checks if the absolute value heading delta of the heading delta is greater than 90, meaning the target is behind the robot
            this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, fmod((targetHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
        }

        driveTurnOutput = this->driveTurnPID.output();

        driveTurnDeadband = radiansToDegrees(atan2(this->driveTurnSettleConstants.deadband, std::abs(this->drivePID.error)));
        if(this->drivePID.error < 2 * this->driveTurnPID.deadband || std::abs(this->driveTurnPID.error) < driveTurnDeadband) { //if heading is already close enough to facing the target or the drivePID is <3 inches away from target, stop turning
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
    } while(exit ? !this->drivePID.settled() : std::abs(this->drivePID.error) > this->drivePID.deadband);

    std::cout << "DriveToPointPID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->drivePID.startError);
        this->prevError = this->drivePID.error;
        this->prevIntegral = this->drivePID.integral + this->driveTurnPID.integral;
        this->prevMotorOutput = this->drivePID.motorOutput + this->driveTurnPID.motorOutput;

        this->prevChaining = true;
    }

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;
};

//turn to a global heading
void Drivetrain::turnToHeading(double targetHeading, bool exit) {
    this->turnPID.setError(0);
    this->turnPID.setOutputConstants(turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd);
    this->turnPID.setLogicConstants(turnLogicConstants.startIntegral);
    this->turnPID.setSettleConstants(turnSettleConstants.deadband, turnSettleConstants.stallCounter, turnSettleConstants.counterLimit);
    this->turnPID.setControlOutputConstants(turnControlOutputConstants.minMotorOutput, turnControlOutputConstants.maxMotorOutput, turnControlOutputConstants.accelerationSlew, turnControlOutputConstants.minDecelerationSlew, turnControlOutputConstants.maxDecelerationSlew);
    //this->turnPID.setIntegralMethod(Reset); //hard reset
    this->turnPID.setIntegralMethod(Decay);

    this->turnPID.setDerivative(0);
    this->turnPID.setMotorOutput(0);
    this->turnPID.setCounter(0);

    if(!prevChaining) {
        this->turnPID.setIntegral(0);
        this->turnPID.setPrevError(this->turnPID.startError);
        this->turnPID.setPrevMotorOutput(0);
    }

    else {
        this->turnPID.setIntegral(this->prevIntegral);
        this->turnPID.setPrevError(this->prevError);
        this->turnPID.setPrevMotorOutput(this->prevMotorOutput);
    }

    double turnOutput;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;

    do {
        this->turnPID.error = getTurnError(this->odometry.globalOrientation, targetHeading);

        turnOutput = this->turnPID.output();

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
    } while(exit ? !this->turnPID.settled() : std::abs(this->turnPID.error) > this->turnPID.deadband);


    std::cout << "TurnPID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->turnPID.startError);
        this->prevError = this->turnPID.error;
        this->prevIntegral = this->turnPID.integral;
        this->prevMotorOutput = this->turnPID.motorOutput;

        this->prevChaining = true;
    }

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;
};

void Drivetrain::turnToPoint(double finalX, double finalY, bool exit) {
    double targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //wraps the heading to [0, 360), which is necessary b/c atan2 returns a value in the ragne [-pi, pi), this is the heading facing the target
                                                                                                                                                   //atan is flipped b/c in normal coordinate systems, 0 degrees faces +X, but for our robot, 0 degrees faces +Y

    turnToHeading(targetHeading, exit);
};

void Drivetrain::swingToHeading(std::string Direction, double targetHeading, bool exit) {
    this->swingPID.setError(0);
    this->swingPID.setOutputConstants(turnOutputConstants.kp, turnOutputConstants.ki, turnOutputConstants.kd);
    this->swingPID.setLogicConstants(turnLogicConstants.startIntegral);
    this->swingPID.setSettleConstants(turnSettleConstants.deadband, turnSettleConstants.stallCounter, turnSettleConstants.counterLimit);
    this->swingPID.setControlOutputConstants(turnControlOutputConstants.minMotorOutput, turnControlOutputConstants.maxMotorOutput, turnControlOutputConstants.accelerationSlew, turnControlOutputConstants.minDecelerationSlew, turnControlOutputConstants.maxDecelerationSlew);
    this->swingPID.setIntegralMethod(Decay);

    this->swingPID.setDerivative(0);
    this->swingPID.setMotorOutput(0);
    this->swingPID.setCounter(0);

    if(!prevChaining) {
        this->swingPID.setIntegral(0);
        this->swingPID.setPrevError(this->swingPID.startError);
        this->swingPID.setPrevMotorOutput(0);
    }

    else {
        this->swingPID.setIntegral(this->prevIntegral);
        this->swingPID.setPrevError(this->prevError);
        this->swingPID.setPrevMotorOutput(this->prevMotorOutput);
    }

    double swingOutput;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;

    do {
        this->swingPID.error = getTurnError(this->odometry.globalOrientation, targetHeading);

        swingOutput = this->swingPID.output();

        leftOutput = percentToVolt(swingOutput);
        rightOutput = percentToVolt(swingOutput);

        if(this->leftVelocity < this->rightVelocity) {
            rightOutput *= this->leftVelocity / this->rightVelocity;
        }

        if(this->rightVelocity < this->leftVelocity) {
            leftOutput *= this->rightVelocity / this->leftVelocity;
        }

        if(Direction == "Right") { //swinging to right
            LeftDrive.spin(forward, leftOutput, voltageUnits::volt);
            RightDrive.stop(brake);
        }

        else { //swinging to left
            LeftDrive.stop(brake);
            RightDrive.spin(reverse, rightOutput, voltageUnits::volt);
        }

        vex::wait(20, msec);
    } while(exit ? !this->swingPID.settled() : std::abs(this->swingPID.error) > this->swingPID.deadband);


    std::cout << "TurnPID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->swingPID.startError);
        this->prevError = this->swingPID.error;
        this->prevIntegral = this->swingPID.integral;
        this->prevMotorOutput = this->swingPID.motorOutput;

        this->prevChaining = true;
    }

    std::cout << odometry.xCoordinate << ", " << odometry.yCoordinate << ", " << odometry.globalOrientation << std::endl;
};

void Drivetrain::moveToPose(double finalX, double finalY, double finalHeading, bool exit) {
    double targetDistance = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
    double targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //wraps the heading to [0, 360), which is necessary b/c atan2 returns a value in the ragne [-pi, pi), this is the heading facing the target
                                                                                                                                                   //atan is flipped b/c in normal coordinate systems, 0 degrees faces +X, but for our robot, 0 degrees faces +Y

    double driveHeadingDelta = getTurnError(this->odometry.globalOrientation, targetHeading);

    this->drivePID.setError(targetDistance);
    this->drivePID.setOutputConstants(driveOutputConstants.kp, driveOutputConstants.ki, driveOutputConstants.kd);
    this->drivePID.setLogicConstants(driveLogicConstants.startIntegral);
    this->drivePID.setSettleConstants(driveSettleConstants.deadband, driveSettleConstants.stallCounter, driveSettleConstants.counterLimit);
    this->drivePID.setControlOutputConstants(driveControlOutputConstants.minMotorOutput, driveControlOutputConstants.maxMotorOutput, driveControlOutputConstants.accelerationSlew, driveControlOutputConstants.minDecelerationSlew, driveControlOutputConstants.maxDecelerationSlew);
    this->drivePID.setIntegralMethod(Decay);

    this->drivePID.setDerivative(0);
    this->drivePID.setMotorOutput(0);
    this->drivePID.setCounter(0);

    if(!prevChaining) {
        this->drivePID.setIntegral(0);
        this->drivePID.setPrevError(this->drivePID.startError);
        this->drivePID.setPrevMotorOutput(0);
    }

    else {
        this->drivePID.setIntegral(this->prevIntegral);
        this->drivePID.setPrevError(this->prevError);
        this->drivePID.setPrevMotorOutput(this->prevMotorOutput);
    }

    this->driveTurnPID.setError(0);
    this->driveTurnPID.setOutputConstants(driveTurnOutputConstants.kp, driveTurnOutputConstants.ki, driveTurnOutputConstants.kd);
    this->driveTurnPID.setLogicConstants(driveTurnLogicConstants.startIntegral);
    this->driveTurnPID.setSettleConstants(driveTurnSettleConstants.deadband, driveTurnSettleConstants.stallCounter, driveTurnSettleConstants.counterLimit);
    this->driveTurnPID.setControlOutputConstants(driveTurnControlOutputConstants.minMotorOutput, driveTurnControlOutputConstants.maxMotorOutput, driveTurnControlOutputConstants.accelerationSlew, driveTurnControlOutputConstants.minDecelerationSlew, driveTurnControlOutputConstants.maxDecelerationSlew);
    this->driveTurnPID.setIntegralMethod(Reset);

    this->driveTurnPID.setDerivative(0);
    this->driveTurnPID.setMotorOutput(0);
    this->driveTurnPID.setCounter(0);

    if(!prevChaining) {
        this->driveTurnPID.setIntegral(0);
        this->driveTurnPID.setPrevError(this->driveTurnPID.startError);
        this->driveTurnPID.setPrevMotorOutput(0);
    }

    else {
        this->driveTurnPID.setIntegral(this->prevIntegral);
        this->driveTurnPID.setPrevError(this->prevError);
        this->driveTurnPID.setPrevMotorOutput(this->prevMotorOutput);
    }

    if(std::abs(getTurnError(this->odometry.globalOrientation, targetHeading)) > 90) {
        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, fmod((targetHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
    }

    else {
        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, targetHeading);
    }

    bool locked = false;

    double driveOutput = 0;
    double driveTurnOutput = 0;

    //left and right outputs
    double leftOutput = 0;
    double rightOutput = 0;

    do {
        this->drivePID.error = hypot((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate));
        targetHeading = fmod(radiansToDegrees(atan2((finalX - this->odometry.xCoordinate), (finalY - this->odometry.yCoordinate))) + 360, 360); //updates the heading facing the target
        driveHeadingDelta = getTurnError(this->odometry.globalOrientation, targetHeading);

        this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, finalHeading);

        driveOutput = this->drivePID.output() * cos(degreesToRadians(driveHeadingDelta)); //must calculate before recalculating driveTurnPID.error if going backward b/c that means that initial error is (90, 270), which means cosine will be negative
                                                                                     //also helps reduce forward/backward movement when heading is less correct, applies more power when facing in right direction

        if(std::abs(getTurnError(this->odometry.globalOrientation, finalHeading)) > 90) { //checks if the absolute value heading delta of the heading delta is greater than 90, meaning the target is behind the robot
            this->driveTurnPID.error = getTurnError(this->odometry.globalOrientation, fmod((finalHeading + 180), 360)); //makes |error| <= 90 b/c it treats the back of the robot as the front
        }

        driveTurnOutput = this->driveTurnPID.output() * (1.0 - this->drivePID.error / this->drivePID.startError); //scales up the turn output as the robot gets closer to the target

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
    } while(exit ? !this->drivePID.settled() : std::abs(this->drivePID.error) > this->drivePID.deadband);

    std::cout << "DriveToPointPID is done!" << std::endl;

    if(exit) {
        LeftDrive.stop(hold);
        RightDrive.stop(hold);

        this->prevChaining = false;
    }

    else {
        this->prevDirection = getSign(this->drivePID.startError);
        this->prevError = this->drivePID.error;
        this->prevIntegral = this->drivePID.integral + this->driveTurnPID.integral;
        this->prevMotorOutput = this->drivePID.motorOutput + this->driveTurnPID.motorOutput;

        this->prevChaining = true;
    }
};

//==== Pure Pursuit Function ====//
void Drivetrain::followPath(PurePursuit& pathToFollow, double targetSpeed) {
    double currentX = this->odometry.xCoordinate;
    double currentY = this->odometry.yCoordinate;
    double currentHeading = odometry.globalOrientation;

    double prevX = this->odometry.xCoordinate;
    double prevY = this->odometry.yCoordinate;  

    double distanceToGoal = hypot((pathToFollow.path[pathToFollow.path.size() - 1].x - currentX), (pathToFollow.path[pathToFollow.path.size() - 1].y - currentY));

    double driveCounter = 0;

    double leftOutput = 0;
    double rightOutput = 0;

    while((distanceToGoal > pathToFollow.lookaheadDistance) || (driveCounter < this->drivePID.counterLimit)) {
        currentX = this->odometry.xCoordinate;
        currentY = this->odometry.yCoordinate;
        currentHeading = this->odometry.globalOrientation;

        Waypoint lookaheadPoint = pathToFollow.getLookaheadPoint(currentX, currentY);

        double curvature = pathToFollow.computeCurvature(currentX, currentY, currentHeading, lookaheadPoint);

        std::vector<double> wheelSpeeds = pathToFollow.computeWheelSpeeds(curvature, targetSpeed);

        leftOutput = wheelSpeeds[0];
        rightOutput = wheelSpeeds[1];

        if(this->leftVelocity < this->rightVelocity) {
            rightOutput *= this->leftVelocity / this->rightVelocity;
        }

        if(this->rightVelocity < this->leftVelocity) {
            leftOutput *= this->rightVelocity / this->leftVelocity;
        }

        LeftDrive.spin(forward, leftOutput, voltageUnits::volt); //left output should be in volts -> target speed should be in volts
        RightDrive.spin(forward, rightOutput, voltageUnits::volt); //right output should be in volts -> target speed should be in volts

        distanceToGoal = hypot((pathToFollow.path[pathToFollow.path.size() - 1].x - currentX), (pathToFollow.path[pathToFollow.path.size() - 1].y - currentY));

        prevX = currentX;
        prevY = currentY;

        vex::wait(20, msec);
    }

    LeftDrive.stop(hold);
    RightDrive.stop(hold);

    driveToPoint(pathToFollow.path[pathToFollow.path.size() - 1].x, pathToFollow.path[pathToFollow.path.size() - 1].y, true);
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