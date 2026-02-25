#pragma once

#include "vex.h"
#include "custom_controller.h"
#include "custom_motor.h"
#include "odometry.h"
#include "pid.h"
#include "type_defines.h"
#include "util.h"
#include <iostream>
using namespace vex;

class Drivetrain {
    private:
        Custom_Controller& Controller;
        Custom_Motor_Group& LeftDrive;
        Custom_Motor_Group& RightDrive;
        rotation& HorizontalTracker;
        rotation& VerticalTracker;
        inertial& Inertial;

        double inertialScale;
        double gearRatio; //actually the reciprocal of the gear ratio
        //one of leftVelocity and rightVelocity should be 1
        double leftVelocity; //find this using ratio code in driver control
        double rightVelocity; //find this using ratio code in driver control
        double horizontalWheelDiameter; //in inches
        double verticalWheelDiameter; //in inches
        double horizontalToCenterDistance; //inches
        double verticalToCenterDistance; //inches

        OdometryType odometryType;

        //==== Global PIDs ====//
        PID drivePID;
        PID bearingPID;
        PID driveTurnPID;
        PID turnPID;
        PID swingPID;

        //==== Chaining ====//
        PIDType prevPIDType;

        bool prevChaining; //checks if previous move was chaining
        double prevDirection; //which direction LeftDrive moved, 1 if forward, -1 if reverse
        double prevError; //needed for smooth derivative calculation
        double prevIntegral; //previous motion's integral
        double prevMotorOutput; //previous motion's motor output

        //==== Trampoline Method Setup ====//
        static int trackPositionStatic(void* instance); //the actual static function that is passed into the task positionTrackingTask
        int trackPosition(); //makes trackPosition a local function that can access odometry, Inertial, etc., removing the need for extern
        
        task positionTrackingTask;

    public:
        Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double verticalWheelDiameter);
        Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, rotation& HorizontalTracker, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance);
        Drivetrain(Custom_Controller& Controller, Custom_Motor_Group& LeftDrive, Custom_Motor_Group& RightDrive, rotation& HorizontalTracker, rotation& VerticalTracker, inertial& Inertial, double inertialScale, double gearRatio, double leftVelocity, double rightVelocity, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance);

        Odometry odometry;

        void calibrateInertial();
        void setInertialHeading(double heading);
        double getInertialHeading();

        void resetHorizontalTrackerPosition();
        void resetVerticalTrackerPosition();
        double getHorizontalTrackerPositionInches();
        double getVerticalTrackerPositionInches();

        void setCoordinates(double initialXCoordinate, double initialYCoordinate, double initialGlobalOrientation);

        typedef struct outputConstants {
            double kp;
            double ki;
            double kd;
        };

        typedef struct logicConstants {
            double error; //not in PID constructor
            double startIntegral;
            double integral; //not in PID constructor
            double prevError; //not in PID constructor
            double derivative; //not in PID constructor
        };

        typedef struct settleConstants {
            double deadband;
            double motorOutput; //not in PID constructor
            double stallCounter;
            double counterLimit;
            double counter;
        };

        typedef struct controlOutputConstants {
            double prevMotorOutput; //not in PID constructor
            double minMotorOutput;
            double maxMotorOutput;
            double accelerationSlew;
            double minDecelerationSlew;
            double maxDecelerationSlew;
        };

        //==== DRIVE DISTANCE CONSTANTS ====//
        outputConstants driveOutputConstants;
        logicConstants driveLogicConstants;
        settleConstants driveSettleConstants;
        controlOutputConstants driveControlOutputConstants;

        outputConstants bearingOutputConstants;
        logicConstants bearingLogicConstants;
        settleConstants bearingSettleConstants;
        controlOutputConstants bearingControlOutputConstants;

        //driveToPoint turning constants
        outputConstants driveTurnOutputConstants;
        logicConstants driveTurnLogicConstants;
        settleConstants driveTurnSettleConstants;
        controlOutputConstants driveTurnControlOutputConstants;


        //==== DRIVE DISTANCE SLOW CONSTANTS ====// 
        outputConstants driveSlowOutputConstants;
        logicConstants driveSlowLogicConstants;
        settleConstants driveSlowSettleConstants;
        controlOutputConstants driveSlowControlOutputConstants;

        outputConstants bearingSlowOutputConstants;
        logicConstants bearingSlowLogicConstants;
        settleConstants bearingSlowSettleConstants;
        controlOutputConstants bearingSlowControlOutputConstants;

        //driveToPointSlow turning constants
        outputConstants driveTurnSlowOutputConstants;
        logicConstants driveTurnSlowLogicConstants;
        settleConstants driveTurnSlowSettleConstants;
        controlOutputConstants driveTurnSlowControlOutputConstants;


        //==== TURN TO HEADING CONSTANTS ====//
        outputConstants turnOutputConstants;
        logicConstants turnLogicConstants;
        settleConstants turnSettleConstants;
        controlOutputConstants turnControlOutputConstants;



        //==== PID MOVEMENTS====//
        void driveDistance(double targetDistance, bool exit);
        void driveToPoint(double xCoordinate, double yCoordinate, bool exit);

        void driveDistanceSlow(double targetDistance, bool exit);
        void driveToPointSlow(double finalX, double finalY, bool exit);

        void turnToHeading(double targetHeading, bool exit);
        void turnToPoint(double finalX, double finalY, bool exit);
        void swingToHeading(std::string Direction, double targetHeading, bool exit);
        void swingToPoint(double finalX, double finalY, bool exit);

        void oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation); //moves the drivetrain forward and backward for matchloading

        //==== AUTON MOVEMENTS ====//
        void leftSpin(directionType direction, int velocity, percentUnits units);
        void rightSpin(directionType direction, int velocity, percentUnits units);
        void stop();

        //==== DRIVER CONTROL ====//
        void runArcadeDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed);
        void runTankDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed);
};