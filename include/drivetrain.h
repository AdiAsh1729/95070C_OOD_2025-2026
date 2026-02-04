#pragma once

#include "vex.h"
#include "custom_controller.h"
#include "odometry.h"
#include "pid.h"
#include "type_defines.h"
#include "util.h"
#include <iostream>
using namespace vex;

class Drivetrain {
    private:
        Custom_Controller& Controller;
        motor_group& LeftDrive;
        motor_group& RightDrive;
        rotation& HorizontalTracker;
        rotation& VerticalTracker;
        inertial& Inertial;

        double gearRatio;
        double wheelDiameter;
        double distanceToSideOfDrivetrain;
        double horizontalWheelDiameter;
        double verticalWheelDiameter;
        double horizontalToCenterDistance;
        double verticalToCenterDistance;

        OdometryType odometryType;

        //Trampoline method setup
        static int trackPositionStatic(void* instance); //the actual static function that is passed into the task positionTrackingTask
        int trackPosition(); //makes trackPosition a local function that can access odometry, Inertial, etc., removing the need for extern
        
        task positionTrackingTask;

    public:
        Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, inertial& Inertial, double gearRatio, double wheelDiameter);
        Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, inertial& Inertial, double gearRatio, double wheelDiameter, double distanceToSideOfDrivetrain);
        Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, rotation& HorizontalTracker, inertial& Inertial, double gearRatio,  double wheelDiameter, double distanceToSideOfDrivetrain, double horizontalWheelDiameter, double horizontalToCenterDistance);
        Drivetrain(Custom_Controller& Controller, motor_group& LeftDrive, motor_group& RightDrive, rotation& HorizontalTracker, rotation& VerticalTracker, inertial& Inertial, double gearRatio,  double wheelDiameter, double distanceToSideOfDrivetrain, double horizontalWheelDiameter, double verticalWheelDiameter, double horizontalToCenterDistance, double verticalToCenterDistance);

        Odometry odometry;

        void calibrateInertial();
        void resetInertialHeading();

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
            double previousError; //not in PID constructor
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
            double previousMotorOutput; //not in PID constructor
            double minimumMotorOutput;
            double maximumMotorOutput;
            double slewRate;
        };

        //==== DRIVE DISTANCE CONSTANTS ====//
        outputConstants driveDistanceOutputConstants;
        logicConstants driveDistanceLogicConstants;
        settleConstants driveDistanceSettleConstants;
        controlOutputConstants driveDistanceControlOutputConstants;

        outputConstants bearingOutputConstants;
        logicConstants bearingLogicConstants;
        settleConstants bearingSettleConstants;
        controlOutputConstants bearingControlOutputConstants;


        //==== DRIVE DISTANCE SLOW CONSTANTS ====// 
        outputConstants driveDistanceSlowOutputConstants;
        logicConstants driveDistanceSlowLogicConstants;
        settleConstants driveDistanceSlowSettleConstants;
        controlOutputConstants driveDistanceSlowControlOutputConstants;

        outputConstants bearingSlowOutputConstants;
        logicConstants bearingSlowLogicConstants;
        settleConstants bearingSlowSettleConstants;
        controlOutputConstants bearingSlowControlOutputConstants;


        //==== TURN TO HEADING CONSTANTS ====//
        outputConstants turnToHeadingOutputConstants;
        logicConstants turnToHeadingLogicConstants;
        settleConstants turnToHeadingSettleConstants;
        controlOutputConstants turnToHeadingControlOutputConstants;

        
        //==== ARC TO HEADING CONSTANTS ====//
        outputConstants arcToHeadingOutputConstants;
        logicConstants arcToHeadingLogicConstants;
        settleConstants arcToHeadingSettleConstants;
        controlOutputConstants arcToHeadingControlOutputConstants;



        //==== PID MOVEMENTS====//
        void driveDistance(double targetDistance);
        void driveDistanceV(double targetDistance);

        void driveDistanceSlow(double targetDistance);
        void driveDistanceSlowV(double targetDistance);

        void turnToHeading(double targetHeading);

        void arcToHeading(double radius, double targetHeading);
        void arcToHeadingV(double radius, double targetHeading);

        void oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation); //moves the drivetrain forward and backward for matchloading


        //==== DRIVER CONTROL ====//
        void runArcadeDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed);
        void runTankDrive(Button SlowButtonID, double scaleDriveSpeed, double scaleTurnSpeed);
};