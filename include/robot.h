#pragma once

#include "vex.h"
#include "custom_controller.h"
#include "custom_motor.h"
#include "drivetrain.h"
#include "intake.h"
#include "pure-pursuit.h"

class Robot {
    private:
        Custom_Controller& Controller;
        Drivetrain& chassis;
        Intake& intake;
        bool runningPreAuton;

        Auton CurrentAuton;
        Driver CurrentDriver;

    public:
        Robot(Custom_Controller& Controller, Drivetrain& chassis, Intake& intake);

        //==== Robot Selector Functions ====//
        void displayAuton();
        void changeAutonRight();
        void changeAutonLeft();

        void displayDriver();
        void changeDriverRight();
        void changeDriverLeft();

        //==== Robot Pre-Autonomous Functions ====//
        void setPistons();
        void preAuton();

        //==== Robot Driver Control Functions ====//
        void runDrive();
        void runIntake();

        void driverControl();

        //==== Robot Autonomous Functions ====//
        void setPIDConstants();
        
        void RightQual();
        void LeftQual();
        void RightElim();
        void LeftElim();
        void SoloAWP();
        void AutonSkills();

        void autonomous();
};