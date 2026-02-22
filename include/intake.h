#pragma once

#include "vex.h"
#include "custom_controller.h"
#include <iostream> //for debugging

using namespace vex;

class Intake {
    private:
        Custom_Controller& Controller;
        motor& Intake1;
        motor& Intake2;
        motor_group& IntakeGroup;
        digital_out& Descore;
        digital_out& Matchloader;
        digital_out& MiddleGoal;
        
        int descorePressed;
        int matchloaderPressed;
        int middleGoalPressed;

    public:
        Intake(Custom_Controller& Controller, motor& Intake1, motor& Intake2, motor_group& IntakeGroup, digital_out& Descore, digital_out& Matchloader, digital_out& MiddleGoal);

        //==== Driver Control ====//
        void runIfPressed1(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);
        void runIfPressed2(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);
        void runIfPressedGroup(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);

        void toggleDescore(Button ToggleButtonID);
        void toggleMatchloader(Button ToggleButtonID); 
        void toggleMiddleGoal(Button ToggleButtonID);

        bool getMiddleGoal();

        //==== Pre-Auton/Auton ====//
        void setDescore(bool state);
        void setMatchloader(bool state);
        void setMiddleGoal(bool state);

        //==== Auton ====//
        void run1(directionType direction, double velocity, percentUnits units);
        void run2(directionType direction, double velocity, percentUnits units);
        void runGroup(directionType direction, double velocity, percentUnits units);
        
        void stop1();
        void stop2();
        void stopGroup();

        void intakeStore(double speed1, double speed2);
        void scoreLong(double speed1, double speed2);
        void scoreMiddle(double speed1, double spee2);

        void oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation);
};