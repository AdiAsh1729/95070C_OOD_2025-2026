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

        void runIfPressed1(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);
        void runIfPressed2(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);
        void runIfPressedGroup(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct);

        void setDescore(bool state);
        void setMatchloader(bool state);
        void setMiddleGoal(bool state);

        void toggleDescore(Button ToggleButtonID);
        void toggleMatchloader(Button ToggleButtonID); 
        void toggleMiddleGoal(Button ToggleButtonID);

        void oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation);
};