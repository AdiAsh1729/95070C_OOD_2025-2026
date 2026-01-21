#include "intake.h"

//Fixes:
/*
make pressed work instead of pressing for motors
*/

Intake::Intake(Custom_Controller& Controller, motor& Intake1, motor& Intake2, motor_group& IntakeGroup, digital_out& Descore, digital_out& Matchloader, digital_out& MiddleGoal):
    Controller(Controller),
    Intake1(Intake1),
    Intake2(Intake2),
    IntakeGroup(IntakeGroup),
    Descore(Descore),
    Matchloader(Matchloader),
    MiddleGoal(MiddleGoal)
{};

void Intake::runIfPressed1(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);
    
    /*if(Controller.pressed(ButtonForwardID)) {
        Intake1.spin(forward, forwardVelocityPct, pct);
    }

    if(Controller.pressed(ButtonReverseID)) {
        Intake1.spin(reverse, reverseVelocityPct, pct);
    }

    if(Controller.pressed(ButtonStopID)) {
        Intake1.stop();
    }*/

    if(ButtonForward.pressing()) {
        //std::cout << "Button Forward" << std::endl;
        Intake1.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        //std::cout << "Button Reverse" << std::endl;
        Intake1.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        //std::cout << "Button Stop" << std::endl;
        Intake1.stop();
    }
};

void Intake::runIfPressed2(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);

    /*if(Controller.pressed(ButtonForwardID)) {
        Intake2.spin(forward, forwardVelocityPct, pct);
    }

    if(Controller.pressed(ButtonReverseID)) {
        Intake2.spin(reverse, reverseVelocityPct, pct);
    }

    if(Controller.pressed(ButtonStopID)) {
        Intake2.stop();
    }*/

    if(ButtonForward.pressing()) {
        //std::cout << "Button Forward" << std::endl;
        Intake2.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        //std::cout << "Button Reverse" << std::endl;
        Intake2.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        //std::cout << "Button Stop" << std::endl;
        Intake2.stop();
    }
};

void Intake::runIfPressedGroup(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);

    /*if(Controller.pressed(ButtonForwardID)) {
        std::cout << "Button Forward" << std::endl;
        IntakeGroup.spin(forward, forwardVelocityPct, pct);
    }

    if(Controller.pressed(ButtonReverseID)) {
        std::cout << "Button Reverse" << std::endl;
        IntakeGroup.spin(reverse, reverseVelocityPct, pct);
    }

    if(Controller.pressed(ButtonStopID)) {
        std::cout << "Button Stop" << std::endl;
        IntakeGroup.stop();
    }*/

    if(ButtonForward.pressing()) {
        //std::cout << "Button Forward" << std::endl;
        IntakeGroup.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        //std::cout << "Button Reverse" << std::endl;
        IntakeGroup.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        //std::cout << "Button Stop" << std::endl;
        IntakeGroup.stop();
    }
};

void Intake::setDescore(bool state) {
    Descore.set(state);
};

void Intake::setMatchloader(bool state) {
    Matchloader.set(state);
};

void Intake::setMiddleGoal(bool state)  {
    MiddleGoal.set(state);
};

void Intake::toggleDescore(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        descorePressed++;
    }
    std::cout << Button(ToggleButtonID) << std::endl;
    if(descorePressed % 2 == 0) {
        Descore.set(false);
        std::cout<<"Setting Descore to False"<<std::endl;
    }

    else {
        Descore.set(true);
        std::cout<<"Setting Descore to True"<<std::endl;
    }
};

void Intake::toggleMatchloader(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        matchloaderPressed++;
    }

    if(matchloaderPressed % 2 == 0) {
        Matchloader.set(false);
    }

    else {
        Matchloader.set(true);
    }
};

void Intake::toggleMiddleGoal(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        middleGoalPressed++;
    }

    if(descorePressed % 2 == 0) {
        MiddleGoal.set(false);
    }

    else {
        MiddleGoal.set(true);
    }
};

void Intake::oscillate(int numberOfOscillations, double speedOfOscillation, double timeOfOscillation) {
    for(int i = 0; i < numberOfOscillations; i++) {
        IntakeGroup.spin(forward, speedOfOscillation, pct);
        wait(timeOfOscillation, msec);
        IntakeGroup.spin(reverse, speedOfOscillation, pct);
        if(i < numberOfOscillations - 1) {
            wait(timeOfOscillation, msec);
        }
    }
}