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
    MiddleGoal(MiddleGoal),
    descorePressed(0),
    matchloaderPressed(0),
    middleGoalPressed(0)
{};

//==== Driver Control ====//
void Intake::runIfPressed1(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);

    if(ButtonForward.pressing()) {
        //std::cout << "Button Forward" << std::endl;
        Intake1.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        Intake1.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        Intake1.stop();
    }
};

void Intake::runIfPressed2(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);

    if(ButtonForward.pressing()) {
        Intake2.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        Intake2.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        Intake2.stop();
    }
};

void Intake::runIfPressedGroup(Button ButtonForwardID, Button ButtonReverseID, Button ButtonStopID, int forwardVelocityPct, int reverseVelocityPct) {
    const controller::button& ButtonForward = Controller.getButton(ButtonForwardID);
    const controller::button& ButtonReverse = Controller.getButton(ButtonReverseID);
    const controller::button& ButtonStop = Controller.getButton(ButtonStopID);

    if(ButtonForward.pressing()) {
        IntakeGroup.spin(forward, forwardVelocityPct, pct);
    }

    if(ButtonReverse.pressing()) {
        IntakeGroup.spin(reverse, reverseVelocityPct, pct);
    }

    if(ButtonStop.pressing()) {
        IntakeGroup.stop();
    }
};

void Intake::toggleDescore(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        Descore.set(!Descore.value());
    }
};

void Intake::toggleMatchloader(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        Matchloader.set(!Matchloader.value());
    }
};

void Intake::toggleMiddleGoal(Button ToggleButtonID) {
    if(Controller.pressed(ToggleButtonID)) {
        MiddleGoal.set(!MiddleGoal.value());
    }
};

bool Intake::getMiddleGoal() {
    return MiddleGoal.value();
}

//==== Pre-Auton / Auton ====//
void Intake::setDescore(bool state) {
    Descore.set(state);
};

void Intake::setMatchloader(bool state) {
    Matchloader.set(state);
};

void Intake::setMiddleGoal(bool state)  {
    MiddleGoal.set(state);
};

//==== Auton ====//
void Intake::run1(directionType direction, double velocity, percentUnits units) {
    Intake1.spin(direction, velocity, units);
};

void Intake::run2(directionType direction, double velocity, percentUnits units) {
    Intake2.spin(direction, velocity, units);
};

void Intake::runGroup(directionType direction, double velocity, percentUnits units) {
    IntakeGroup.spin(direction, velocity, units);
};

void Intake::stop1() {
    Intake1.stop();
};

void Intake::stop2() {
    Intake2.stop();
};

void Intake::stopGroup() {
    IntakeGroup.stop();
};

void Intake::intakeStore(double speed1, double speed2) {
    MiddleGoal.set(false);

    Intake1.spin(forward, speed1, voltageUnits::volt);
    Intake2.spin(forward, speed2, voltageUnits::volt);
}

void Intake::scoreLong(double speed1, double speed2) {
    MiddleGoal.set(false);

    Intake1.spin(forward, speed1, voltageUnits::volt);
    Intake2.spin(reverse, speed2, voltageUnits::volt);
};

void Intake::scoreMiddle(double speed1, double speed2) {
    MiddleGoal.set(true);

    Intake1.spin(forward, speed1, voltageUnits::volt);
    Intake2.spin(forward, speed2, voltageUnits::volt);
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