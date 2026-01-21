#include "robot.h"

Robot::Robot(Custom_Controller& Controller, Drivetrain& chassis, Intake& intake):
    Controller(Controller),
    chassis(chassis),
    intake(intake),
    runningPreAuton(true)
{};

//==== Robot Selector Functions ====//
void Robot::displayAuton() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    switch(CurrentAuton) {
        case Null:
            Controller.Screen.print("No Auton Selected");
            break;

        case RunRightQual:
            Controller.Screen.print("Right Qual");
            break;

        case RunLeftQual:
            Controller.Screen.print("Left Qual");
            break;

        case RunRightElim:
            Controller.Screen.print("Right Elim");
            break;

        case RunLeftElim:
            Controller.Screen.print("Left Elim");
            break;

        case RunSoloAWP:
            Controller.Screen.print("Solo AWP");
            break;

        case RunAutonSkills:
            Controller.Screen.print("Auton Skills");
            break;
    }
};

void Robot::changeAutonRight() {
    displayAuton();
    switch(CurrentAuton) {
        case Null:
            CurrentAuton = RunRightQual;
            displayAuton();
            break;

        case RunRightQual:
            CurrentAuton = RunLeftQual;
            displayAuton();
            break;

        case RunLeftQual:
            CurrentAuton = RunRightElim;
            displayAuton();
            break;

        case RunRightElim:
            CurrentAuton = RunLeftElim;
            displayAuton();
            break;

        case RunLeftElim:
            CurrentAuton = RunSoloAWP;
            displayAuton();
            break;

        case RunSoloAWP:
            CurrentAuton = RunAutonSkills;
            displayAuton();
            break;

        case RunAutonSkills:
            CurrentAuton = Null;
            displayAuton();
            break;
    }
};

void Robot::changeAutonLeft() {
    displayAuton();
    switch(CurrentAuton) {
        case Null:
            CurrentAuton = RunAutonSkills;
            displayAuton();
            break;

        case RunRightQual:
            CurrentAuton = Null;
            displayAuton();
            break;

        case RunLeftQual:
            CurrentAuton = RunRightQual;
            displayAuton();
            break;

        case RunRightElim:
            CurrentAuton = RunLeftQual;
            displayAuton();
            break;

        case RunLeftElim:
            CurrentAuton = RunRightElim;
            displayAuton();
            break;

        case RunSoloAWP:
            CurrentAuton = RunLeftElim;
            displayAuton();
            break;

        case RunAutonSkills:
            CurrentAuton = RunSoloAWP;
            displayAuton();
            break;
    }
};

void Robot::displayDriver() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    switch(CurrentDriver) {
        case Nikhil:
            Controller.Screen.print("Nikhil");
            break;

        case Thomas:
            Controller.Screen.print("Thomas");
            break;

        case Shaurya:
            Controller.Screen.print("Shaurya");
            break;

        case Noob:
            Controller.Screen.print("Noob");
            break;
    }
};

void Robot::changeDriverRight() {
    displayDriver();
    switch(CurrentDriver) {
        case Nikhil:
            CurrentDriver = Thomas;
            displayDriver();
            break;

        case Thomas:
            CurrentDriver = Shaurya;
            displayDriver();
            break;

        case Shaurya:
            CurrentDriver = Noob;
            displayDriver();
            break;

        case Noob:
            CurrentDriver = Nikhil;
            displayDriver();
            break;
    }
};

void Robot::changeDriverLeft() {
    displayDriver();
    switch(CurrentDriver) {
        case Nikhil:
            CurrentDriver = Noob;
            displayDriver();
            break;

        case Thomas:
            CurrentDriver = Nikhil;
            displayDriver();
            break;

        case Shaurya:
            CurrentDriver = Thomas;
            displayDriver();
            break;

        case Noob:
            CurrentDriver = Shaurya;
            displayDriver();
            break;
    }
}

//==== Robot Pre-Autonomous Functions ====//
void Robot::setPistons() {
    intake.setDescore(true);
    intake.setMatchloader(false);
    intake.setMiddleGoal(true);
};

//NOTE: It's better to use a variable and toggle it between 1 and 0 (true and false) rather than use break, which leaves many open threads because it is effectively a force stop

void Robot::preAuton() {
    setPistons(); //set the pistons to their initial state so the robot stays in size

    // Calibrate Inertial Sensors
    chassis.calibrateInertial();
    wait(3000, msec);
    chassis.resetInertialHeading();

    displayAuton();

    int autonSelectorLoopVariable = 1;
    int driverSelectorLoopVariable = 1;

    //Select Auton
    while(autonSelectorLoopVariable == 1) {
        if(Controller.pressed(Right)) {
            changeAutonRight();
        }

        if(Controller.pressed(Left)) {
            changeAutonLeft();
        }

        if(Controller.pressed(A)) {
            Controller.resetPressed();
            Controller.rumble("-");
            autonSelectorLoopVariable = 0; //exits the loop, a cleaner break
        }

        wait(20, msec);
    }

    wait(1000, msec);
    
    //Checks if running DriverSkills(if) or Auton(else)
    if(CurrentAuton == Null) {
        displayDriver();

        //Select Driver
        while(driverSelectorLoopVariable == 1) {
            if(Controller.pressed(Right)) {
                changeDriverRight();
            }

            if(Controller.pressed(Left)) {
                changeDriverLeft();
            }

            if(Controller.pressed(A)) {
                Controller.resetPressed();
                Controller.rumble("-");
                //std::cout << "PreAuton ended1" << std::endl;
                //runningPreAuton = false;
                //std::cout << runningPreAuton << std::endl;
                driverSelectorLoopVariable = 0 ; //exits the loop, a cleaner break
                //return;
            }

            wait(20, msec);
        }
    }

    //else {
        while(!Controller.allReleased()) { //wait for hardware and software to sync
            wait(10, msec);
            std::cout << "Waiting for all released" << std::endl;
        }
        std::cout << "Exited while loop" << std::endl;
        Controller.resetPressed();
        //std::cout << "PreAuton ended2" << std::endl;
        //std::cout << runningPreAuton << std::endl;
        wait(20, msec);
        runningPreAuton = false;
        return;
    //}
};

//==== Robot Driver Control Functions ====//
void Robot::runDrive() {
    switch(CurrentDriver) {
        case Nikhil:
            chassis.runArcadeDrive(Up, 0.70, 0.30);
            break;

        case Thomas:
            chassis.runArcadeDrive(Up, -0.40, 0.30);
            break;

        case Shaurya:
            chassis.runArcadeDrive(Up, 0.45, 0.4367911);
            break;

        case Noob:
            chassis.runArcadeDrive(Up, 0.10, 0.10);
            break;
    }
};

void Robot::runIntake() {
    switch(CurrentDriver) {
        case Nikhil:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressedGroup(L1, L2, B, 99, 99);
            break;

        case Thomas:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressedGroup(L1, L2, B, 99, 99);
            break;

        case Shaurya:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressedGroup(L1, L2, B, 99, 99);
            break;

        case Noob:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressedGroup(L1, L2, B, 99, 99);
            break;
    }
};

void Robot::driverControl() {
    //std::cout << runningPreAuton << std::endl;
    if(!runningPreAuton) {
        while(true) {
            //std::cout << "DriverControl started!" << std::endl;
            intake.toggleDescore(A);
            intake.toggleMatchloader(X);
            intake.toggleMiddleGoal(A);
            
            runDrive();
            runIntake();

            wait(20, msec);
        }
    }
};

//==== Robot Autonomous Functions ====//
void Robot::setPIDConstants() {
    //==== Drive Distance Constants ====//
    chassis.driveDistanceOutputConstants.kp = 0.75;
    chassis.driveDistanceOutputConstants.ki = 0.001;
    chassis.driveDistanceOutputConstants.kd = 0.65;

    chassis.driveDistanceLogicConstants.startIntegral = 10;

    chassis.driveDistanceSettleConstants.deadband = 0.5;
    chassis.driveDistanceSettleConstants.stallCounter = 0.2;
    chassis.driveDistanceSettleConstants.counterLimit = 25;

    chassis.driveDistanceControlOutputConstants.minimumMotorOutput = 0.5;
    chassis.driveDistanceControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveDistanceControlOutputConstants.slewRate = 2;


    chassis.bearingOutputConstants.kp = 0;
    chassis.bearingOutputConstants.ki = 0;
    chassis.bearingOutputConstants.kd = 0;

    chassis.bearingLogicConstants.startIntegral = 10;

    chassis.bearingSettleConstants.deadband = 3;
    chassis.bearingSettleConstants.stallCounter = 0;
    chassis.bearingSettleConstants.counterLimit = 0;

    chassis.bearingControlOutputConstants.minimumMotorOutput = 0;
    chassis.bearingControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.bearingControlOutputConstants.slewRate = 99.9;


    //==== Drive Distance Slow Constants ====//
    chassis.driveDistanceSlowOutputConstants.kp = 0;
    chassis.driveDistanceSlowOutputConstants.ki = 0;
    chassis.driveDistanceSlowOutputConstants.kd = 0;

    chassis.driveDistanceSlowLogicConstants.startIntegral = 10;

    chassis.driveDistanceSlowSettleConstants.deadband = 0.5;
    chassis.driveDistanceSlowSettleConstants.stallCounter = 0.2;
    chassis.driveDistanceSlowSettleConstants.counterLimit = 25;

    chassis.driveDistanceSlowControlOutputConstants.minimumMotorOutput = 0.5;
    chassis.driveDistanceSlowControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveDistanceSlowControlOutputConstants.slewRate = 2;


    chassis.bearingSlowOutputConstants.kp = 0;
    chassis.bearingSlowOutputConstants.ki = 0;
    chassis.bearingSlowOutputConstants.kd = 0;

    chassis.bearingSlowLogicConstants.startIntegral = 10;

    chassis.bearingSlowSettleConstants.deadband = 3;
    chassis.bearingSlowSettleConstants.stallCounter = 0;
    chassis.bearingSlowSettleConstants.counterLimit = 0;

    chassis.bearingSlowControlOutputConstants.minimumMotorOutput = 0;
    chassis.bearingSlowControlOutputConstants.maximumMotorOutput = 10;
    chassis.bearingSlowControlOutputConstants.slewRate = 99.9;


    //==== Turn To Heading Constants ====//
    chassis.turnToHeadingOutputConstants.kp = 0.3;
    chassis.turnToHeadingOutputConstants.ki = 0.049;
    chassis.turnToHeadingOutputConstants.kd = 0.5;

    chassis.turnToHeadingLogicConstants.startIntegral = 20;

    chassis.turnToHeadingSettleConstants.deadband = 3;
    chassis.turnToHeadingSettleConstants.stallCounter = 5;
    chassis.turnToHeadingSettleConstants.counterLimit = 25;

    chassis.turnToHeadingControlOutputConstants.minimumMotorOutput = 0.5;
    chassis.turnToHeadingControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.turnToHeadingControlOutputConstants.slewRate = 99.9;
};

void Robot::RightQual() {
    chassis.driveDistance(48);
};

void Robot::LeftQual() {

};

void Robot::RightElim() {

};

void Robot::LeftElim() {

};

void Robot::SoloAWP() {

};

void Robot::AutonSkills() {

};

void Robot::autonomous() {
    timer autonTimer;
    autonTimer.clear();

    switch(CurrentAuton) {
        case Null:
            break;

        case RunRightQual:
            RightQual();
            break;

        case RunLeftQual:
            LeftQual();
            break;

        case RunRightElim:
            RightElim();
            break;

        case RunLeftElim:
            LeftElim();
            break;

        case RunSoloAWP:
            SoloAWP();
            break;

        case RunAutonSkills:
            AutonSkills();
            break;
    }

    std::cout << autonTimer.time(msec) << std::endl;
};