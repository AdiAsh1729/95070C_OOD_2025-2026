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
    intake.setDescore(false);
    intake.setMatchloader(false);
    intake.setMiddleGoal(false);
};

//NOTE: It's better to use a variable and toggle it between 1 and 0 (true and false) rather than use break, which leaves many open threads because it is effectively a force stop

void Robot::preAuton() {
    setPistons(); //set the pistons to their initial state so the robot stays in size

    // Calibrate Inertial Sensors
    chassis.calibrateInertial();
    wait(3000, msec);

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
                Controller.rumble("-");
                driverSelectorLoopVariable = 0 ; //exits the loop, a cleaner break
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
            intake.runIfPressed2(R1, R2, B, 0, 99);

            if(!intake.getMiddleGoal()) { //if scoring on long goal
                intake.runIfPressed1(L1, L2, B, 99, 99);
                intake.runIfPressed2(L2, L1, B, 99, 99);
            }
            
            else {
                if(CurrentAuton == Null) {
                    intake.runIfPressedGroup(L1, L2, B, 65, 99);
                }

                else {
                    intake.runIfPressedGroup(L1, L2, B, 99, 99);
                }
            }

            break;

        case Thomas:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressed2(R1, R2, B, 0, 99);

            if(!intake.getMiddleGoal()) { //if scoring on long goal
                intake.runIfPressed1(L1, L2, B, 99, 99);
                intake.runIfPressed2(L2, L1, B, 99, 99);
            }
            
            else {
                if(CurrentAuton == Null) {
                    intake.runIfPressedGroup(L1, L2, B, 65, 99);
                }

                else {
                    intake.runIfPressedGroup(L1, L2, B, 99, 99);
                }
            }

            break;

        case Shaurya:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressed2(R1, R2, B, 0, 99);

            if(!intake.getMiddleGoal()) { //if scoring on long goal
                intake.runIfPressed1(L1, L2, B, 99, 99);
                intake.runIfPressed2(L2, L1, B, 99, 99);
            }
            
            else {
                if(CurrentAuton == Null) {
                    intake.runIfPressedGroup(L1, L2, B, 65, 99);
                }

                else {
                    intake.runIfPressedGroup(L1, L2, B, 99, 99);
                }
            }

            break;

        case Noob:
            intake.runIfPressed1(R1, R2, B, 99, 99);
            intake.runIfPressed2(R1, R2, B, 0, 99);

            if(!intake.getMiddleGoal()) { //if scoring on long goal
                intake.runIfPressed1(L1, L2, B, 99, 99);
                intake.runIfPressed2(L2, L1, B, 99, 99);
            }
            
            else {
                if(CurrentAuton == Null) {
                    intake.runIfPressedGroup(L1, L2, B, 65, 99);
                }

                else {
                    intake.runIfPressedGroup(L1, L2, B, 99, 99);
                }
            }

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
            intake.toggleMiddleGoal(Y);
            
            runDrive();
            runIntake();

            wait(20, msec);
        }
    }
};

//==== Robot Autonomous Functions ====//
void Robot::setPIDConstants() {
    //==== How To Tune ====//
    /*
    Step 1: Tune kp until the robot just overshoots, tune to the nearest 10th
    Step 2: Increase kd until the robot movement is smoother, either good enough or just undershoots
    Step 3 (optional): Increase ki, but it will be very small, ~0.001 - 0.002

    Step 1: For the bearing PID: tune a slow but consistent drive PID
    Step 2: Set the ki to 0
    Step 3: Increase kp to the nearest 10th, you can test by nudging the robot
    Step 4: Might need to tune kd if the bearing PID isn't smooth
    */

    //all min/max motor outputs and slew rates should be in pct
    //note that lowering the P value in PID functions basically increases the amount of distance the PID gets to decelerate

    //==== Drive Distance Constants ====//
    chassis.driveDistanceOutputConstants.kp = 9;
    chassis.driveDistanceOutputConstants.ki = 0;
    chassis.driveDistanceOutputConstants.kd = 0.6;

    chassis.driveDistanceLogicConstants.startIntegral = 10;

    chassis.driveDistanceSettleConstants.deadband = 0.5;
    chassis.driveDistanceSettleConstants.stallCounter = 0.1;
    chassis.driveDistanceSettleConstants.counterLimit = 25;

    chassis.driveDistanceControlOutputConstants.minimumMotorOutput = 0; 
    chassis.driveDistanceControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveDistanceControlOutputConstants.accelerationSlew = 99.9; //might have to tune this
    chassis.driveDistanceControlOutputConstants.minDecelerationSlew = 0; //might have to tune this
    chassis.driveDistanceControlOutputConstants.maxDecelerationSlew = 99.9; //might have to tune this


    //used for keeping bearing when using driveDistance
    chassis.bearingOutputConstants.kp = 0.38;
    chassis.bearingOutputConstants.ki = 0;
    chassis.bearingOutputConstants.kd = 0;

    chassis.bearingLogicConstants.startIntegral = 20;

    chassis.bearingSettleConstants.deadband = 3;
    chassis.bearingSettleConstants.stallCounter = 0;
    chassis.bearingSettleConstants.counterLimit = 0;

    chassis.bearingControlOutputConstants.minimumMotorOutput = 0;
    chassis.bearingControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.bearingControlOutputConstants.accelerationSlew = 99.9;
    chassis.bearingControlOutputConstants.minDecelerationSlew = 0;
    chassis.bearingControlOutputConstants.maxDecelerationSlew = 99.9;

    //used for driveToPoint
    chassis.driveTurnOutputConstants.kp = 3.25;
    chassis.driveTurnOutputConstants.ki = 0;
    chassis.driveTurnOutputConstants.kd = 0.3;

    chassis.driveTurnLogicConstants.startIntegral = 20;

    chassis.driveTurnSettleConstants.deadband = 3;
    chassis.driveTurnSettleConstants.stallCounter = 0;
    chassis.driveTurnSettleConstants.counterLimit = 0;

    chassis.driveTurnControlOutputConstants.minimumMotorOutput = 7.7;
    chassis.driveTurnControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveTurnControlOutputConstants.accelerationSlew = 99.9;
    chassis.driveTurnControlOutputConstants.minDecelerationSlew = 0;
    chassis.driveTurnControlOutputConstants.maxDecelerationSlew = 99.9;


    //==== Drive Distance Slow Constants ====//
    chassis.driveDistanceSlowOutputConstants.kp = 0;
    chassis.driveDistanceSlowOutputConstants.ki = 0;
    chassis.driveDistanceSlowOutputConstants.kd = 0;

    chassis.driveDistanceSlowLogicConstants.startIntegral = 10;

    chassis.driveDistanceSlowSettleConstants.deadband = 0.25;
    chassis.driveDistanceSlowSettleConstants.stallCounter = 0.1;
    chassis.driveDistanceSlowSettleConstants.counterLimit = 25;

    chassis.driveDistanceSlowControlOutputConstants.minimumMotorOutput = 7.7;
    chassis.driveDistanceSlowControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveDistanceSlowControlOutputConstants.accelerationSlew = 99.9; //might have to tune this
    chassis.driveDistanceSlowControlOutputConstants.minDecelerationSlew = 0; //might have to tune this
    chassis.driveDistanceSlowControlOutputConstants.maxDecelerationSlew = 99.9; //might have to tune this

    //used for keeping bearing when using driveDistanceSlow
    chassis.bearingSlowOutputConstants.kp = 0;
    chassis.bearingSlowOutputConstants.ki = 0;
    chassis.bearingSlowOutputConstants.kd = 0;

    chassis.bearingSlowLogicConstants.startIntegral = 20;

    chassis.bearingSlowSettleConstants.deadband = 3;
    chassis.bearingSlowSettleConstants.stallCounter = 0;
    chassis.bearingSlowSettleConstants.counterLimit = 0;

    chassis.bearingSlowControlOutputConstants.minimumMotorOutput = 0;
    chassis.bearingSlowControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.bearingSlowControlOutputConstants.accelerationSlew = 99.9;
    chassis.bearingSlowControlOutputConstants.minDecelerationSlew = 0;
    chassis.bearingSlowControlOutputConstants.maxDecelerationSlew = 99.9;

    //used for keeping heading when using driveToPointSlow
    chassis.driveTurnSlowOutputConstants.kp = 0;
    chassis.driveTurnSlowOutputConstants.ki = 0;
    chassis.driveTurnSlowOutputConstants.kd = 0;

    chassis.driveTurnSlowLogicConstants.startIntegral = 20;

    chassis.driveTurnSlowSettleConstants.deadband = 3;
    chassis.driveTurnSlowSettleConstants.stallCounter = 0;
    chassis.driveTurnSlowSettleConstants.counterLimit = 0;

    chassis.driveTurnSlowControlOutputConstants.minimumMotorOutput = 7.7;
    chassis.driveTurnSlowControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.driveTurnSlowControlOutputConstants.accelerationSlew = 99.9;
    chassis.driveTurnSlowControlOutputConstants.minDecelerationSlew = 0;
    chassis.driveTurnSlowControlOutputConstants.maxDecelerationSlew = 99.9;

    //==== Turn To Heading Constants ====//
    chassis.turnToHeadingOutputConstants.kp = 3.25;
    chassis.turnToHeadingOutputConstants.ki = 0;
    chassis.turnToHeadingOutputConstants.kd = 0.3;

    chassis.turnToHeadingLogicConstants.startIntegral = 20;

    chassis.turnToHeadingSettleConstants.deadband = 3;
    chassis.turnToHeadingSettleConstants.stallCounter = 0.5;
    chassis.turnToHeadingSettleConstants.counterLimit = 25;

    chassis.turnToHeadingControlOutputConstants.minimumMotorOutput = 7.7;
    chassis.turnToHeadingControlOutputConstants.maximumMotorOutput = 99.9;
    chassis.turnToHeadingControlOutputConstants.accelerationSlew = 99.9;
    chassis.turnToHeadingControlOutputConstants.minDecelerationSlew = 0;
    chassis.turnToHeadingControlOutputConstants.maxDecelerationSlew = 99.9;
};

void Robot::RightQual() {
    setPIDConstants();
    chassis.setCoordinates(0, 0, 0);

    intake.intakeStore(99, 0); //start running intake
    //chassis.driveDistance(12, true);
    chassis.driveToPoint(0, 12, true);
    //chassis.turnToHeading(45, true);
    chassis.turnToPoint(5, 17, true);
    //chassis.driveDistance(7, true);
    chassis.driveToPoint(5, 17, true);  //collect first 1-2 blocks
    //chassis.driveDistance(8, true);
    chassis.driveToPoint(10.65, 22.65, true); //finish collecting 3 blocks in the L
    intake.setMatchloader(true);
    chassis.turnToPoint(36.5, 0, true);
    chassis.driveToPoint(36.5, 0, true); //drive to matchloader accurately
    chassis.turnToHeading(180, true); //turn to matchloader
    wait(500, msec);
    chassis.driveDistance(12, true); //stop right before matchloader

    //matchloading
    chassis.leftSpin(forward, 40, pct);
    chassis.rightSpin(forward, 40, pct);
    wait(300, msec);
    chassis.leftSpin(forward, 5, pct);
    chassis.rightSpin(forward, 5, pct);
    wait(500, msec);
    intake.stopGroup();

    //going and scoring in long goal
    //drive back to long goal which is slightly offset from matchloader
    chassis.driveToPoint(37.5, 16, true);

    chassis.leftSpin(reverse, 99, pct);
    chassis.rightSpin(reverse, 99, pct);
    wait(100, msec);
    intake.setMatchloader(false); //put matchloader up
    intake.scoreLong(12, 12); //score in long goal
    wait(3000, msec);
    intake.stopGroup();
    chassis.stop();
    chassis.driveDistance(8, true);
    chassis.turnToHeading(90, true);
    chassis.driveDistance(14, true);
    chassis.setInertialHeading(90);
    chassis.turnToHeading(180, true);
    chassis.driveDistance(-24, true); //push into control
};

void Robot::LeftQual() {
    setPIDConstants();
    chassis.setCoordinates(0, 0, 0);

    //while(true) {
    //    std::cout << chassis.odometry.xCoordinate << ", " << chassis.odometry.yCoordinate << ", " << chassis.odometry.globalOrientation << ", " << chassis.getInertialHeading() << std::endl;
    //    wait(100, msec);
    //}

    chassis.driveToPoint(24, 24, true);
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