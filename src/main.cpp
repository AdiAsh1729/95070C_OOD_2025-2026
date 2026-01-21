#include "vex.h"
#include "robot.h"
#include "robot_config.h"

competition Competition;

void driverControl() {
    robot.driverControl();
}

void autonomous() {
    robot.autonomous();
}

int main() {
    robot.preAuton(); //Calibrate Inertial sensor, select auton, select driver

    Competition.drivercontrol(driverControl); //If switch is in Driver Control run DriverControl
    Competition.autonomous(autonomous); //If switch is autonomous run autonomous

    while(true) {
        wait(20, msec);
    }
}