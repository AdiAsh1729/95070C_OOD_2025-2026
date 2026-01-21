#include "robot_config.h"

brain Brain; //our brain
Custom_Controller Controller1 = Custom_Controller(); //our controller

Custom_Motor FL = Custom_Motor(PORT4, ratio6_1, true); //our front left motor
Custom_Motor ML = Custom_Motor(PORT6, ratio6_1, true); //our middle left motor
Custom_Motor BL = Custom_Motor(PORT3, ratio6_1, true); //our back left motor
motor_group LeftDrive = motor_group(FL, ML, BL); //a motor group consisting of the motors on the left side of our drivetrain
Custom_Motor FR = Custom_Motor(PORT10, ratio6_1, false); //our front right motor
Custom_Motor MR = Custom_Motor(PORT13, ratio6_1, false); //our middle right motor
Custom_Motor BR = Custom_Motor(PORT16, ratio6_1, false); //our back right motor
motor_group RightDrive = motor_group(FR, MR, BR); //a motor group consisting of the motors on the right side of our drivetrain

motor Intake1 = motor(PORT5, ratio6_1, false); //the motor controlling our first stage intake
motor Intake2 = motor(PORT14, ratio6_1, true); //the motor controlling our second stage intake
motor_group IntakeGroup = motor_group(Intake1, Intake2); //the motor group consisting of the motors controlling our intake

digital_out Descore = digital_out(Brain.ThreeWirePort.A); //the pneumatics controlling both our bunny ears mech and our middle goal scoring mech simultaneously
digital_out Matchloader = digital_out(Brain.ThreeWirePort.H); //the pneumatics controlling our matchloader
digital_out MiddleGoal = digital_out(Brain.ThreeWirePort.A); //the pneumatics controlling both our bunny ears mech and our middle goal scoring mech simultaneously

inertial Inertial = inertial(PORT16); //our inertial sensor

//Port 6 is saved for DummyRotation

Drivetrain chassis(Controller1, LeftDrive, RightDrive, Inertial, GEAR_RATIO, WHEEL_DIAMETER);
Intake intake(Controller1, Intake1, Intake2, IntakeGroup, Descore, Matchloader, MiddleGoal);
Robot robot(Controller1, chassis, intake);

void vexcodeInit(void) {

} 