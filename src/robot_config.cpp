#include "robot_config.h"

brain Brain; //our brain
Custom_Controller Controller1 = Custom_Controller(); //our controller

Custom_Motor FL = Custom_Motor(PORT13, ratio6_1, true); //our front left motor
Custom_Motor ML = Custom_Motor(PORT19, ratio6_1, true); //our middle left motor
Custom_Motor BL = Custom_Motor(PORT10, ratio6_1, true); //our back left motor
Custom_Motor_Group LeftDrive = Custom_Motor_Group(BL, ML, FL); //a motor group consisting of the motors on the left side of our drivetrain
Custom_Motor FR = Custom_Motor(PORT2, ratio6_1, false); //our front right motor
Custom_Motor MR = Custom_Motor(PORT14, ratio6_1, false); //our middle right motor
Custom_Motor BR = Custom_Motor(PORT5, ratio6_1, false); //our back right motor
Custom_Motor_Group RightDrive = Custom_Motor_Group(BR, MR, FR); //a motor group consisting of the motors on the right side of our drivetrain

motor Intake1 = motor(PORT3, ratio6_1, false); //the motor controlling our first stage intake
motor Intake2 = motor(PORT1, ratio6_1, false); //the motor controlling our second stage intake
motor_group IntakeGroup = motor_group(Intake1, Intake2); //the motor group consisting of the motors controlling our intake

digital_out Descore = digital_out(Brain.ThreeWirePort.E); //the pneumatics controlling both our bunny ears mech and our middle goal scoring mech simultaneously
digital_out Matchloader = digital_out(Brain.ThreeWirePort.B); //the pneumatics controlling our matchloader
digital_out MiddleGoal = digital_out(Brain.ThreeWirePort.G); //the pneumatics controlling both our bunny ears mech and our middle goal scoring mech simultaneously

inertial Inertial = inertial(PORT17); //our inertial sensor

rotation HorizontalTracker = rotation(PORT9);

//Port 6 is saved for DummyRotation in drivetrain.cpp

Drivetrain chassis(Controller1, LeftDrive, RightDrive, HorizontalTracker, Inertial, INERTIAL_SCALE, GEAR_RATIO, LEFT_VELOCITY, RIGHT_VELOCITY, HORIZONTAL_WHEEL_DIAMETER, VERTICAL_WHEEL_DIAMETER, HORIZONTAL_TO_CENTER_DISTANCE);
Intake intake(Controller1, Intake1, Intake2, IntakeGroup, Descore, Matchloader, MiddleGoal);
Robot robot(Controller1, chassis, intake);

void vexcodeInit(void) {

} 