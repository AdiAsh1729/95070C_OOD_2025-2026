#pragma once

#include "vex.h"
#include "custom_controller.h"
#include "custom_motor.h" 
#include "drivetrain.h"
#include "intake.h"
#include "robot.h"
#include "type_defines.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

extern brain Brain;
extern Custom_Controller Controller;

extern Custom_Motor FL;
extern Custom_Motor ML;
extern Custom_Motor BL;
extern Custom_Motor_Group LeftDrive;
extern Custom_Motor FR;
extern Custom_Motor MR;
extern Custom_Motor BR;
extern Custom_Motor_Group RightDrive;

extern motor Intake1;
extern motor Intake2;
extern motor_group IntakeGroup;

extern digital_out Descore_MiddleGoal;
extern digital_out Matchloader;
extern digital_out Park;

extern inertial Inertial;

extern rotation HorizontalTracker;

extern Drivetrain chassis;
extern Intake intake;
extern Robot robot;

void vexcodeInit(void);