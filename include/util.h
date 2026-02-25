#pragma once

#include "vex.h"
#include <cmath>
#include <iostream>
using namespace vex;

double clamp(double input, double minimumValue, double maximumValue);
double degreesToRadians(double degrees);
double getSign(double input);
double getTurnError(double heading, double target); //returns in range [-180, 180]
double percentToVolt(double percent);
double radiansToDegrees(double radians);