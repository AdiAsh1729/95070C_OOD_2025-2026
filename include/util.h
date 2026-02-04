#pragma once

#include "vex.h"
#include <cmath>
#include <iostream>
using namespace vex;

double percentToVolt(double percent);
double getSign(double input);
double getTurnError(double heading, double target);
double clamp(double input, double minimumValue, double maximumValue);
double degreesToRadians(double degrees);
double radiansToDegrees(double radians);
double getGreaterArcDistance(double headingDelta, double radius, double distanceToSideOfDrivetrain);
double getLesserArcDistance(double headingDelta, double radius, double distanceToSideOfDrivetrain);