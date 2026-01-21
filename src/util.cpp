#include "util.h"

double getSign(double input) {
    return (input > 0) - (input < 0);
}

double getTurnError(double heading, double targetHeading) {
    if(std::max(heading, targetHeading) - std::min(heading, targetHeading) <= 180) {
        return targetHeading - heading;
    }

    else {
        if(heading > targetHeading) {
            return targetHeading - heading + 360;
        }

        else {
            return targetHeading - heading - 360;
        }
    }
}

double clamp(double input, double minimumValue, double maximumValue) {
    double sign = getSign(input);

    if(std::abs(input) < minimumValue) {
        return minimumValue * sign;
    }

    else if(std::abs(input) > maximumValue) {
        return maximumValue * sign;
    }

    else {
        return input;
    }
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180;
}

double radiansToDegrees(double radians) {
    return radians * 180 / M_PI;
}

double getGreaterArcDistance(double headingDelta, double radius, double distanceToSideOfDrivetrain) {
    return 2 * M_PI * (radius + distanceToSideOfDrivetrain);
}

double getLesserArcDistance(double headingDelta, double radius, double distanceToSideOfDrivetrain) {
    return 2 * M_PI * (radius - distanceToSideOfDrivetrain);
}