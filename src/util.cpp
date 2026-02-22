#include "util.h"

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

/*double distanceToLine(double pointX, double pointY, double heading, double xCoordinate, double yCoordinate) {
    double slope;
    double perpendicularSlope;

    double yIntercept;
    double perpendicularYIntercept;

    double slopeDelta;
    double yInterceptDelta;

    double xIntersection;
    double yIntersection;

    if(std::abs(heading) != 180) {
        slope = cos(heading)/sin(heading); //heading (in the range [0, 360))

        //eq. of OG line is y = (slope * x) - (slope * pointX - pointY)
        //eq. of perpendicular line passing through (pointX, pointY) is y = ((-1/slope) * x) - ((-1/slope) * xCoordinate - yCoordinate)
        perpendicularSlope = -1/slope;
        yIntercept = pointY - slope * pointX;
        perpendicularYIntercept = yCoordinate - xCoordinate / slope;

        xIntersection = 1;
        yIntersection = 1;

        return 0;
    }

    return 0;
}*/

double getSign(double input) {
    return (input > 0) - (input < 0);
}

double getTurnError(double heading, double targetHeading) { //returns in range [-180, 180]
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

double percentToVolt(double percent) {
    return percent * 12 / 100;
}

double radiansToDegrees(double radians) {
    return radians * 180 / M_PI;
}