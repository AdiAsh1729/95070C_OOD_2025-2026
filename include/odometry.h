#pragma once

#include "util.h"

class Odometry {
    private:
        double horizontalToCenterDistance;
        double verticalToCenterDistance;
        double previousHorizontalTrackerPosition;
        double previousVerticalTrackerPosition;
        double previousGlobalOrientationRadians;

    public:
        Odometry(double horizontalToCenterDistance, double verticalToCenterDistance);

        double xCoordinate;
        double yCoordinate;
        double globalOrientation;

        void setPosition(double xCoordinate, double yCoordinate, double globalOrientation);

        void updatePosition(double horizontalTrackerPosition, double verticalTrackerPosition, double globalOrientation); //tracker positions will be in inches
};