#pragma once

#include "util.h"
#include <cmath>

class Odometry {
    private:
        double horizontalToCenterDistance; //in inches
        double verticalToCenterDistance; //in inches
        double previousHorizontalTrackerPosition; //in inches
        double previousVerticalTrackerPosition; //in inches
        double previousGlobalOrientationRadians; //in degrees

    public:
        Odometry(double horizontalToCenterDistance, double verticalToCenterDistance);

        double xCoordinate;
        double yCoordinate;
        double globalOrientation; //in degrees

        //for full odometry system
        void setPosition(double xCoordinate, double yCoordinate, double globalOrientation);
        
        void updatePosition(double horizontalTrackerPosition, double verticalTrackerPosition, double globalOrientation); //tracker positions will be in inches
};