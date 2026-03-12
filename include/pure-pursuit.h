#pragma once

#include "util.h"

#include <cmath>
#include <vector>

struct Waypoint {
    double x;
    double y;
};

class PurePursuit {
    private:

    public:
        double wheelbase; //distance between left and right wheels, used for calculating the curvature of the robot's path to the lookahead point and thus the left and right wheel speeds
        double lookaheadDistance; //the distance the robot looks ahead on the path to find the lookahead point, we tune this
        std::vector<Waypoint>& path;

        double leftSpeed;
        double rightSpeed;

        PurePursuit(double wheelbase, double lookaheadDistance, std::vector<Waypoint>& path);

        Waypoint getLookaheadPoint(double currentX, double currentY);
        double computeCurvature(double currentX, double currentY, double currentOrienation, Waypoint lookaheadPoint);
        std::vector<double> computeWheelSpeeds(double curvature, double targetSpeed);
};