#include "pure-pursuit.h"

PurePursuit::PurePursuit(double wheelbase, double lookaheadDistance, std::vector<Waypoint>& path):
    wheelbase(wheelbase),
    lookaheadDistance(lookaheadDistance),
    path(path),

    leftSpeed(0),
    rightSpeed(0)
{};

Waypoint PurePursuit::getLookaheadPoint(double currentX, double currentY) {
    for(size_t i = 0; i < path.size() - 1; i++) {
            Waypoint prevWaypoint = path[i]; //previous waypoint
            Waypoint nextWaypoint = path[i + 1]; //next waypoint

            double waypointXDelta = nextWaypoint.x - prevWaypoint.x; //change in x between the previous and next waypoint, used for the parametric equations of the line between the waypoints
            double waypointYDelta = nextWaypoint.y - prevWaypoint.y; //change in y between the previous and next waypoint, used for the parametric equations of the line between the waypoints

            double xDelta = prevWaypoint.x - currentX; //change in x between the robot and the current waypoint, used for the parametric equations of the line between the waypoints
            double yDelta = prevWaypoint.y - currentY; //change in y between the robot and the current waypoint, used for the parametric equations of the line between the waypoints

            double a = pow(waypointXDelta, 2) + pow(waypointYDelta, 2); //coefficient of t^2 in the quadratic formula when you plug the parametric equations of the line between the waypoints into the equation of a circle with a radius of the lookahead distance centered at the robot's position
            double b = 2 * (xDelta * waypointXDelta + yDelta * waypointYDelta); //coefficient of t in the quadratic formula when you plug the parametric equations of the line between the waypoints into the equation of a circle with a radius of the lookahead distance centered at the robot's position
            double c = pow(xDelta, 2) + pow(yDelta, 2) - pow(lookaheadDistance, 2); //the constant in the quadratic formula when you plug the parametric equations of the line between the waypoints into the equation of a circle with a radius of the lookahead distance centered at the robot's position

            double discriminant = pow(b, 2) - 4 * a * c; //the discriminant of the quadratic formula, if its negative, then there are no intersections between the line between the waypoints and the circle centered at the robot with a radius of the lookahead distance, so we can skip this waypoint

            if(discriminant < 0) {
                continue;
            }

            double intersection1 = (-b + sqrt(discriminant)) / (2 * a);
            double intersection2 = (-b - sqrt(discriminant)) / (2 * a);

            bool intersection1Valid = (intersection1 >= 0 && intersection1 <= 1);
            bool intersection2Valid = (intersection2 >= 0 && intersection2 <= 1);

            double distToIntersection = -1; //the distance between the robot and the intersection point, used to determine which intersection point is the lookahead point if there are two valid intersection points

            if(intersection1Valid && intersection2Valid) {
                distToIntersection = std::max(intersection1, intersection2);
            }

            if(intersection1Valid && !intersection2Valid) {
                distToIntersection = intersection1;
            }

            if(!intersection1Valid && intersection2Valid) {
                distToIntersection = intersection2;
            }

            if(distToIntersection >= 0) {
                Waypoint lookaheadPoint;
                lookaheadPoint.x = prevWaypoint.x + distToIntersection * waypointXDelta;
                lookaheadPoint.y = prevWaypoint.y + distToIntersection * waypointYDelta;
                return lookaheadPoint;
            }
        }

    return path.back();
};

double PurePursuit::computeCurvature(double currentX, double currentY, double currentOrienation, Waypoint lookaheadPoint) {
    double currentOrientationRadians = degreesToRadians(currentOrienation);

    double xDelta = lookaheadPoint.x - currentX;
    double yDelta = lookaheadPoint.y - currentY;

    double localX =  cos(currentOrientationRadians) * xDelta + sin(currentOrientationRadians) * yDelta;
    double localY = -sin(currentOrientationRadians) * xDelta + cos(currentOrientationRadians) * yDelta;

    if (localY == 0) {
        return 0;
    }
    return (2 * localX) / (pow(localX, 2) + pow(localY, 2)); // curvature
}

std::vector<double> PurePursuit::computeWheelSpeeds(double curvature, double targetSpeed) {
    double leftSpeed = targetSpeed * (2 + curvature * this->wheelbase) / 2;
    double rightSpeed = targetSpeed * (2 - curvature * this->wheelbase) / 2;

    return {leftSpeed, rightSpeed};
}