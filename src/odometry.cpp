#include "odometry.h"

Odometry::Odometry(double horizontalToCenterDistance, double verticalToCenterDistance):
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance)
{};

void Odometry::setPosition(double xCoordinate, double yCoordinate, double globalOrientation) {
    this->previousHorizontalTrackerPosition = 0;
    this->previousVerticalTrackerPosition = 0;
    this->previousGlobalOrientationRadians = degreesToRadians(globalOrientation);
    this->xCoordinate = xCoordinate;
    this->yCoordinate = yCoordinate;
    this->globalOrientation = globalOrientation;
}

void Odometry::updatePosition(double horizontalTrackerPosition, double verticalTrackerPosition, double globalOrientation) {
    double horizontalTrackerDelta = horizontalTrackerPosition - this->previousHorizontalTrackerPosition;
    double verticalTrackerDelta = verticalTrackerPosition - this->previousVerticalTrackerPosition;
    this->globalOrientation = globalOrientation;
    double globalOrientationRadians = degreesToRadians(this->globalOrientation);
    double globalOrientationDeltaRadians = globalOrientationRadians - this->previousGlobalOrientationRadians;

    double localXOffset = 0.0;
    double localYOffset = 0.0;

    double localPolarAngle = 0.0;
    double globalPolarAngle = 0.0;
    double polarLength = 0.0;

    double xCoordinateDelta = 0.0;
    double yCoordinateDelta = 0.0;

    const double epsilon = 1e-6;

    if(globalOrientationDeltaRadians < epsilon) {
        localXOffset = horizontalTrackerDelta;
        localYOffset = verticalTrackerDelta;
    }

    else {
        localXOffset = 2 * sin(globalOrientationDeltaRadians/2.0) * ((horizontalTrackerDelta / globalOrientationDeltaRadians) + horizontalToCenterDistance);
        localYOffset = 2 * sin(globalOrientationDeltaRadians/2.0) * ((verticalTrackerDelta / globalOrientationDeltaRadians) + verticalToCenterDistance);
    }

    localPolarAngle = atan2(localYOffset, localXOffset);
    globalPolarAngle = localPolarAngle - this->previousGlobalOrientationRadians - globalOrientationDeltaRadians / 2;
    polarLength = sqrt(pow(localXOffset, 2) + pow(localYOffset, 2));

    xCoordinateDelta = cos(globalPolarAngle) * polarLength;
    yCoordinateDelta = sin(globalPolarAngle) * polarLength;

    this->xCoordinate += xCoordinateDelta;
    this->yCoordinate += yCoordinateDelta;

    this->previousHorizontalTrackerPosition = horizontalTrackerPosition;
    this->previousVerticalTrackerPosition = verticalTrackerPosition;
    this->previousGlobalOrientationRadians = globalOrientationRadians;
}