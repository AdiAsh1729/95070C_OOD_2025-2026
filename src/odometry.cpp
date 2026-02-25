#include "odometry.h"

Odometry::Odometry(double horizontalToCenterDistance, double verticalToCenterDistance):
    horizontalToCenterDistance(horizontalToCenterDistance),
    verticalToCenterDistance(verticalToCenterDistance)
{};

void Odometry::setPosition(double xCoordinate, double yCoordinate, double globalOrientation) {
    this->prevHorizontalTrackerPosition = 0;
    this->prevVerticalTrackerPosition = 0;
    this->prevGlobalOrientationRadians = degreesToRadians(globalOrientation);
    this->xCoordinate = xCoordinate;
    this->yCoordinate = yCoordinate;
    this->globalOrientation = globalOrientation;
}

void Odometry::updatePosition(double horizontalTrackerPosition, double verticalTrackerPosition, double globalOrientation) {
    double horizontalTrackerDelta = horizontalTrackerPosition - this->prevHorizontalTrackerPosition; //in inches, calculates change in horizontal encoder's movement
    double verticalTrackerDelta = verticalTrackerPosition - this->prevVerticalTrackerPosition; //in inches, calculates change in vertical encoder's movement

    //need wraparound for global orientation b/c if there is a sudden jump in inertial from (< 360 -> > 0), or (> 0 -> < 360). then position will spike
    //odometry looks for the least change in heading, so you want to clamp between -180 and 180
    this->globalOrientation = globalOrientation; //in degrees updates the current global orientation
    double globalOrientationRadians = degreesToRadians(this->globalOrientation); //converts global orientation to radians for trig calculations
    double globalOrientationDeltaRadians = fmod((globalOrientationRadians - this->prevGlobalOrientationRadians) + M_PI, 2 * M_PI) - M_PI; //radians, calculates change in global orientation
                                                                                                                                              //wraparound function, add pi, then take it mod 2pi so that its in the range [0, 2pi), but then you subtract pi for the pi you added before, so its in the range [-pi, pi)
                                                                                                                                              //wrap function works for normal turns too, its just that if you cross the 360 barrier, then it applies and corrects it


    double localXOffset = 0.0; //initializes local x change
    double localYOffset = 0.0; //initializes local y change

    double localPolarAngle = 0.0; //initializes local polar angle
    double globalPolarAngle = 0.0; //initializes global polar angle
    double polarLength = 0.0; //initializes the length of the polar segment, or the distance between the robot's prev and current position

    double xCoordinateDelta = 0.0; //initializes global x change
    double yCoordinateDelta = 0.0; //initializes global y change

    if(globalOrientationDeltaRadians == 0.0) {
        localXOffset = horizontalTrackerDelta; //if robot maintains heading, the x offfset is just the change in the horizontal encoder's position
        localYOffset = verticalTrackerDelta; //ir robot maintains heading, they y offset is just the change in the vertical encoder's position
    }

    else {
        localXOffset = 2 * sin(globalOrientationDeltaRadians/2.0) * ((horizontalTrackerDelta / globalOrientationDeltaRadians) + horizontalToCenterDistance); //if robot doesn't maintain heading, calculate using Step 8/eq. 6 in the Purdue Sigbots Wiki
        localYOffset = 2 * sin(globalOrientationDeltaRadians/2.0) * ((verticalTrackerDelta / globalOrientationDeltaRadians) + verticalToCenterDistance); //if robot doesn't maintain heading, caluclate using Step 8/eq. 6 in the Purdue Sigbots Wiki
    }

    localPolarAngle = atan2(localYOffset, localXOffset); //the angle of movement relative to the robot's frame, where forward, right, backward, and left are 90, +-0, -90, and +-180 degrees respectively
    globalPolarAngle = localPolarAngle - (this->prevGlobalOrientationRadians + globalOrientationDeltaRadians / 2); //The "(this->prevGlobalOrientationRadians + globalOrientationDeltaRadians / 2)" is Step 9 in the Purdue Sigbots Wiki
                                                                                                                       //The "localPolarAngle - ..." is Purdue Sigbots Wiki Step 10
    polarLength = hypot(localXOffset, localYOffset); //calculates the length of the polar segment, or the distance between the robot's prev and current position

    xCoordinateDelta = cos(globalPolarAngle) * polarLength; //calculates global change in the robot's x-coordinate
    yCoordinateDelta = sin(globalPolarAngle) * polarLength; //caluclates global change in the robot's y-coordinate

    this->xCoordinate += xCoordinateDelta; //updates robot's x-coordinate
    this->yCoordinate += yCoordinateDelta; //updates robot's y-coordinate

    this->prevHorizontalTrackerPosition = horizontalTrackerPosition; //updates values for next cycle
    this->prevVerticalTrackerPosition = verticalTrackerPosition; //updates values for next cycle
    this->prevGlobalOrientationRadians = globalOrientationRadians; //updates values for next cycle
}