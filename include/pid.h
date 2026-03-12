#pragma once

#include "type_defines.h"
#include "util.h"

#include <cmath>
#include <iostream>

class PID {
    public:
        double startError; //error at the start of the PID
        
        //===== OUTPUT_CONSTANTS =====//
        double kp; //the constant we multiply our error by: we tune this
        double ki; //the constant we multiply our integral by: we tune this
        double kd; //the constant we multiply our derivative by: we tune this
        
        //===== LOGIC_CONSTANTS =====//
        double error; //current error
        double startIntegral; //when we start adding on to our integral term
        double integral; //our integral term
        double prevError; //our error in the prev loop
        double derivative; //our derivative term

        //===== SETTLE_CONSTANTS =====//
        double deadband; //how far off we can be from our target: we will never perfectly achieve our target, so we need this
        double motorOutput; //how fast our motors will spin this loop
        double stallCounter; //difference between previous error and current error which tells you if you should add onto counter
        double counterLimit; //checks if the robot has stalled for too long
        double counter; //a stall constant, ++ every time diff in position is less than stallCounter
                
        //===== CONTROL_OUTPUT_CONSTANTS =====//
        double prevMotorOutput; //how fast our motors spun the prev loop
        double minMotorOutput; //minimum speed our motors need to spin at
        double maxMotorOutput; //maximum speed our motors can spin at
        double accelerationSlew; //difference allowed between our previous and current motor output for acceleration
        double minDecelerationSlew; //difference allowed between our previous and current motor output for deceleration
        double maxDecelerationSlew; //difference allowed between our previous and current motor output for deceleration if motion chaining

        IntegralMethod integralMethod;
        
        PID(double startError, double kp, double ki, double kd, double startIntegral, double deadband, double stallCounter, double counterLimit, double minMotorOutput, double maxMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDecelerationSlew, IntegralMethod integralMethod); //our constructor, what we use to create our object

        void setError(double startError);
        void setOutputConstants(double kp, double ki, double kd);
        void setLogicConstants(double startIntegral);
        void setSettleConstants(double deadband, double stallCounter, double counterLimit);
        void setControlOutputConstants(double minMotorOutput, double maxMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDeclerationSlew);
        void setIntegralMethod(IntegralMethod integralMethod);

        void setIntegral(double integral);
        void setPrevError(double prevError);
        void setDerivative(double derivative);
        void setMotorOutput(double motorOutput);
        void setCounter(double counter);
        void setPrevMotorOutput(double prevMotorOutput);
        
        bool settled(); //a function which returns true or false: if true, our PID is finished, otherwise our PID needs to keep on running
        double output(); //how fast our motors will spin this loop, function which returns our motor output
};
