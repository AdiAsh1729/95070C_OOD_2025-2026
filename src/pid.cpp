#include "pid.h"

PID::PID(double startError, double kp, double ki, double kd, double startIntegral, double deadband, double stallCounter, double counterLimit, double minimumMotorOutput, double maximumMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDecelerationSlew, IntegralMethod integralMethod):
    startError(startError),

    kp(kp),
    ki(ki),
    kd(kd),

    error(startError),
    startIntegral(startIntegral),
    integral(0),
    previousError(startError),
    derivative(0),

    deadband(deadband),
    motorOutput(0),
    stallCounter(stallCounter),
    counterLimit(counterLimit),
    counter(0),
    
    previousMotorOutput(0),
    minimumMotorOutput(minimumMotorOutput),
    maximumMotorOutput(maximumMotorOutput),
    accelerationSlew(accelerationSlew),
    minDecelerationSlew(minDecelerationSlew),
    maxDecelerationSlew(maxDecelerationSlew),

    integralMethod(integralMethod)
{};

bool PID::settled() {
    return (std::abs(this->error) < deadband) || ((counter > counterLimit) && (counterLimit != 0)); //checks if the robot is within the deadband or is stalling(only checked if stalling matters -> e.g. in bearing PID, counterLimit is 0)
};

double PID::output() {
    //==== Derivative Calculation ====//
    this->derivative = (this->error - this->previousError) * 50;
    
    //=== Integral Calculation ====//
    if(std::abs(this->error) < this->startIntegral) {
        this->integral += this->error / 50;
    }

    if(integralMethod == Reset) {
        if(getSign(this->error) != getSign(this->startError)) {
            this->integral = 0;
        }
    }

    //==== Main Output ====//
    this->motorOutput = this->error * this->kp + this->integral * this->ki + this->derivative * this->kd;

    //==== Slew Rate Controllers ====//
    if(std::abs(this->motorOutput) > std::abs(previousMotorOutput) && getSign(this->motorOutput) == getSign(this->previousMotorOutput)) { //slew controller for acceleration if not crossed target
        if(std::abs(this->motorOutput - this->previousMotorOutput) > accelerationSlew) {
            this->motorOutput = this->previousMotorOutput + getSign(this->motorOutput) * accelerationSlew;
        }
    }

    if(std::abs(this->motorOutput) < std::abs(this->previousMotorOutput) && getSign(this->motorOutput) == getSign(this->previousMotorOutput)) { //slew controller for deceleration if not crossed target
        if(std::abs(this->previousMotorOutput - this->motorOutput) < this->minDecelerationSlew) {
            this->motorOutput = this->previousMotorOutput - minDecelerationSlew * getSign(this->motorOutput);
        }

        if(getSign(this->motorOutput) * (this->previousMotorOutput - this->motorOutput) > this->maxDecelerationSlew) {
            this->motorOutput = this->previousMotorOutput - maxDecelerationSlew * getSign(this->motorOutput);
        }
    }
    
    //==== Clamper ====//
    if(this->motorOutput != 0) { //clamps if not first move and if not applying deceleration slew after crossing target
        this->motorOutput = clamp(this->motorOutput, this->minimumMotorOutput, this->maximumMotorOutput); //clamps the motor output between the minimum and maximum values
    }

    //==== Stall Check ====//
    if(this->motorOutput != 0) {
        if(std::abs(this->error - this->previousError) < this->stallCounter) {
            this->counter++;
        }
    }

    else {
        this->counter = 0;
    }

    //==== Updates Previous Values ====//
    this->previousMotorOutput = this->motorOutput;
    this->previousError = this->error;

    return this->motorOutput;
};