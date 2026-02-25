#include "pid.h"

<<<<<<< Updated upstream
PID::PID(double startError, double kp, double ki, double kd, double startIntegral, double deadband, double stallCounter, double counterLimit, double minMotorOutput, double maxMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDecelerationSlew, IntegralMethod integralMethod):
=======
PID::PID(double startError, double kp, double ki, double kd, double startIntegral, double deadband, double stallCounter, double counterLimit, bool exit, double minimumMotorOutput, double maximumMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDecelerationSlew, IntegralMethod integralMethod):
>>>>>>> Stashed changes
    startError(startError),

    kp(kp),
    ki(ki),
    kd(kd),

    error(startError),
    startIntegral(startIntegral),
    integral(0),
    prevError(startError),
    derivative(0),

    deadband(deadband),
    motorOutput(0),
    stallCounter(stallCounter),
    counterLimit(counterLimit),
    counter(0),
    exit(exit),
    
<<<<<<< Updated upstream
    prevMotorOutput(0),
    minMotorOutput(minMotorOutput),
    maxMotorOutput(maxMotorOutput),
=======
    previousMotorOutput(0),
    minimumMotorOutput(minimumMotorOutput),
    maximumMotorOutput(maximumMotorOutput),
>>>>>>> Stashed changes
    accelerationSlew(accelerationSlew),
    minDecelerationSlew(minDecelerationSlew),
    maxDecelerationSlew(maxDecelerationSlew),

    integralMethod(integralMethod)
{};

void PID::setError(double startError) {
    this->startError = startError;
    this->error = startError;
};

void PID::setOutputConstants(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
};

void PID::setLogicConstants(double startIntegral) {
    this->startIntegral = startIntegral;
};

void PID::setSettleConstants(double deadband, double stallCounter, double counterLimit) {
    this->deadband = deadband;
    this->stallCounter = stallCounter;
    this->counterLimit = counterLimit;
};

void PID::setControlOutputConstants(double minMotorOutput, double maxMotorOutput, double accelerationSlew, double minDecelerationSlew, double maxDecelerationSlew) {
    this->minMotorOutput = minMotorOutput;
    this->maxMotorOutput = maxMotorOutput;
    this->accelerationSlew = accelerationSlew;
    this->minDecelerationSlew = minDecelerationSlew;
    this->maxDecelerationSlew = maxDecelerationSlew;
};

void PID::setIntegralMethod(IntegralMethod integralMethod) {
    this->integralMethod = integralMethod;
};

void PID::setIntegral(double integral) {
    this->integral = integral;
};

void PID::setPrevError(double prevError) {
    this->prevError = prevError;
};

void PID::setDerivative(double derivative) {
    this->derivative = derivative;
};

void PID::setMotorOutput(double motorOutput) {
    this->motorOutput = motorOutput;
};

void PID::setCounter(double counter) {
    this->counter = counter;
};

void PID::setPrevMotorOutput(double prevMotorOutput) {
    this->prevMotorOutput = prevMotorOutput;
};

bool PID::settled() {
    return (std::abs(this->error) < deadband) || ((counter > counterLimit) && (counterLimit != 0)); //checks if the robot is within the deadband or is stalling(only checked if stalling matters -> e.g. in bearing PID, counterLimit is 0)
};

double PID::output() {
    //==== Derivative Calculation ====//
    this->derivative = (this->error - this->prevError) * 50;
    
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
<<<<<<< Updated upstream
    if(std::abs(this->motorOutput) > std::abs(prevMotorOutput) && getSign(this->motorOutput) == getSign(this->prevMotorOutput)) { //slew controller for acceleration if not crossed target
        if(std::abs(this->motorOutput - this->prevMotorOutput) > accelerationSlew) {
            this->motorOutput = this->prevMotorOutput + getSign(this->motorOutput) * accelerationSlew;
        }
    }

    if(std::abs(this->motorOutput) < std::abs(this->prevMotorOutput) && getSign(this->motorOutput) == getSign(this->prevMotorOutput)) { //slew controller for deceleration if not crossed target
        if(std::abs(this->prevMotorOutput - this->motorOutput) < this->minDecelerationSlew) {
            this->motorOutput = this->prevMotorOutput - minDecelerationSlew * getSign(this->motorOutput);
        }

        if(getSign(this->motorOutput) * (this->prevMotorOutput - this->motorOutput) > this->maxDecelerationSlew) {
            this->motorOutput = this->prevMotorOutput - maxDecelerationSlew * getSign(this->motorOutput);
=======
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
>>>>>>> Stashed changes
        }
    }
    
    //==== Clamper ====//
    if(this->motorOutput != 0) { //clamps if not first move and if not applying deceleration slew after crossing target
<<<<<<< Updated upstream
        this->motorOutput = clamp(this->motorOutput, this->minMotorOutput, this->maxMotorOutput); //clamps the motor output between the minimum and maximum values
=======
        this->motorOutput = clamp(this->motorOutput, this->minimumMotorOutput, this->maximumMotorOutput); //clamps the motor output between the minimum and maximum values
>>>>>>> Stashed changes
    }

    //==== Stall Check ====//
    if(this->motorOutput != 0) {
<<<<<<< Updated upstream
        if(std::abs(this->error - this->prevError) < this->stallCounter) {
=======
        if(std::abs(this->error - this->previousError) < this->stallCounter) {
>>>>>>> Stashed changes
            this->counter++;
        }
    }

    else {
        this->counter = 0;
    }

<<<<<<< Updated upstream
    //==== Updates prev Values ====//
    this->prevMotorOutput = this->motorOutput;
    this->prevError = this->error;
=======
    //==== Updates Previous Values ====//
    this->previousMotorOutput = this->motorOutput;
    this->previousError = this->error;
>>>>>>> Stashed changes

    return this->motorOutput;
};