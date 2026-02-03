#include "pid.h"

PID::PID(double startError, double kp, double ki, double kd, double startIntegral, double deadband, double stallCounter, double counterLimit, double minimumMotorOutput, double maximumMotorOutput, double slewRate, IntegralMethod integralMethod):
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
    slewRate(slewRate),

    integralMethod(integralMethod)
{};

bool PID::settled() {
    return (std::abs(this->error) < deadband) || ((counter > counterLimit) && (counterLimit != 0));
};

double PID::output() {
    this->derivative = (this->error - this->previousError) * 50;
    
    if(std::abs(this->error) < this->startIntegral) {
        this->integral += this->error / 50;
    }

    if(integralMethod == Reset) {
        if(getSign(this->error) != getSign(this->startError)) {
            this->integral = 0;
        }
    }

    this->motorOutput = this->error * this->kp + this->integral * this->ki + this->derivative * this->kd; //main output
    clamp(this->motorOutput, this->minimumMotorOutput, this->maximumMotorOutput);

    if(getSign(this->motorOutput) * (this->motorOutput - this->previousMotorOutput) > this->slewRate) {
        this->motorOutput = this->previousMotorOutput + slewRate * getSign(this->motorOutput - this->previousMotorOutput);
    }

    if(this->motorOutput != 0) {
        if(std::abs(this->error - this->previousError) < this->stallCounter) {
            counter++;
        }
    }

    else {
        counter = 0;
    }

    this->previousMotorOutput = this->motorOutput;
    this->previousError = this->error;

    return this->motorOutput;
};