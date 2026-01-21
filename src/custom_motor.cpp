#include "custom_motor.h"

//Constructor
Custom_Motor::Custom_Motor(const int32_t port, const gearSetting cartridge, bool reversed):
    motor(port, cartridge, reversed)
{};

void Custom_Motor::halt() {
    this->spin(reverse, 10, pct);
    wait(10, msec);
    this->setVelocity(0, pct);
};