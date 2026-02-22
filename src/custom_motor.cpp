#include "custom_motor.h"

//Constructor
Custom_Motor::Custom_Motor(const int32_t port, const gearSetting cartridge, bool reversed):
    motor(port, cartridge, reversed),
    port(port),
    cartridge(cartridge),
    reversed(reversed)
{};

Custom_Motor_Group::Custom_Motor_Group(Custom_Motor& motor1, Custom_Motor& motor2, Custom_Motor& motor3):
    motor_group(motor1, motor2, motor3),
    motor1(motor1),
    motor2(motor2),
    motor3(motor3),
    positions(3)
{};

double Custom_Motor_Group::getPosition() {
    positions[0] = motor1.position(degrees);
    positions[1] = motor2.position(degrees);
    positions[2] = motor3.position(degrees);

    std::sort(positions.begin(), positions.end());
    
    return positions[1];
};