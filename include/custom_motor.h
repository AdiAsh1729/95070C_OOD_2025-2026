#pragma once

#include "vex.h"
#include <algorithm>
#include <vector>

using namespace vex;

class Custom_Motor : public motor {
    private:
        const int32_t port;
        const gearSetting cartridge;
        bool reversed;

    public:
        Custom_Motor(const int32_t port, const gearSetting cartridge, bool reversed);
};

class Custom_Motor_Group : public motor_group {
    private:
        Custom_Motor& motor1;
        Custom_Motor& motor2;
        Custom_Motor& motor3;
        std::vector<int> positions; //creates a vector storing the 3 motor's positions, is resized in intitalizer list

    public:
        Custom_Motor_Group(Custom_Motor& motor1, Custom_Motor& motor2, Custom_Motor& motor3);

        double getPosition();
};