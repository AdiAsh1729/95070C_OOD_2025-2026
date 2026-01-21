#pragma once

#include "vex.h"

using namespace vex;

class Custom_Motor : public motor {
    private:

    public:
        Custom_Motor(const int32_t port, const gearSetting cartridge, bool reversed);

        void halt();
};