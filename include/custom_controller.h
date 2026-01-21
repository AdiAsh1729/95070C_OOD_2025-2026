#pragma once

#include "vex.h"
#include "type_defines.h"
#include <iostream>

using namespace vex;

class Custom_Controller : public controller {
    private:
        bool buttonStatePrevious[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
        bool buttonStateCurrent[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
        
    public:
        Custom_Controller();

        const controller::button& getButton(Button checkButtonID);
        bool pressed(Button checkButtonID);
        bool released(Button checkButtonID);
        bool allReleased();
        void resetPressed();
};