#include "custom_controller.h"

//Constructor
Custom_Controller::Custom_Controller() {

};

const controller::button& Custom_Controller::getButton(Button checkButtonID) {
    switch(checkButtonID) {
        case A:
            return this->ButtonA;

        case B:
            return this->ButtonB;

        case X:
            return this->ButtonX;

        case Y:
            return this->ButtonY;
            
        case Right:
            return this->ButtonRight;

        case Left:
            return this->ButtonLeft;

        case Up:
            return this->ButtonUp;

        case Down:
            return this->ButtonDown;

        case R1:
            return this->ButtonR1;

        case R2:
            return this->ButtonR2;

        case L1:
            return this->ButtonL1;

        case L2:
            return this->ButtonL2;
    }
};

bool Custom_Controller::pressed(Button checkButtonID) {
    const controller::button& CheckButton = getButton(checkButtonID);

    buttonStatePrevious[checkButtonID] = buttonStateCurrent[checkButtonID];

    if(CheckButton.pressing()) {
        buttonStateCurrent[checkButtonID] = true;
    }

    else {
        buttonStateCurrent[checkButtonID] = false;
    }

    if(!buttonStatePrevious[checkButtonID] && buttonStateCurrent[checkButtonID]) {
        return true;
    }

    else {
        return false;
    }
};

bool Custom_Controller::released(Button checkButtonID) {
    const controller::button& CheckButton = getButton(checkButtonID);

    buttonStatePrevious[checkButtonID] = buttonStateCurrent[checkButtonID];

    if(CheckButton.pressing()) {
        buttonStateCurrent[checkButtonID] = true;
    }

    else {
        buttonStateCurrent[checkButtonID] = false;
    }

    if(buttonStatePrevious[checkButtonID] && !buttonStateCurrent[checkButtonID]) {
        return true;
    }

    else {
        return false;
    }
};

bool Custom_Controller::allReleased() {
    int numberOfFalse = 0;

    for(int i = 0; i < 12; i++) {
        if(getButton((Button)i).pressing() == false) {
            numberOfFalse ++;
        }

        else {
            std::cout << Button(i) << std::endl;
        }
    }    

    if(numberOfFalse == 12) {
        return true;
    }

    else {
        return false;
    }
};

void Custom_Controller::resetPressed() { //a reset function
    for(int i = 0; i < 12; i++) {
        buttonStatePrevious[i] = false;
        buttonStateCurrent[i] = false;
    }
}