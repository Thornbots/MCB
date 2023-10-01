#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include <cmath>
#include "drivers_singleton.hpp"

namespace ThornBots {
    class ControlsHandler {
    public:
        //Constructor
        ControlsHandler(tap::Drivers* m_driver);
        //Destructor
        ~ControlsHandler();
        /*
        * Main function for the ControlsHandler class. This function will be called in the main.cpp file.
        */
        void main();


    private:
        bool keyboardAndMouseEnabled = false;
        int leftSwitchValue;
        int rightSwitchValue;
        double right_stick_vert = 0.0;
        double right_stick_horz = 0.0;
        double left_stick_vert = 0.0;
        double left_stick_horz = 0.0;
        int16_t wheel_value = 0;

        tap::Drivers* drivers;

        //Functions
        
        /**
        * Reads inputs from the keyboard and mouse and checks to see if KBM(keyboard and Mouse) mode should
        * be enabled or not. It requires the pressing of CTRL + SHIFT + R to enable KBM mode.
        */
        bool toggleKeyboardAndMouse();

        /*
        * Reads the state of the left switch on the remote and sets leftSwitchValue to 2 if the switch is up,
        * 1 if the switch is in the middle, and 0 if the switch is down.
        */
        void findLeftSwitchState();

        /*
        * Reads the state of the right switch on the remote and sets rightSwitchValue to 2 if the switch is up,
        * 1 if the switch is in the middle, and 0 if the switch is down.
        */
        void findRightSwitchState();
    };
}