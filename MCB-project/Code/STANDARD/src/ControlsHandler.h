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
        bool keyboardAndMouseEnabled;

        tap::Drivers* drivers;

        //Functions
        
        /**
        * Reads inputs from the keyboard and mouse and checks to see if KBM(keyboard and Mouse) mode should
        * be enabled or not. It requires the pressing of CTRL + SHIFT + R to enable KBM mode.
        */
        bool toggleKeyboardAndMouse();
    };
}