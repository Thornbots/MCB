#include "ControlsHandler.h"



namespace ThornBots {
    ControlsHandler::ControlsHandler(tap::Drivers* m_driver) {
        this->drivers = m_driver;
        keyboardAndMouseEnabled = false;
    }

    ControlsHandler::~ControlsHandler() {
    }

    ControlsHandler::main() {
        if (KeyboardAndMouseEnabled) {
            // We are using Keyboard and Mouse controls
            // TODO : Make this work

            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W)) {
                //TODO: Make the robot move forward
            }
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A)) {
                //TODO: Make the robot move left
            }
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S)) {
                //TODO: Make the robot move backwards
            }
            if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D)) {
                //TODO: Make the robot move right
            }
        } else {
            
        }

    }

    bool ControlsHandler::toggleKeyboardAndMouse() {
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL) &&
            drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT) &&
            drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) {

            return true;
        }
        return false;
    }
}