#include "ControlsHandler.h"



namespace ThornBots {
    ControlsHandler::ControlsHandler(tap::Drivers* m_driver) {
        this->drivers = m_driver;
    }

    ControlsHandler::~ControlsHandler() {
    }

    void ControlsHandler::main() {

        keyboardAndMouseEnabled = toggleKeyboardAndMouse();


        if (keyboardAndMouseEnabled) {
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
            //We are using the remote controls

            findLeftSwitchState();

            findRightSwitchState();

            // Get Current state of the Right Stick on the remote and set the appropriate
            right_stick_vert = drivers->remote.getChannel(
                tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            right_stick_horz = drivers->remote.getChannel(
                tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

            // Get Current state of the Left Stick on the remote and set the appropriate
            left_stick_vert = drivers->remote.getChannel(
                tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            left_stick_horz = drivers->remote.getChannel(
                tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);

            // Get Current state of the wheel on the remote and set the appropriate
            wheel_value = drivers->remote.getWheel();

            

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

    void ControlsHandler::findLeftSwitchState() {
        auto leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        switch (leftSwitchState) {
            case tap::communication::serial::Remote::SwitchState::UP:
                // isLeftStickDown = false;
                // isLeftStickUp = true;
                // doBeyblading = true;
                // turretIndependent = false;
                leftSwitchValue = 2;
                break;

            case tap::communication::serial::Remote::SwitchState::MID:
                // isLeftStickUp = false;
                // isLeftStickDown = false;
                // doBeyblading = false;
                // turretIndependent = true;
                leftSwitchValue = 1;
                break;

            case tap::communication::serial::Remote::SwitchState::DOWN:
                // isLeftStickDown = true;
                // isLeftStickUp = false;
                // doBeyblading = false;
                // turretIndependent = false;
                leftSwitchValue = 0;
                break;
        }
        
    }

    void ControlsHandler::findRightSwitchState() {
        auto rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        switch (rightSwitchState) {
            case tap::communication::serial::Remote::SwitchState::UP:
                // Nothing as of now
                //isRightStickMid = false;
                rightSwitchValue = 2;
                break;

            case tap::communication::serial::Remote::SwitchState::MID:
                // TODO: Make the drivebase align with the turret.
                // i.e., if the angle offset is negative, make it spin CW or vice versa
                //isRightStickMid = true;
                rightSwitchValue = 1;
                break;

            case tap::communication::serial::Remote::SwitchState::DOWN:
                //turretController->reZero();
                //isRightStickMid = false;
                rightSwitchValue = 0;
                // TODO: Lock the turret to the front of the drivetrain.
                // i.e., set the desired angle to be 0 (the center of the robot)
                break;
        }
    }
}