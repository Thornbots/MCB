#include <cmath>

#include "ControlsHandler.h"
#include "DriveTrainController.h"
#include "TurretController.h"




namespace ThornBots {
    ControlsHandler::ControlsHandler(tap::Drivers* m_driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController) {
        this->drivers = m_driver;
        this->driveTrainController = driveTrainController;
        this->turretController = turretController;

        //temp to be deleted
        bool KeyboardAndMouseEnabled = false;
        bool doBeyblading = false;
        float temp_yaw_angle = 0.0;
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
            right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

            // Get Current state of the Left Stick on the remote and set the appropriate
            left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);

            // Get Current state of the wheel on the remote and set the appropriate
            wheel_value = drivers->remote.getWheel();

        }
        temp_yaw_angle = turretController->getYawEncoderAngle();

        // Call the setMotorValues and setMotorSpeeds function in the DriveTrainController class
        driveTrainController->setMotorValues(
            right_stick_vert,
            right_stick_horz,
            left_stick_vert,
            left_stick_horz,
            temp_yaw_angle,
            rightSwitchValue,
            leftSwitchValue);

        driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());

        //TODO Fix the following codes to get robot turret to work again
        // Call the setMotorValues and setMotor Speeds function in the TurretController class
        // turretController->setMotorValues(
        //     KeyboardAndMouseEnabled,
        //     doBeyblading,
        //     angleOffset,
        //     -right_stick_vert,
        //     right_stick_horz,
        //     driveTrainController->motor_one.getShaftRPM(),
        //     driveTrainController->motor_four.getShaftRPM(),
        //     wheel_value,
        //     rightSwitchValue,
        //     leftSwitchValue);
        turretController->setMotorSpeeds(sendTurretTimeout.execute());

    }

    double ControlsHandler::getAngle(double xPosition, double yPosition) {
        //error handling to prevent runtime errors in atan2
        if(xPosition == 0) {
            if(yPosition == 0) {
                return 0;
            }
            if(yPosition > 0) {
                return 0;
            }
            return (double)(PI);
        }
        if(yPosition == 0) {
            if(xPosition > 0) {
                return (double) (3 * PI) / 2;
            }
            return (double) PI / 2;
        }

        return atan2(yPosition, xPosition) + PI / 2.0;
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