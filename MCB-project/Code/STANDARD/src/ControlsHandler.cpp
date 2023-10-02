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

    void ControlsHandler::update() {

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

            temp_yaw_angle = turretController->getYawEncoderAngle();

        }

        switch(rightSwitchValue) {
            case 0: //Turret is locked to the drivebase (Turret moves drivetrain follows)
                //TODO
                break;
            case 1: //Turret is independent of the drivebase
                //left stick translates the robot
                //right stick handle pitch and yaw of the turret

                //step 1 convert to pitch and yaw of the right stick
                double distance = right_stick_vert;
                double turnSpeed = MAX_SPEED * distance;

                //step2 find angle and speed of left stick
                double translationAngle = getAngle(left_stick_horz, left_stick_vert);
                double magnitude = hypot(left_stick_horz, left_stick_vert);
                double translationSpeed = MAX_SPEED * magnitude;

                //Drive train needs translation speed, and translation angle to know how to move
                //TODO implement tempName
                driveTrainController->tempName( translationSpeed, translationAngle, temp_yaw_angle);

                break;
            case 2: //Turret is aligned with the drivebase (Drivetrain moves turret follows)
                //left stick translates the robot
                //right stick only rotates the robot horz values don't matter

                //step 1 find the turnspeed of the right stick
                double distance = right_stick_vert;
                double turnSpeed = MAX_SPEED * distance;

                //step2 find angle and speed of left stick
                double translationAngle = getAngle(left_stick_horz, left_stick_vert);
                double magnitude = hypot(left_stick_horz, left_stick_vert);
                double translationSpeed = MAX_SPEED * magnitude;

                //Drive train needs turn speed, translation speed, and translation angle to know how to move
                //TODO implement tempName
                driveTrainController->tempName( turnSpeed, translationSpeed, translationAngle);
                driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());

                //TODO understand turret controller
                turretController->tempName();
                turretController->setMotorSpeeds(sendTurretTimeout.execute());

                break;
        }

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
                beybladeFactor = BEYBLADE_FACTOR;
                break;

            case tap::communication::serial::Remote::SwitchState::MID:
                // isLeftStickUp = false;
                // isLeftStickDown = false;
                // doBeyblading = false;
                // turretIndependent = true;
                beybladeFactor = BEYBLADE_FACTOR / 2;
                break;

            case tap::communication::serial::Remote::SwitchState::DOWN:
                // isLeftStickDown = true;
                // isLeftStickUp = false;
                // doBeyblading = false;
                // turretIndependent = false;
                beybladeFactor = 0;
                break;
        }
        
    }

    void ControlsHandler::findRightSwitchState() {
        auto rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        switch (rightSwitchState) {
            case tap::communication::serial::Remote::SwitchState::UP:
                // TODO: Make the drivebase align with the turret.
                // i.e., if the angle offset is negative, make it spin CW or vice versa
                rightSwitchValue = 2;
                break;

            case tap::communication::serial::Remote::SwitchState::MID:
                rightSwitchValue = 1;
                break;

            case tap::communication::serial::Remote::SwitchState::DOWN:
                //Lock the turret to the drivebase
                turretController->reZero();
                rightSwitchValue = 0;
                break;
        }
    }
}