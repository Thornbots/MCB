#include <cmath>

#include "RobotController.h"
#include "DriveTrainController.h"
#include "TurretController.h"

tap::arch::PeriodicMilliTimer sendDrivetrainTimeout(2);
tap::arch::PeriodicMilliTimer sendTurretTimeout(2);

namespace ThornBots {
    RobotController::RobotController(tap::Drivers* m_driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController) {
        this->drivers = m_driver;
        this->driveTrainController = driveTrainController;
        this->turretController = turretController;

        //temp to be deleted
        // bool KeyboardAndMouseEnabled = false;
        // bool doBeyblading = false;
        // float temp_yaw_angle = 0.0;
    }

    RobotController::~RobotController() {
    }

    void RobotController::update() {

        //keyboardAndMouseEnabled = toggleKeyboardAndMouse();
        keyboardAndMouseEnabled = false;

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

        } else { //We are using the remote controls

            //findLeftSwitchState();
            leftSwitchValue = 0;
            //findRightSwitchState();
            rightSwitchValue = 0;

            // Get Current state of the Right Stick on the remote and set the appropriate
            right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

            // Get Current state of the Left Stick on the remote and set the appropriate
            left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);

            // Get Current state of the wheel on the remote and set the appropriate
            projectileMotorSpeed = drivers->remote.getWheel();
            
            //temp_yaw_angle = turretController->getYawEncoderAngle(); //Figure out what this does
        }

        //main logic for the robot
        switch(rightSwitchValue) {
            case 2: //Turret is locked to the drivebase (Turret moves drivetrain follows)
                //left stick translates the robot with respect to the turret
                //right stick handles the pitch and yaw of the turret

                //step 1 right stick inputs into yaw and pitch motor speeds.
                yawMotorSpeed = right_stick_horz * MAX_SPEED;
                pitchMotorSpeed = right_stick_vert * MAX_SPEED;

                //step2 find angle and speed of left stick
                translationAngle = getAngle(left_stick_horz, left_stick_vert);
                magnitude = hypot(left_stick_horz, left_stick_vert);
                translationSpeed = MAX_SPEED * magnitude;

                //Turret moves first
                turretController->TurretMovesDriveTrainFollows(yawMotorSpeed, pitchMotorSpeed, projectileMotorSpeed);
                turretController->setMotorSpeeds(sendTurretTimeout.execute());

                //Drive train needs translation speed, and translation angle to know how to move
                driveTrainController->TurretMovesDriveTrainFollow( translationSpeed, translationAngle, temp_yaw_angle);
                driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());


                break;
            case 1: //Turret is independent of the drivebase
                //left stick translates the robot independently
                //right stick handle pitch and yaw of the turret

                //step 1 right stick inputs into yaw and pitch motor speeds.
                yawMotorSpeed = right_stick_horz * MAX_SPEED;
                pitchMotorSpeed = right_stick_vert * MAX_SPEED;

                //step2 find angle and speed of left stick
                translationAngle = getAngle(left_stick_horz, left_stick_vert);
                magnitude = hypot(left_stick_horz, left_stick_vert);
                translationSpeed = MAX_SPEED * magnitude;

                //Turret moves first
                turretController->TurretMovesDriveTrainFollows(yawMotorSpeed, pitchMotorSpeed, projectileMotorSpeed);
                turretController->setMotorSpeeds(sendTurretTimeout.execute());

                //Drive train needs translation speed, and translation angle to know how to move
                driveTrainController->TurretMovesDriveTrainIndependent( translationSpeed, translationAngle, temp_yaw_angle);
                driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());



                break;
            case 0: //Turret is aligned with the drivebase (Drivetrain moves turret follows)
                //left stick translates the robot
                //right stick only rotates the robot horz values don't matter

                //step 1 find the turnspeed of the right stick
                distance = right_stick_vert;
                turnSpeed = MAX_SPEED * distance;   

                //step2 find angle and speed of left stick
                translationAngle = getAngle(left_stick_horz, left_stick_vert);
                magnitude = hypot(left_stick_horz, left_stick_vert);               //may want to square this value, was done before to make values closer to 1 relatively unchanged, if needed we can add it back
                translationSpeed = MAX_SPEED * magnitude;

                //Drive train needs turn speed, translation speed, and translation angle to know how to move
                driveTrainController->DriveTrainMovesTurretFollow(turnSpeed, translationSpeed, translationAngle);
                driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());

                //Turrets then follows DriveTrain
                turretController->FollowDriveTrain();
                turretController->setMotorSpeeds(sendTurretTimeout.execute());

                break;
            default:
                break;
        }
    }

    void RobotController::stopRobot() {
        driveTrainController->stopMotors(sendDrivetrainTimeout.execute());
        turretController->stopMotors(sendTurretTimeout.execute());
    }

    double RobotController::getAngle(double xPosition, double yPosition) {
        //error handling to prevent runtime errors in atan2
        if(xPosition == 0) {
            if(yPosition == 0) {
                return 0;
            }
            if(yPosition > 0) {
                return 0;
            }
            return PI;
        }
        if(yPosition == 0) {
            if(xPosition > 0) {
                return -((double)PI/(double)2); //0 degrees in radians
            }
            return ((double)PI/(double)2); //180 degrees in radians
        }

        return -atan2(yPosition, xPosition);
    }

    bool RobotController::toggleKeyboardAndMouse() {
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL) &&
            drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT) &&
            drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) {

            return true;
        }
        return false;
    }

    void RobotController::findLeftSwitchState() {
        tap::communication::serial::Remote::SwitchState leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        switch (leftSwitchState) {
            case tap::communication::serial::Remote::SwitchState::UP:
                // isLeftStickDown = false;
                // isLeftStickUp = true;
                // doBeyblading = true;
                // turretIndependent = false;
                beybladeFactor = MAXIMUM_BEYBLADE_FACTOR;
                break;

            case tap::communication::serial::Remote::SwitchState::MID:
                // isLeftStickUp = false;
                // isLeftStickDown = false;
                // doBeyblading = false;
                // turretIndependent = true;
                beybladeFactor = MAXIMUM_BEYBLADE_FACTOR / 2;
                break;

            case tap::communication::serial::Remote::SwitchState::DOWN:
                // isLeftStickDown = true;
                // isLeftStickUp = false;
                // doBeyblading = false;
                // turretIndependent = false;
                beybladeFactor = 0;
                break;

            case tap::communication::serial::Remote::SwitchState::UNKNOWN:
                //Do as little as possible (Don't beyblade) (Get's in this state from broken hardware on the controller)
                beybladeFactor = 0;
                break;
        }
        
    }

    void RobotController::findRightSwitchState() {
        tap::communication::serial::Remote::SwitchState rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
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
                //turretController->reZero();
                rightSwitchValue = 0;
                break;

            case tap::communication::serial::Remote::SwitchState::UNKNOWN:
                //Do as little as possible. (Gets in this state from broken hardware on the controller)
                break;
        }
    }
} //Namespace ThornBots