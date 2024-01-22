#include "RobotController.h"
#include <cmath>

namespace ThornBots {
    double stickLeftHorz, stickLeftVert, stickRightHorz, stickRightVert, stickLeftAngle, stickLeftMagn, stickRightAngle, stickRightMagn = 0.0;
    /*
    * Constructor for RobotController
    */
    RobotController::RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController) {
        this->drivers = driver;
        this->driveTrainController = driveTrainController;
        this->turretController = turretController;
    }

    void RobotController::initialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        driveTrainController->initialize();
        turretController->initialize();
        modm::delay_ms(2500); //Delay 2.5s to allow the IMU to turn on and get working before we move it around
        //TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's reading correctly, ect)
    }

    void RobotController::update() {
        drivers->canRxHandler.pollCanData();
        updateAllInputVariables();

        //START: Handeling moving and aiming due to controller inputs. Once keyboard functionality is added, need to move this to a function.
        switch(leftSwitchState) {
            case(tap::communication::serial::Remote::SwitchState::UP): 
                //Left Switch is up. So need to beyblade at fast speed, and let right stick contorl turret yaw and pitch
                driveTrainController->driveTrainBeyBladeAndTranslate((leftStickMagnitude * MAX_SPEED), leftStickAngle, (FAST_BEYBLADE_FACTOR * MAX_SPEED));
                turretController->TurretMove();
                break;
            case(tap::communication::serial::Remote::SwitchState::MID):
                driveTrainController->driveTrainBeyBladeAndTranslate((leftStickMagnitude * MAX_SPEED), leftStickAngle, (SLOW_BEYBLADE_FACTOR * MAX_SPEED));
                turretController->TurretMove();
                //Left Switch is mid. So need to beyblade at slow speed, and let right stick control turret yaw and pitch
                break;
            case(tap::communication::serial::Remote::SwitchState::DOWN):
                //Left Switch is down. So need to not beyblade, and let right stick be decided on the right switch value
                switch(rightSwitchState) {
                    case(tap::communication::serial::Remote::SwitchState::UP):
                        //Left switch is down, and right is up. So driveTrainFollows Turret
                        driveTrainController->turretMoveDriveTrainFollow(leftStickMagnitude * MAX_SPEED, leftStickAngle, 0);
                        turretController->TurretMove();
                        break;
                    case(tap::communication::serial::Remote::SwitchState::MID):
                        //Left switch is down, and right is mid. So move turret independently of drivetrain
                        driveTrainController->turretMoveDriveTrainIndependent(leftStickMagnitude * MAX_SPEED, leftStickAngle);
                        turretController->TurretMove();
                        break;
                    case(tap::communication::serial::Remote::SwitchState::DOWN):
                        //Left switch is down, and right is mid. So move drivetrain and have turret follow.
                        driveTrainController->turretMoveDriveTrainIndependent(leftStickMagnitude * MAX_SPEED, leftStickAngle);
                        turretController->driveTrainMovesTurretFollows();
                        break;
                    default:
                        //Should not be in this state. So if we are, just tell robot to do nothing.
                        stopRobot();
                        break;
                }
                break;
            default:
                //Should not be in this state. So if we are, just tell robot to do nothing.
                stopRobot();
                break;
        }
        //STOP: Handeling moving and aiming due to controller inputs. Once keyboard functionality is added, need to move this to a function.

        if(drivers->remote.isConnected()) {
            if(driveTrainMotorsTimer.execute()) {
                driveTrainController->setMotorSpeeds();
            }
            if(turretMotorsTimer.execute()) {
                turretController->setMotorSpeeds();
            }
        } else {
            if(driveTrainMotorsTimer.execute()) {
                driveTrainController->stopMotors();
            }
            if(turretMotorsTimer.execute()) {
                turretController->stopMotors();
            }
        }

        // drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes into can signal
    }

    void RobotController::stopRobot() {
        driveTrainController->stopMotors();
        turretController->stopMotors();
    }

    void RobotController::updateAllInputVariables() {
        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
        if (IMUTimer.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        }

        //START Updating stick values
        //Actually Reading from remote
        rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
        left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
        right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
        right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
        //Turning the remote raw values into values we can use more easily (circular cordinates)
        leftStickAngle = getAngle(left_stick_horz, left_stick_vert);
        rightStickAngle = getAngle(right_stick_horz, right_stick_vert);
        leftStickMagnitude = getMagnitude(left_stick_horz, left_stick_vert);
        rightStickMagnitude = getMagnitude(right_stick_horz, right_stick_vert);
        //STOP Updating stick values
        
        wheelValue = drivers->remote.getWheel();

        stickLeftHorz = left_stick_horz;
        stickLeftVert = left_stick_vert;
        stickRightHorz = right_stick_horz;
        stickRightVert = right_stick_vert;
        stickLeftAngle = leftStickAngle;
        stickLeftMagn = leftStickMagnitude;
        stickRightAngle = rightStickAngle;
        stickRightMagn = rightStickMagnitude;
    }

    double RobotController::getAngle(double x, double y) {
        //error handling to prevent runtime errors in atan2
        if(x == 0) {
            if(y == 0) {
                return 0;
            }
            if(y > 0) {
                return 0;
            }
            return PI;
        }
        if(y == 0) {
            if(x > 0) {
                return -((double)PI/(double)2); //0 degrees in radians
            }
            return ((double)PI/(double)2); //180 degrees in radians
        }

        return -atan2(y, x);
    }

    double RobotController::getMagnitude(double x, double y) {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    bool RobotController::toggleKeyboardAndMouse() {
        //TODO
        return false;
    }
}