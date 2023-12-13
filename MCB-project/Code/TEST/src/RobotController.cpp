#include "RobotController.h"
#include <cmath>

namespace ThornBots {
    /*
    * Constructor for RobotController
    */
    RobotController::RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController) {
        this->drivers = driver;
        this->driveTrainController = driveTrainController;
        this->turretController = turretController;
    }

    void RobotController::inialize() {
        Board::initialize();
        drivers->can.initialize();
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->bmi088.requestRecalibration();
        drivers->remote.initialize();
        this->driveTrainController->initialize();
        this->turretController->initialize();
        modm::delay_ms(2500); //Delay 2.5s to allow the IMU to turn on and get working before we move it around
        //TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's reading correctly, ect)
    }

    void RobotController::update() {
        drivers->canRxHandler.pollCanData();

        drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes into can signal

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
        rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        //TODO
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

    bool RobotController::toggleKeyboardAndMouse() {
        //TODO
        return false;
    }
}