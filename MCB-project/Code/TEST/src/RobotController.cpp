#include "RobotController.h"

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
        this->driveTrainController->initialize();
        this->turretController->initialize();
        //TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's reading correctly, ect)
    }

    void RobotController::update() {
        //TODO
    }

    void RobotController::stopRobot() {
        //TODO
    }

    void RobotController::updateAllInputVariables() {
        //TODO
    }

    double RobotController::getAngle(double x, double y) {
        //TODO
        return ((double)0.0);
    }

    bool RobotController::toggleKeyboardAndMouse() {
        //TODO
        return false;
    }

    int RobotController::findLeftSwitchState() {
        //TODO
        return 0;
    }

    int RobotController::findRighTSwitchState() {
        //TODO
        return 0;
    }
}