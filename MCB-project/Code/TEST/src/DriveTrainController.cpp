#include "DriveTrainController.h"

namespace ThornBots{
    DriveTrainController::DriveTrainController(tap::Drivers* driver) {
        this->drivers = driver;
    }

    void DriveTrainController::initialize() {
        //TODO
    }

    void DriveTrainController::DriveTrainMovesTurretFollow() {
        //TODO
    }

    void DriveTrainController::TurretMoveDriveTrainFollow() {
        //TODO
    }

    void DriveTrainController::TurretMoveDriveTrainIndependent() {
        //TODO
    }

    void DriveTrainController::setMotorSpeeds() {
        //TODO
    }

    void DriveTrainController::stopMotors() {
        //TODO
    }

    void DriveTrainController::convertTranslationSpeedToMotorSpeeds(double magnitude, double angle) {
        //TODO
    }

    void DriveTrainController::adjustMotorSpeedWithTurnSpeed(double turnAngle) {
        //TODO
    }
}