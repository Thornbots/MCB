#include "TurretController.h"

namespace ThornBots {
    TurretController::TurretController(tap::Drivers* driver) {
        this->drivers = driver;
        //TODO: Complete this
    }
    void TurretController::initialize() {
        motor_Pitch.initialize();
        motor_Yaw.initialize();
        //Nothing needs to be done to drivers
        //Nothing needs to be done to the controllers
    }

    void TurretController::turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double dt) {
        if(turretControllerTimer.execute()) {
            pitchMotorVoltage = getPitchVoltage(desiredPitchAngle, dt);
            yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, dt);
            
        }
        //TODO: Add flywheels, indexer, and servo
    }

    void TurretController::setMotorSpeeds() {
        motor_Pitch.setDesiredOutput(pitchMotorVoltage);
        motor_Yaw.setDesiredOutput(yawMotorVoltage);
    }

    void TurretController::stopMotors() {
        motor_Pitch.setDesiredOutput(0);
        motor_Yaw.setDesiredOutput(0);

        drivers->djiMotorTxHandler.encodeAndSendCanData();
        //TODO: Add the other motors
    }


    void TurretController::reZeroYaw() {
        //TODO
    }

    int TurretController::getYawVoltage(double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double desiredAngleWorld, double dt) {
        if (robotDisabled) return 0;
        return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, dt); //1000 to convert to mV which taproot wants. DTrpm is 0, can calculate and pass in the future
    }

    int TurretController::getPitchVoltage(double targetAngle, double dt) {
        if (robotDisabled) return 0;
        return 1000*pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle, dt);
    }


    void TurretController::disable(){
        robotDisabled = true;
    }
    void TurretController::enable(){
        robotDisabled = false;
    }

}
