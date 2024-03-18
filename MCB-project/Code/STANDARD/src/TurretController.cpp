#include "TurretController.h"

namespace ThornBots {
    TurretController::TurretController(tap::Drivers* driver) {
        this->drivers = driver;
        //TODO: Complete this
    }
    void TurretController::initialize() {
        motor_Pitch.initialize();
        motor_Yaw.initialize();
        motor_Indexer.initialize();
        motor_Flywheel1.initialize();
        motor_Flywheel2.initialize();
        drivers->pwm.init(); //For the servo we will be using

        //Nothing needs to be done to drivers
        //Nothing needs to be done to the controllers
    }

    void TurretController::turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double dt) {
        if(turretControllerTimer.execute()) {
            pitchMotorVoltage = getPitchVoltage(desiredPitchAngle, dt);
            yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, dt);
            
            flyWheelVoltage = getFlywheelVoltage();
            indexerVoltage = getIndexerVoltage();
        }
        //TODO: Add flywheels, indexer, and servo
    }

    void TurretController::setMotorSpeeds() {
        motor_Pitch.setDesiredOutput(pitchMotorVoltage);
        motor_Yaw.setDesiredOutput(yawMotorVoltage);

        // uint32_t voltageToSet = 0;
        

        motor_Indexer.setDesiredOutput(indexerVoltage);
        motor_Flywheel1.setDesiredOutput(flyWheelVoltage);
        motor_Flywheel2.setDesiredOutput(flyWheelVoltage);
    }

    void TurretController::stopMotors() {
        motor_Pitch.setDesiredOutput(0);
        motor_Yaw.setDesiredOutput(0);
        motor_Indexer.setDesiredOutput(0);
        motor_Flywheel1.setDesiredOutput(0);
        motor_Flywheel2.setDesiredOutput(0);
        drivers->djiMotorTxHandler.encodeAndSendCanData();
        //TODO: Add the other motors
    }

    void TurretController::enableShooting() {
        this->shootingSafety = true;
    }

    void TurretController::disableShooting() {
        this->shootingSafety = false;
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

    int TurretController::getFlywheelVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return FLYWHEEL_MOTOR_MAX_SPEED;
        }else{
            return 0;
        }
    }

    int TurretController::getIndexerVoltage() {
        if (robotDisabled) return 0;
        if(shootingSafety){
            return indexerVoltage;
        }else{
            return 0;
        }
    }

    void TurretController::enableIndexer() {
        indexerVoltage = INDEXER_MOTOR_MAX_SPEED;
        
    }

    void TurretController::disableIndexer() {
        indexerVoltage = 0;
    }

    void TurretController::disable(){
        robotDisabled = true;
    }
    void TurretController::enable(){
        robotDisabled = false;
    }

}
