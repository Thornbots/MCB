#include "TurretController.h"

namespace ThornBots {
    double yawAngle, yawRPM = 0.0;

    TurretController::TurretController(tap::Drivers* driver) {
        this->drivers = driver;
        //TODO: Complete this
    }
    void TurretController::initialize() {
        motor_Pitch.initialize();
        motor_Yaw.initialize();
        //TODO: Finish the rest of these
    }

    void TurretController::followDriveTrain(double angleError) {
        if(angleError > 0) {} //bye bye warnings
        //TODO
    }

    void TurretController::turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double dt) {
        if(dt > 0) {} //Bye bye warning
        pitchMotorVoltage = getPitchVoltage(desiredPitchAngle);
        updateYawVariables();
            yawAngle = measuredYawMotorEncoderAngle;
            yawRPM = measuredYawMotorRPM;
        yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, dt);
        //TODO: Add flywheels, indexer, and servo
    }

    void TurretController::setMotorSpeeds() {
        motor_Pitch.setDesiredOutput(pitchMotorVoltage);
        motor_Yaw.setDesiredOutput(yawMotorVoltage);
    }

    void TurretController::stopMotors() {
        motor_Pitch.setDesiredOutput(0);
        motor_Yaw.setDesiredOutput(0);
        //TODO: Add the other motors
    }

    void TurretController::enableShooting() {
        //TODO
    }

    void TurretController::disableShooting() {
        //TODO
    }

    void TurretController::reZeroYaw() {
        //TODO
    }

    void TurretController::updateYawVariables() {
        measuredYawMotorRPM = motor_Yaw.getShaftRPM();
        measuredYawMotorEncoderAngle = tap::motor::DjiMotor::encoderToDegrees(motor_Yaw.getEncoderWrapped());
    }

    int TurretController::getYawVoltage(double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double desiredAngleWorld, double dt) {
        return yawController.getDesiredVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredAngleWorld, dt);
    }

    int TurretController::getPitchVoltage(double targetAngle) {
        double position = tap::motor::DjiMotor::encoderToDegrees(motor_Pitch.getEncoderWrapped());
        if(targetAngle < (double)-20) {targetAngle = (double)-20;}
        else if(targetAngle > (double)20) {targetAngle = (double)20;}

        targetAngle += (double)270;
        pidControllerPitch.runControllerDerivateError(targetAngle - position, 1);
        return pidControllerPitch.getOutput();
    }
}
