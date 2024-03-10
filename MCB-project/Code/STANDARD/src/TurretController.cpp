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
        // motor_Yaw.updateEncoderValue(0);

        //Nothing needs to be done to drivers
        //Nothing needs to be done to the controllers
    }

    void TurretController::followDriveTrain(double angleError) {
        //TODO: Test that this works and tune the constants
        pidControllerTFollowsDT.runControllerDerivateError(angleError, 1);
        yawMotorVoltage = pidControllerTFollowsDT.getOutput();
    }

    void TurretController::turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double dt) {
        pitchMotorVoltage = getPitchVoltage(desiredPitchAngle);
        updateYawVariables();
        if(turretControllerTimer.execute()) {
            yawMotorVoltage = getYawVoltage(driveTrainRPM, yawAngleRelativeWorld, yawRPM, desiredYawAngle, dt);
        }
        //TODO: Add flywheels, indexer, and servo
        flyWheelVoltage = getFlywheelVoltage();
        indexerVoltage = getIndexerVoltage();
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
        return 1000 * yawController.calculate(yawAngleRelativeWorld, yawRPM, 0, desiredAngleWorld, dt); //1000 to convert to mV which taproot wants. DTrpm is 0, can calculate and pass in the future
    }

    int TurretController::getPitchVoltage(double targetAngle) {
        double position = tap::motor::DjiMotor::encoderToDegrees(motor_Pitch.getEncoderWrapped());
        if(targetAngle < (double)-20) {targetAngle = (double)-20;}
        else if(targetAngle > (double)20) {targetAngle = (double)20;}

        targetAngle += (double)270;
        pidControllerPitch.runControllerDerivateError(targetAngle - position, 1);
        return pidControllerPitch.getOutput();
    }

    int TurretController::getFlywheelVoltage() {
        return 0;
        //TODO
    }

    int TurretController::getIndexerVoltage() {
        return 0;
        //TODO
    }
}
