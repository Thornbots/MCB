#include "DriveTrainController.h"

namespace ThornBots{
    static double motorOneSpeed, motorTwoSpeed, motorThreeSpeed, motorFourSpeed = 0;
    double motorOneRPM, motorTwoRPM, motorThreeRPM, motorFourRPM = 0.0;
    DriveTrainController::DriveTrainController(tap::Drivers* driver) {
        this->drivers = driver;
    }

    void DriveTrainController::initialize() {
        motor_one.initialize();
        motor_two.initialize();
        motor_three.initialize();
        motor_four.initialize();
        //Nothing needs to be done to drivers
        //Nothing needs to be done to PID controllers
    }

    void DriveTrainController::moveDriveTrain(double turnSpeed, double translationSpeed, double translationAngle) {
        convertTranslationSpeedToMotorSpeeds(translationSpeed, translationAngle);
        adjustMotorSpeedWithTurnSpeed(turnSpeed);
    }

    void DriveTrainController::followTurret(double translationSpeed, double translationAngle, double driveTrainAngleFromTurret) {
        //TODO: Check that this works
        convertTranslationSpeedToMotorSpeeds(translationSpeed, translationAngle);

        pidControllerDTFollowsT.runControllerDerivateError(driveTrainAngleFromTurret, 1); //TODO: TUNE THE PID CONSTANTS!!!
        double turnSpeed = ((double)pidController.getOutput());
        
        adjustMotorSpeedWithTurnSpeed(turnSpeed);
    }

    void DriveTrainController::setMotorSpeeds() {
        drivers->canRxHandler.pollCanData();
        motorOneRPM = motor_one.getShaftRPM();
        motorTwoRPM = motor_two.getShaftRPM();
        motorThreeRPM = motor_three.getShaftRPM();
        motorFourRPM = motor_four.getShaftRPM();

        // Motor1 (The driver's front wheel)
        // pidController.runControllerDerivateError(motorOneSpeed - motor_one.getShaftRPM(), 1);
        pidController.runControllerDerivateError(motorOneSpeed - motor_one.getShaftRPM(), 1);
        motor_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor2 (The passenger's front wheel)
        pidController.runControllerDerivateError(motorTwoSpeed - motor_two.getShaftRPM(), 1);
        motor_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor3 (The driver's back wheel)
        pidController.runControllerDerivateError(motorThreeSpeed - motor_three.getShaftRPM(), 1);
        motor_three.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor4 (The passenger's back wheel)
        pidController.runControllerDerivateError(motorFourSpeed - motor_four.getShaftRPM(), 1);
        motor_four.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        
        drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
    }

    void DriveTrainController::stopMotors() {
        motorOneSpeed = 0;
        motorTwoSpeed = 0;
        motorThreeSpeed = 0;
        motorFourSpeed = 0;
    }

    void DriveTrainController::convertTranslationSpeedToMotorSpeeds(double translationSpeed, double translationAngle) {
        motorOneSpeed = translationSpeed * sin(translationAngle + (PI / 4));
        motorTwoSpeed = translationSpeed * sin(translationAngle - (PI / 4));
        motorThreeSpeed = translationSpeed * sin(translationAngle - (PI / 4));
        motorFourSpeed = translationSpeed * sin(translationAngle + (PI / 4));
    }

    void DriveTrainController::adjustMotorSpeedWithTurnSpeed(double turnSpeed) {
        motorOneSpeed += turnSpeed;
        motorTwoSpeed -= turnSpeed;
        motorThreeSpeed += turnSpeed;
        motorFourSpeed -= turnSpeed;
    }
}