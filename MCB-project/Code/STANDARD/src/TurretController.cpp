#include "TurretController.h"
#include <cmath>
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "DriveTrainController.h"
#include <cmath>

namespace ThornBots {
    TurretController::TurretController(tap::Drivers* m_driver) {
        this->drivers = m_driver;
        motor_yaw.initialize();
        motor_indexer.initialize();
        motor_pitch.initialize();
        flywheel_one.initialize();
        flywheel_two.initialize();
    }

    float TurretController::getYawEncoderAngle() {
        return tap::motor::DjiMotor::encoderToDegrees(motor_yaw.getEncoderWrapped());
    }

    TurretController::~TurretController() {} //Watch this cute video of a cat instead: https://youtu.be/hg3e1KflmC8

    /**
     * Updates the values of the motors' speeds. i.e. if you're beyblading, it will tell the motor_yaw_speed to change depending on what needs to change
    */
    void TurretController::setMotorValues(bool useWASD, bool doBeyblading, double angleOffset, double right_stick_vert, double right_stick_horz, int motor_one_speed, int motor_four_speed, int16_t wheel_value, bool isRightStickUp, bool isLeftStickUp, int rightSwitchValue, int leftSwitchValue) {
        int psuedo_right_switch = rightSwitchValue;
        if (leftSwitchValue != 0) {
            psuedo_right_switch = 1;
        }
        current_yaw_angle -= .002*15*right_stick_horz;
        motor_yaw_speed = getYawMotorSpeed(angleOffset, motor_one_speed, motor_four_speed, isRightStickUp, isLeftStickUp, right_stick_horz, right_stick_vert, psuedo_right_switch);
        motor_pitch_speed = getPitchMotorSpeed(useWASD, right_stick_vert, (right_stick_vert * 20.0f));
        flywheel_speed = getFlywheelsSpeed(wheel_value);
        motor_indexer_speed = getIndexerMotorSpeed(wheel_value);
    }

    /**
     * Tells all the motors to go to their assined speeds
     * i.e. Tells motor_yaw to to go motor_yaw_speed and so on.
     * sendMotorTimeout should be the method call sendMotorTimeout.execute() when calling this method
    */
    void TurretController::setMotorSpeeds(bool sendMotorTimeout) {
        if(!sendMotorTimeout) { return; }
        drivers->canRxHandler.pollCanData();
        
        //Yaw Motor
        motor_yaw.setDesiredOutput(static_cast<int32_t>(motor_yaw_speed));

        //Pitch Motor
        motor_pitch.setDesiredOutput(static_cast<int32_t>(motor_pitch_speed - 6000.0f)); //The ~6000.0f is the feed forward (counters gravity for the pitch motor)

        //Indexer Motor
        pidController.runControllerDerivateError(motor_indexer_speed - motor_indexer.getShaftRPM(), 1);
        motor_indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        
        //Flywheel One
        pidController.runControllerDerivateError(flywheel_speed - flywheel_one.getShaftRPM(), 1);
        flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        
        //Flywheel Two
        pidController.runControllerDerivateError(flywheel_speed - flywheel_two.getShaftRPM(), 1);
        flywheel_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        
        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }

    /**
     * Tells all of the motors of the turret to go to 0 RPM.
     * We are hard coding this as well as updating the values to just ensure that the motors stop.
     * sendMotorTimeout should be the method call sendMotorTimeout.execute() when calling this method
    */
    void TurretController::stopMotors(bool sendMotorTimeout) {
        if(!sendMotorTimeout) { return; }
        motor_yaw_speed = 0;
        motor_pitch_speed = 0;
        motor_indexer_speed = 0;
        flywheel_speed = 0;

        //Yaw Motor
        pidController.runControllerDerivateError(0 - motor_yaw.getShaftRPM(), 1);
        motor_yaw.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Pitch Motor
        pidController.runControllerDerivateError(0 - motor_pitch.getShaftRPM(), 1);
        motor_pitch.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Indexer Motor
        pidController.runControllerDerivateError(0 - motor_indexer.getShaftRPM(), 1);
        motor_indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Flywheel One
        pidController.runControllerDerivateError(0 - flywheel_one.getShaftRPM(), 1);
        flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Flywheel Two
        pidController.runControllerDerivateError(0 - flywheel_two.getShaftRPM(), 1);
        flywheel_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
    }

    void TurretController::startShooting(){
        isShooting = true;
    }
    
    void TurretController::stopShooting(){
        isShooting = false;
    }

    /**
     * TODO: Make this functional!
     * Want an exponentional growth as the value is around 2-3, but also want a very weak response around 0-2. (ish. these numbers are *magical* numbers)
    */
    int TurretController::homemadePID(double value) {
        // if(abs(value) < 45) { return 0; }
        return (350 * log((pow(value, 2) / 30) + 1));
    }

    /**
     * Returns the speed that the yaw motor needs to go to counterbalance the effect of beyblading or turning.
     * Currently only works when in beyBlading mode.
     * TODO: Make this spin the yawMotor dependent on mouse(only when NOT using CV)
    */
    int TurretController::getYawMotorSpeed(double actualAngle, int motor_one_speed, int motor_four_speed, bool isRightStickUp, bool isLeftStickUp, double right_stick_horz, double right_stick_vert, int rightSwitchValue) {
        if(rightSwitchValue == 1 || rightSwitchValue == 2) {
            desiredAngle -= right_stick_horz * 0.03;
        } else if (rightSwitchValue == 0) {
            float position = getYawEncoderAngle();
            desiredAngle = actualAngle - position - 210.0;
        }
        double kF = 0.42;
        while(actualAngle - desiredAngle > 180){
            actualAngle -= 360;
        }
        while(desiredAngle - actualAngle > 180){
            actualAngle += 360;
        }
        yawPidController.runControllerDerivateError(desiredAngle-actualAngle, 1);
        tmp = actualAngle;
        return ((motor_one_speed - motor_four_speed) *kF) + yawPidController.getOutput();
    }

    int TurretController::getPitchMotorSpeed(bool useWASD, double right_stick_vert, double target_angle) {
        //desiredPitch += right_stick_vert * 0.01;
        //if (desiredPitch < -20.0f) desiredPitch = -20.0f;
        //if (desiredPitch > 20.0f) desiredPitch = 20.0f;
        float position = tap::motor::DjiMotor::encoderToDegrees(motor_pitch.getEncoderWrapped());
        float temp_target_angle = target_angle + 6.0f;
        if (temp_target_angle < -20.0f) temp_target_angle = -20.0f;
        if (temp_target_angle > 20.0f) temp_target_angle = 20.0f;
        float desired = 270.0f + temp_target_angle;
        pitchPidController.runControllerDerivateError(desired - position, 1);
        return pitchPidController.getOutput(); //TODO
    }

    int TurretController::getIndexerMotorSpeed(int16_t wheel_value) {
        if(wheel_value > 1.0) {
            return 1500;
        } else if(wheel_value < -1.0) {
            return 4000;
        }
        return 0;
    }

    int TurretController::getFlywheelsSpeed(int16_t wheel_value){
        if(wheel_value > 1 || wheel_value < -1) {
            return flywheel_max_speed;
        }
        return 0;
    }

    void TurretController::reZero(){
        current_yaw_angle = 0;
    }
};