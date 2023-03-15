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

namespace ThornBots {
    TurretController::TurretController(tap::Drivers* m_driver) {
        this->drivers = m_driver;
        this->motor_yaw.initialize();
        this->motor_indexer.initialize();
        this->motor_pitch.initialize();
        this->flywheel_one.initialize();
        this->flywheel_two.initialize();
    }

    TurretController::~TurretController() {} //Watch this cute video of a cat instead: https://youtu.be/hg3e1KflmC8

    /**
     * Updates the values of the motors' speeds. i.e. if you're beyblading, it will tell the motor_yaw_speed to change depending on what needs to change
    */
    void TurretController::setMotorValues(bool useWASD, bool doBeyblading, double angleOffset, double right_stick_vert, double right_stick_horz) {
        this->motor_yaw_speed = getYawMotorSpeed(useWASD, doBeyblading, angleOffset, right_stick_horz);
        this->motor_pitch_speed = useWASD ? getPitchMotorSpeed(useWASD, right_stick_vert, angleOffset) : 0;
        this->flywheel_speed = this->flywheel_max_speed;
        this->motor_indexer_speed = getMotorIndexerSpeed();
    }

    /**
     * Tells all the motors to go to their assined speeds
     * i.e. Tells motor_yaw to to go motor_yaw_speed and so on.
     * sendMotorTimeout should be the method call sendMotorTimeout.execute() when calling this method
    */
    void TurretController::setMotorSpeeds(bool sendMotorTimeout) {
        if(!sendMotorTimeout) { return; }
        //Yaw Motor
        pidController.runControllerDerivateError(motor_yaw_speed - motor_yaw.getShaftRPM(), 1);
        motor_yaw.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Pitch Motor
        pidController.runControllerDerivateError(motor_pitch_speed - motor_pitch.getShaftRPM(), 1);
        motor_pitch.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Indexer Motor
        pidController.runControllerDerivateError(motor_indexer_speed - motor_indexer.getShaftRPM(), 1);
        motor_indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Flywheel One
        pidController.runControllerDerivateError(flywheel_speed - flywheel_one.getShaftRPM(), 1);
        flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        //Flywheel Two
        pidController.runControllerDerivateError(flywheel_speed - flywheel_two.getShaftRPM(), 1);
        flywheel_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        drivers->djiMotorTxHandler.processCanSendData(); //Processes these motor speed changes into can signal
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

        drivers->djiMotorTxHandler.processCanSendData(); //Processes these motor speed changes into can signal
    }

    int TurretController::homemadePID(double value) {
        return (int) (1 * (((value * 100)) / 3)); 
        //TODO: Make this better (low priority)
    }

    /**
     * Returns the speed that the yaw motor needs to go to counterbalance the effect of beyblading or turning.
     * Currently only works when in beyBlading mode.
     * TODO: Make this spin the yawMotor dependent on mouse(only when NOT using CV)
    */
    int TurretController::getYawMotorSpeed(bool useWASD, bool doBeyblading, double angleOffset, double right_stick_horz) {
        if(doBeyblading) {
            if(abs(angleOffset) > 180) {
                angleOffset < 0 ? angleOffset += 360 : angleOffset -= 360;
            }
            return angleOffset > (double) 0.0 ? homemadePID(angleOffset) : -1 * homemadePID(abs(angleOffset));
        }
        if(useWASD) { 
            return (int) right_stick_horz * YAW_MOTOR_SCALAR;
        }

        return -1; //TODO: Implement this to take into acount WASD control (make it move based on mouse movement)
    }

    int TurretController::getPitchMotorSpeed(bool useWASD, double right_stick_vert, double angleOffSet) {
        return angleOffSet == (double) 0.0 ? 0 : 0; //TODO
    }

    int TurretController::getIndexerMotorSpeed() {
        return -1; //TODO
    }

};