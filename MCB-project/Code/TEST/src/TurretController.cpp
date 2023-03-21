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
        motor_yaw.initialize();
        motor_indexer.initialize();
        motor_pitch.initialize();
        flywheel_one.initialize();
        flywheel_two.initialize();
    }

    TurretController::~TurretController() {} //Watch this cute video of a cat instead: https://youtu.be/hg3e1KflmC8

    /**
     * Updates the values of the motors' speeds. i.e. if you're beyblading, it will tell the motor_yaw_speed to change depending on what needs to change
    */
    void TurretController::setMotorValues(bool useWASD, bool doBeyblading, double angleOffset, double right_stick_vert, double right_stick_horz) {
        motor_yaw_speed = getYawMotorSpeed(useWASD, doBeyblading, angleOffset, right_stick_horz);
        motor_pitch_speed = 0.0;//useWASD ? getPitchMotorSpeed(useWASD, right_stick_vert, angleOffset) : 0;
        flywheel_speed = getFlywheelsSpeed();
        motor_indexer_speed = getIndexerMotorSpeed();
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
        pidController.runControllerDerivateError(motor_yaw_speed - motor_yaw.getShaftRPM(), 1);
        motor_yaw.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Pitch Motor
        pidController.runControllerDerivateError(motor_pitch_speed - motor_pitch.getShaftRPM(), 1);
        motor_pitch.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Indexer Motor
        pidController.runControllerDerivateError(motor_indexer_speed - motor_indexer.getShaftRPM(), 1);
        motor_indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();
        
        //Flywheel One
        pidController.runControllerDerivateError(flywheel_speed - flywheel_one.getShaftRPM(), 1);
        flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();
        
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
        drivers->canRxHandler.pollCanData();
        motor_yaw_speed = 0;
        motor_pitch_speed = 0;
        motor_indexer_speed = 0;
        flywheel_speed = 0;

        //Yaw Motor
        pidController.runControllerDerivateError(0 - motor_yaw.getShaftRPM(), 1);
        motor_yaw.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Pitch Motor
        pidController.runControllerDerivateError(0 - motor_pitch.getShaftRPM(), 1);
        motor_pitch.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Indexer Motor
        pidController.runControllerDerivateError(0 - motor_indexer.getShaftRPM(), 1);
        motor_indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Flywheel One
        pidController.runControllerDerivateError(0 - flywheel_one.getShaftRPM(), 1);
        flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();

        //Flywheel Two
        pidController.runControllerDerivateError(0 - flywheel_two.getShaftRPM(), 1);
        flywheel_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
        drivers->djiMotorTxHandler.encodeAndSendCanData();
    }

    void TurretController::startShooting(){
        isShooting = true;
    }
    
    void TurretController::stopShooting(){
        isShooting = false;
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
        if(abs(angleOffset) > 180) {
            angleOffset < 0 ? angleOffset += 360 : angleOffset -= 360;
        }
        return angleOffset > (double) 0.0 ? homemadePID(angleOffset) : -1 * homemadePID(abs(angleOffset));
    }

    int TurretController::getPitchMotorSpeed(bool useWASD, double right_stick_vert, double angleOffSet) {
        return angleOffSet == (double) 0.0 ? 0 : 0; //TODO
    }

    int TurretController::getIndexerMotorSpeed() {
        if(isShooting){
            return motor_indexer_max_speed;
        }else{
            return 0; //No firing, so no spinny spin spin =)
        }
    }

    int TurretController::getFlywheelsSpeed(){
        if(isShooting){
            return flywheel_max_speed;
        }else{
            return 0; //So we don't have to rev it up to 100% to start firing the next time (Maybe change dependent on power consumption)
        }
    }
};