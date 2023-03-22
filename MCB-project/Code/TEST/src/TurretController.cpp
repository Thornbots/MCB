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
    void TurretController::setMotorValues(bool useWASD, bool doBeyblading, double angleOffset, double right_stick_vert, double right_stick_horz, int motor_one_speed, int motor_four_speed) {
        motor_yaw_speed = getYawMotorSpeed(0, angleOffset, motor_one_speed, motor_four_speed);
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
        motor_yaw.setDesiredOutput(static_cast<int32_t>(motor_yaw_speed));

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
    int TurretController::getYawMotorSpeed(int desiredAngle, int actualAngle, int motor_one_speed, int motor_four_speed) {
        // if(abs(angleOffset) > 180) {
        //     angleOffset < 0 ? angleOffset += 360 : angleOffset -= 360;
        // }
        // int speed = homemadePID(angleOffset);
        // if(abs(speed) <= motor_yaw_max_speed) { return speed; }
        // return speed < 0 ? -1.0 * motor_yaw_max_speed : motor_yaw_max_speed; 
        double kF = 0.42;
        if(abs(desiredAngle) > 180) {
            desiredAngle < 0 ? desiredAngle += 360 : desiredAngle -= 360;
        }
        if(abs(actualAngle) > 180) {
            actualAngle < 0 ? actualAngle += 360 : actualAngle -= 360;
        }
        yawPidController.runControllerDerivateError(desiredAngle - actualAngle, 1);
        tmp = actualAngle;
        return ((motor_one_speed - motor_four_speed) *kF) + yawPidController.getOutput();
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