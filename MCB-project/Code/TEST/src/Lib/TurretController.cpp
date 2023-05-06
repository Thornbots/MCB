#pragma once
#include "TurretController.h"
#include <cmath>
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "DriveTrainController.h"
#include <cmath>

namespace ThornBots {
    
    TurretController::TurretController() {
    }

    bool TurretController::Initialize(){
        // TODO: Replace me! Pass a reference to these classes into some function
        // this->m_CommunicationHandler = std::make_shared<CommunicationHandler>(new CommunicationHandler());
        // this->m_HardwareHandler = std::make_shared<HardwareHandler>(new HardwareHandler());
        // this->m_RefereeSystem = std::make_shared<RefereeSystem>(new RefereeSystem());
    }

    void TurretController::Rotate(float angle){
        this->m_YawAngle += angle;
    }

    void TurretController::Update(){  
        m_HardwareHandler.get()->SetMotorPowerOutput(Motor::MOTOR_PITCH, getPitchMotorSpeed(GetPitchAngle()));
        m_HardwareHandler.get()->SetMotorPowerOutput(Motor::MOTOR_INDEXER, getIndexerMotorSpeed());

        float motor_one_speed  = m_HardwareHandler.get()->GetMotorShaftRPM(Motor::MOTOR_FRONT_LEFT);
        float motor_four_speed = m_HardwareHandler.get()->GetMotorShaftRPM(Motor::MOTOR_BACK_RIGHT);
        m_HardwareHandler.get()->SetMotorPowerOutput(Motor::MOTOR_PITCH, getYawMotorSpeed(GetYawAngle(), motor_one_speed, motor_four_speed));
    
        UpdateMotor(Motor::MOTOR_INDEXER);
        UpdateMotor(Motor::FLYWHEEL_RIGHT);
        UpdateMotor(Motor::FLYWHEEL_LEFT);
        UpdateMotor(Motor::MOTOR_YAW);
        UpdateMotor(Motor::MOTOR_PITCH);
    }

    void TurretController::UpdateMotor(Motor motorID) {
        pidController.runControllerDerivateError(0 - m_HardwareHandler.get()->GetMotorShaftRPM(motorID), 1);
        m_HardwareHandler.get()->SetMotorPowerOutput(motorID, pidController.getOutput());
    }

    void TurretController::EmergencyStop(){
        Motor motors[] = {Motor::MOTOR_INDEXER, Motor::MOTOR_PITCH, Motor::MOTOR_YAW, Motor::FLYWHEEL_LEFT, Motor::FLYWHEEL_RIGHT};
        for(Motor motor : motors){
            m_HardwareHandler.get()->SetMotorPowerOutput(motor, 0);
        }
    }

    uint32_t TurretController::getPitchMotorSpeed(double target_angle) {
        const float ENC_RESOLUTION = 8192;

        float encoder = (this->m_HardwareHandler.get()->GetMotorAngle(Motor::MOTOR_PITCH));
        float position = (360.0f * (encoder)) / ENC_RESOLUTION; 
        float desired = 270.0 + target_angle;
        
        this->pitchPidController.runControllerDerivateError(desired - position, 1);
        uint32_t speed = pitchPidController.getOutput();
        return std::min(speed, motor_pitch_max_speed);
    }

    uint32_t TurretController::getYawMotorSpeed(double desiredAngle, uint32_t motor_one_speed, uint32_t motor_four_speed) {
        double kF = 0.42;
        float actualAngle = m_HardwareHandler.get()->GetMotorAngle(Motor::MOTOR_YAW);
        while(actualAngle-desiredAngle > 180){
            actualAngle -= 360;
        }
        while(desiredAngle-actualAngle > 180){
            actualAngle += 360;
        }
        yawPidController.runControllerDerivateError(desiredAngle-actualAngle, 1);
        double tmp = actualAngle;
        // TODO: Should I constrain this value to [0, motor_yaw_max_speed]?
        uint32_t speed = ((motor_one_speed - motor_four_speed) *kF) + yawPidController.getOutput();
        return std::min(speed, motor_yaw_max_speed);
    }

    uint32_t TurretController::getFlywheelsSpeed(){
        if(isShooting){
            return flywheel_max_speed;
        }else{
            return 0; //So we don't have to rev it up to 100% to start firing the next time (Maybe change dependent on power consumption)
        }
    }

    void TurretController::OverrideWithController(){
        this->m_OverrideStatus = 'c';
    }

    void TurretController::OverrideWithKeyboard(){
        this->m_OverrideStatus = 'k';
    }

    void TurretController::SetShooting(bool isShooting){
        this->isShooting = isShooting;
    }

    void TurretController::SetPitchAngle(float angle){
        this->m_PitchAngle = angle;
    }

    void TurretController::SetYawAngle(float angle){
        this->m_YawAngle = angle;
    }

    bool TurretController::GetShooting(){
        return isShooting;
    }

    char TurretController::GetOverride(){
        return m_OverrideStatus;
    }

    float TurretController::GetYawAngle(){
        return m_YawAngle;
    }

    float TurretController::GetPitchAngle(){
        return m_PitchAngle;
    }

    uint32_t TurretController::getIndexerMotorSpeed() {
        if(this->isShooting){
            return motor_indexer_max_speed;
        }else{
            return 0; //No firing, so no spinny spin spin =)
        }
    }

    
    
};