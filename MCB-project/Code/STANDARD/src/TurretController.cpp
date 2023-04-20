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
    
    TurretController::TurretController() {
    }

    bool TurretController::Initialize(){
        this->m_CommunicationHandler = std::make_shared<CommunicationHandler>(new CommunicationHandler());
        this->m_HardwareHandler = std::make_shared<HardwareHandler>(new HardwareHandler());
        this->m_RefereeSystem = std::make_shared<RefereeSystem>(new RefereeSystem());
    }

    void TurretController::Rotate(float angle){
        this->m_YawAngle += angle;
    }

    void TurretController::Update(){  
        m_HardwareHandler.get()->SetMotorOutput(motor_pitch, getPitchMotorSpeed(GetPitchAngle()));
        m_HardwareHandler.get()->SetMotorOutput(motor_indexer, getIndexerMotorSpeed());

        float motor_one_speed  = m_HardwareHandler.get()->GetMotorShaftRPM("11");
        float motor_four_speed = m_HardwareHandler.get()->GetMotorShaftRPM("14");
        m_HardwareHandler.get()->SetMotorOutput(motor_pitch, getYawMotorSpeed(GetYawAngle(), motor_one_speed, motor_four_speed));
    
        UpdateMotor(motor_indexer);
        UpdateMotor(flywheel_right);
        UpdateMotor(flywheel_left);
        UpdateMotor(motor_yaw);
        UpdateMotor(motor_pitch);
    }

    void TurretController::UpdateMotor(char* motorID) {
        pidController.runControllerDerivateError(0 - m_HardwareHandler.get()->GetMotorShaftRPM(motorID), 1);
        m_HardwareHandler.get()->SetMotorOutput(motorID, pidController.getOutput());
    }

    void TurretController::EmergencyStop(){
        char* motors[] = {motor_indexer, motor_pitch, motor_yaw, flywheel_left, flywheel_right};
        for(char* motor : motors){
            m_HardwareHandler.get()->SetMotorOutput(motor, 0);
        }
        
    }

    int TurretController::getPitchMotorSpeed(double target_angle) {
        float position = tap::motor::DjiMotor::encoderToDegrees(this->m_HardwareHandler.get()->getMotorAngle(26));
        float desired = 270.0f + target_angle;
        this->pitchPidController.runControllerDerivateError(desired - position, 1);
        return pitchPidController.getOutput();
    }

    int TurretController::getYawMotorSpeed(double desiredAngle, int motor_one_speed, int motor_four_speed) {
        double kF = 0.42;
        float actualAngle = m_HardwareHandler.get()->getMotorAngle(17);
        while(actualAngle-desiredAngle > 180){
            actualAngle -= 360;
        }
        while(desiredAngle-actualAngle > 180){
            actualAngle += 360;
        }
        yawPidController.runControllerDerivateError(desiredAngle-actualAngle, 1);
        double tmp = actualAngle;
        return ((motor_one_speed - motor_four_speed) *kF) + yawPidController.getOutput();
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

    int TurretController::getIndexerMotorSpeed() {
        if(this->isShooting){
            return motor_indexer_max_speed;
        }else{
            return 0; //No firing, so no spinny spin spin =)
        }
    }

    
    
};