#include "TurretController.h"

namespace ThornBots {
    TurretController::TurretController() {
        m_MotorYaw     = &MOTOR_5;
        m_MotorPitch   = &MOTOR_6;
        m_MotorIndexer = &MOTOR_7;
        m_FlywheelR    = &MOTOR_8;
        m_FlywheelL    = &MOTOR_9;
    }

    void TurretController::Initialize() {
        m_MotorYaw->initialize();
        m_MotorPitch->initialize();
        m_MotorIndexer->initialize();
        m_FlywheelR->initialize();
        m_FlywheelL->initialize();
    }

    int TurretController::GetMotorSpeed(const char motor_name) {
        switch(motor_name) {
            case 'Y':
                return m_YawSpeed;
            case 'P':
                return m_PitchSpeed;
            case 'R':
                return m_FlywheelSpeed;
            case 'L':
                return m_FlywheelSpeed;
            case 'I':
                return m_IndexerSpeed;
            default:
                return -1;
        }
    }

    int TurretController::GetMaxMotorSpeed(const char motor_name) {
        switch(motor_name) {
            case 'Y':
                return m_MaxYawSpeed;
            case 'P':
                return m_MaxPitchSpeed;
            case 'R':
                return m_MaxFlywheelSpeed;
            case 'L':
                return m_MaxFlywheelSpeed;
            case 'I':
                return m_MaxIndexerSpeed;
            default:
                return -1;
        }
    }

    bool TurretController::SetMotorSpeed(const int speed, const char motor_name) {
        switch(motor_name) {
            case 'Y':
                m_YawSpeed = speed;
                return true;
            case 'P':
                m_PitchSpeed = speed;
                return true;
            case 'R':
                m_FlywheelSpeed = speed;
                return true;
            case 'L':
                m_FlywheelSpeed = speed;
                return true;
            case 'I':
                m_IndexerSpeed = speed;
                return true;
            default:
                return false;
        }
    }

    void TurretController::SendMotorSpeeds() {
        if(TURRET_TIMER.execute()){
            SPIN_MOTOR(m_YawSpeed,      m_MotorYaw);
            SPIN_MOTOR(m_PitchSpeed,    m_MotorPitch);
            SPIN_MOTOR(m_IndexerSpeed,  m_MotorIndexer);
            SPIN_MOTOR(m_FlywheelSpeed, m_FlywheelL);
            SPIN_MOTOR(m_FlywheelSpeed, m_FlywheelR);
        }
    }

    void TurretController::StopMotors() {
        m_YawSpeed = 0;
        m_PitchSpeed = 0;
        m_IndexerSpeed = 0;
        m_FlywheelSpeed = 0;
        SendMotorSpeeds();
    }

    void TurretController::UpdateTurretController() {
        m_YawSpeed = CalculateMotorSpeed('Y');
        m_YawSpeed = CalculateMotorSpeed('I');
        m_YawSpeed = CalculateMotorSpeed('P');
        m_YawSpeed = CalculateMotorSpeed('R');
        m_YawSpeed = CalculateMotorSpeed('L');
        SendMotorSpeeds();
    }

    bool TurretController::KeyboardEnabled() {
        return false;
    }

    int TurretController::CalculateMotorSpeed(const char motor_name) {
        if(KeyboardEnabled()) return CalculateMotorSpeedWithKeyboard(motor_name);
        return CalculateMotorSpeedWithController(motor_name);
    }

    int TurretController::CalculateMotorSpeedWithKeyboard(const char motor_name) {
        //TODO
        return 0;
    }

    int TurretController::CalculateMotorSpeedWithController(const char motor_name) {
        switch(motor_name) {
            case 'Y':
                return 0;
            case 'P':
                return 0;
            case 'R':
                return m_MaxFlywheelSpeed;
            case 'L':
                return m_MaxFlywheelSpeed;
            case 'I':
                return m_MaxIndexerSpeed;
            default:
                return -1;
        }
    }

    int TurretController::CalculateTurretPID(float value) {
        return (int) (1 * (((value * 100)) / 3)); 
    }

};