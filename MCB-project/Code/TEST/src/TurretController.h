#pragma once
#include "Core.h"

namespace ThornBots {
    class TurretController {
    public:
        TurretController();
        void Initialize();
        int GetMotorSpeed(const char motor_name);
        int GetMaxMotorSpeed(const char motor_name);
        bool SetMotorSpeed(const int speed, const char motor_name);
        void SendMotorSpeeds();
        void UpdateTurretController();
        void StopMotors();
    private:
        bool KeyboardEnabled();
        int CalculateMotorSpeed(const char motor_name);
        int CalculateMotorSpeedWithKeyboard(const char motor_name);
        int CalculateMotorSpeedWithController(const char motor_name);
        int CalculateTurretPID(float value);
    private:
        // speeds
        int m_YawSpeed, m_PitchSpeed, m_IndexerSpeed, m_FlywheelSpeed = 0;

        //constants
        const int m_MaxYawSpeed = 500;
        const int m_MaxIndexerSpeed = 500;
        const int m_MaxFlywheelSpeed = 960;
        const int m_MaxPitchSpeed = 500;
        const int m_YawScalar = 500;

        // motors
        tap::motor::DjiMotor* m_MotorYaw;
        tap::motor::DjiMotor* m_MotorPitch;
        tap::motor::DjiMotor* m_MotorIndexer;
        tap::motor::DjiMotor* m_FlywheelR;
        tap::motor::DjiMotor* m_FlywheelL;
    };
}