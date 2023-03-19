#pragma once
#include "Core.h"

namespace ThornBots {
    class DrivetrainController {
    public:
        DrivetrainController();
        void Initialize();
        void UpdateDrivetrain();
        void SetMotorSpeeds();
        void StopMotors();
        int GetMotorSpeed(const char* motor_name);
        bool SetMotorSpeed(int speed, std::string motor_name);
        inline int GetTranslationSpeed() { return m_TranslationSpeed; }
        inline int GetRotationSpeed() { return m_RotationSpeed; }
        inline int GetBeybladingSpeed() { return m_BeybladingSpeed; }
        inline void SetTranslationSpeed(int speed) { m_TranslationSpeed = speed; }
        inline void SetRotationSpeed(int speed) { m_RotationSpeed = speed; }
        inline void SetBeybladingSpeed(int speed) { m_BeybladingSpeed = speed; } 
        int CalculateMotorSpeed(const char* motor_name);
    private:
        bool KeyboardEnabled();
        bool BeybladingEnabled();
        int CalculateMotorSpeedWithKeyboard(const char* motor_name);
        int CalculateMotorSpeedWithController(const char* motor_name);
        int CalculateTranslatingSpeed(const char* motor_name);
        float GetStickAngle(float xPosition, float yPosition);
        float GetStickMagnitude(float xPosition, float yPosition);
    private:
        int m_FLSpeed, m_FRSpeed, m_BLSpeed, m_BRSpeed = 0; // motor speeds
        int m_TranslationSpeed, m_RotationSpeed, m_BeybladingSpeed = 0; // other speeds
        const int m_MaximumSpeed = 9500; // maximum speed a motor can reach
        float m_BeybladeFactor = 0.5; // percentage from [0, 1] of how much power beyblading is allotted

        // drivetrain motors
        tap::motor::DjiMotor* m_MotorFL;
        tap::motor::DjiMotor* m_MotorFR;
        tap::motor::DjiMotor* m_MotorBL;
        tap::motor::DjiMotor* m_MotorBR;
    };
}