#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include "tap/motor/dji_motor.hpp"

namespace ThornBots {
    class HardwareHandler{
    public:
        bool Initialize();
        void UpdateIMU();
        void CalibrateIMU();
        void SetMotorOutput(const char* motorID, int output);
        void PollCanData();
        void SendCanData();
        uint32_t GetMotorShaftRPM(const char* motorID);
        bool GetIsDataSent();
        float GetMotorAngle(const char* motorID);
        float GetIMUAngle();
        bool GetIsIMUCalibrated();

    private:
        bool m_IsDataSent;
        tap::motor::DjiMotor m_MotorArray[2][8];
        bool m_IsInitialized;
    };


}