#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"

namespace ThornBots {
    class HardwareHandler {
    public:
        HardwareHandler();
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
        float getMotorAngle(int motorID);
    
    private:
        bool m_IsDataSent;
        tap::motor::DjiMotor m_MotorArray[2][8];
        bool m_IsInitialized;
    };
}