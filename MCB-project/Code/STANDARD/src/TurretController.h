#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include <cmath>

#include "HardwareHandler.h"
#include "RefereeSystem.h"
#include "CommunicationHandler.h"

#include <chrono>
#include <memory>


static tap::algorithms::SmoothPidConfig pid_conf_turret = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 69, 0 };
static tap::algorithms::SmoothPidConfig pid_yaw_conf = { 180, 0, -30000, 0, 1000000, 1, 0, 1, 0, 0, 0 };    //{ 90, 10, -15000, 1, 1000000, 1, 0, 1, 0, 0, 0 };
static tap::algorithms::SmoothPidConfig pid_pitch_conf = { 800, 0.06, 80, 1500, 1000000, 1, 0, 1, 0, 0, 0 };

namespace ThornBots {
    class TurretController {
    public:
        TurretController(tap::Drivers* driver);
        ~TurretController();
        bool Initialize();
        void Update();
        void EmergencyStop();
        void Rotate(float angle);
        void OverrideWithController();
        void OverrideWithKeyboard();
        void SetShooting(bool isShooting);
        void SetPitchAngle(float angle);
        void SetYawAngle(float angle);
        bool GetShooting();
        char GetOverride();
        float GetYawAngle();
        float GetPitchAngle();
    
    private:
        // std::shared_ptr<CommunicationHandler> m_CommunicationHandler; 
        // std::shared_ptr<HardwareHandler> m_HardwareHandler;
        std::shared_ptr<RefereeSystem> m_RefereeSystem;
        char m_OverrideStatus;
        bool m_IsShooting;
        float m_YawAngle;
        float m_PitchAngle;
        uint32_t m_FlywheelL;
        uint32_t m_FlywheelR;
        uint32_t m_Indexer;
        uint32_t m_PitchMotor;
        uint32_t m_YawMotor;
    };
}