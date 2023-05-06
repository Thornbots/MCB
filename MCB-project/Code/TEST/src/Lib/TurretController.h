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
#include <stdint.h>

static tap::algorithms::SmoothPidConfig pid_conf_turret = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
static tap::algorithms::SmoothPidConfig pid_yaw_conf = { 130, 0, -25000, 0, 1000000, 1, 0, 1, 0, 0, 0 };    //{ 90, 10, 30, 1, 1000000, 1, 0, 1, 0, 0, 0 };
static tap::algorithms::SmoothPidConfig pid_pitch_conf = { 400, 0.06, 80, 1500, 1000000, 1, 0, 1, 0, 0, 0 };

namespace ThornBots {
    class TurretController {
    public:
        TurretController();
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
        std::shared_ptr<CommunicationHandler> m_CommunicationHandler; 
        std::shared_ptr<HardwareHandler> m_HardwareHandler;
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
        tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_turret);
        tap::algorithms::SmoothPid yawPidController = tap::algorithms::SmoothPid(pid_yaw_conf);
        tap::algorithms::SmoothPid pitchPidController = tap::algorithms::SmoothPid(pid_pitch_conf);    

        static constexpr uint32_t motor_yaw_max_speed = 500; //Not sure if this is the absolute maximum. need to test. Motor documentation says 320, but it can def spin faster than taproot's 320 rpm.
        static constexpr uint32_t motor_indexer_max_speed = 6000;
        static constexpr uint32_t flywheel_max_speed = 6700;
        static constexpr uint32_t motor_pitch_max_speed = 900;
        static constexpr uint32_t YAW_MOTOR_SCALAR = 500;
        bool isShooting = false;
    
        double current_yaw_angle = 180;

        uint32_t getYawMotorSpeed(double desiredAngle, uint32_t motor_one_speed, uint32_t motor_two_speed);
        uint32_t getPitchMotorSpeed(double target_angle);
        uint32_t getIndexerMotorSpeed();
        uint32_t getFlywheelsSpeed();
        void UpdateMotor(Motor motorID);
        
    };
}