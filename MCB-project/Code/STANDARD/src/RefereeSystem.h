#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include "tap/motor/dji_motor.hpp"


namespace ThornBots {
    class RefereeSystem {
    public:
        bool Initialize();
        void Update();
        float GetChasisPower();
        float GetGimbalPower();
        float GetShooterPower();
        uint32_t GetHealth();
        uint32_t GetShooterSpeed();
        uint32_t GetShooterVelocity();

    private:
        bool m_IsInitialized;
    };

}