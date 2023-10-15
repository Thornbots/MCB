#include "Core.h"
#include <stdint.h>
#include "../Taproot/drivers_singleton.hpp"
#pragma once

namespace ThornBots{
    class RefereeSystem{
        public:
            RefereeSystem(src::Drivers *drivers);
            //Init 

            bool Initialize();

            //Robot Data

            uint16_t GetRobotHP();

            bool IsBlueTeam();

            float ReceivedDPS();

            //Chasis

            float GetChasisPower();

            float GetChasisMaxPower();

            //Turret

            uint8_t GetFiringFrequency();

            uint8_t GetBulletSpeed();

            //Small Shooter
            uint16_t GetSmallShooterHeat();

            uint16_t GetSmallShooterCoolingRate();

            uint16_t GetSmallShooterHeatLimit();

            uint16_t GetSmallShooterBarrelSpeedLimit();

            //Big Shooter
            uint16_t GetBigShooterHeat();

            uint16_t GetBigShooterCoolingRate();

            uint16_t GetBigShooterHeatLimit();

            uint16_t GetBigShooterBarrelSpeedLimit();
        private:
            bool m_isInitialized;
            src::Drivers *drivers;
    };
}