#pragma once
#include "Core.h"
#include "DriveTrainController.h"
#include "TurretController.h"

namespace ThornBots{
    class RobotController{
        public:
            RobotController();
            bool Initialize();
            void Update();
            void EmergencyStop();
        private:
            ThornBots::DriveTrainController *s_DriveTrainController;
            ThornBots::TurretController *s_TurretController;
    };
}