#pragma once
#include "RobotController.h"
#include <stdint.h>

namespace ThornBots{
    class EntryPoint{
        public:
            int main();
            bool Initialize();
            void Update(uint32_t cycleTimeInUS, bool runRobot);
            void Destroy();
        private:
            // ThornBots::RobotController *m_RobotController = new ThornBots::RobotController();
    };
}