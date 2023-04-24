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
    class CommunicationHandler {
    public:
        bool Initialize();
        void Update();
        void SendUart(const char* message);
        uint8_t GetSwitchState(const char* switchID);
        float GetStickValue(const char stickID);
        float GetWheelValue();
        bool GetIsInitialized();
        char* GetKeysPressed();
        bool GetRightMouseClicked();
        bool GetLeftMouseClicked();
        std::pair<int, int> GetMouseCoords();
        char* GetUartOutput();
    
    private:
        char* m_UartOutput;

    };
}