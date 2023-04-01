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
static tap::algorithms::SmoothPidConfig pid_conf_dt = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 500, 0 };
static constexpr float REFINED_ANGLE_OFFSET = 210.0f;


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