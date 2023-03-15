#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"

#pragma once
namespace ThornBots {
    static tap::algorithms::SmoothPidConfig pid_config;
    
    #define PID_CONFIG pid_config
}