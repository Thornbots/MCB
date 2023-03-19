#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"

#pragma once
namespace ThornBots {
    static tap::algorithms::SmoothPidConfig conf = {
    20,
    0,
    0,
    0,
    8000,
    1,
    0,
    1,
    0,
    0,
    0
};
    
    #define PID_CONFIG conf
}