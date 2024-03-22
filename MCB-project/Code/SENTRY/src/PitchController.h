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
#include <cmath>


namespace ThornBots {
    class PitchController {
    public:
        PitchController();
        ~PitchController();
        double calculate(double currentPosition, double currentVelocity, double targetPosition, double deltaT);

        
    private:
        //START getters and setters
        double buildup = 0;
        double pastTargetVelocity = 0;
        double pastOutput = 0;
        

        // Physical constants
        const double KB = 0.716;               // V-rad/s
        const double RA = 8.705;               // ohm
        const double RATIO = 1;                // unitless
        const double VOLT_MAX = 22.2;                   //V
        const double VELO_MAX = VOLT_MAX/(KB*RATIO);    //rad/s
        const double ACCEL_MAX = 40.0;                //rad/s

        // Position controller constants
        const double KP = 10;                // sec^-1

        // Feedforward constants
        const double KSTATIC = 0.1;               // A
        const double KV = KB * RATIO;          // V-s/rad
        const double KF = 0.05;//-0.001;                    // A


        // Velocity feedback
        const double KPV = 0.5; //0.3                  // A-s/rad
        const double KIV = 6; //2                 // A/rad
        const double IV_MAX = 0.1;             // units TBD
        const double INT_THRESH = VOLT_MAX * 0.85;   // V
        const double TAKEBACK = 0.01;          // unitless


        double signum(double num) { 
            return (num > 0) ? 1 : ((num < 0) ? -1 : 0);
        }
 
    };
}
