#include "PitchController.h"
#include <cmath>
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include <cmath>
#include <bits/stdc++.h>


double targetVelo;
double currentVelo;
double targetPos;
double currentPos;
double output2;

namespace ThornBots {
    PitchController::PitchController() {

    }

    double PitchController::calculate(double currentPosition, double currentVelocity, double targetPosition, double deltaT) {
        
        double positionError = targetPosition-currentPosition;

        double targetVelocity = KP*positionError;

        targetPos = targetPosition;
        currentPos = currentPosition;

        //model based motion profile
        double maxVelocity = std::min(VELO_MAX, pastTargetVelocity+ACCEL_MAX*deltaT);
        double minVelocity = std::max(-VELO_MAX, pastTargetVelocity-ACCEL_MAX*deltaT);
        targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity); 


        currentVelo = currentVelocity;
        targetVelo = targetVelocity;
        
        //velocity controller
        double velocityError = targetVelocity-currentVelocity;
        pastTargetVelocity = targetVelocity;

        //integral velocity controller
        if(abs(pastOutput) < INT_THRESH || velocityError*buildup < 0){ //saturation detection
            if(velocityError*buildup < 0){ //overshooting
                buildup*=(1-TAKEBACK); //take back not quite half
            } else {
                buildup+=velocityError*deltaT; //integrate normally
            }
        }
        //calculation for setting target current aka velocity controller
        double targetCurrent = KSTATIC*signum(targetVelocity)+KF+KPV*velocityError+KIV*buildup;

        pastOutput = RA*targetCurrent + KV*targetVelocity;
        output2 = pastOutput*100;
        return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);

    }
}
