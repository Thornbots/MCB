#include "ModeledTurretController.h"
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

namespace ThornBots {
    ModeledTurretController::ModeledTurretController() {

    }

    double ModeledTurretController::calculate(double currentPosition, double currentVelocity, double currentDrivetrainVelocity, double targetPosition, double deltaT) {
        double positionError = targetPosition-currentPosition;
        while (positionError > M_PI){
            positionError -= M_TWOPI;
        } 
        while (positionError < -M_PI){
            positionError += M_TWOPI;
        }

        double choiceKDT = currentDrivetrainVelocity*positionError > 0 ? KDT : KDT_REV;  //check if turret is fighting drivetrain;
        double targetVelocity = (KP+signum(currentDrivetrainVelocity)*choiceKDT)*positionError;

        //model based motion profile
        double aMaxTemp = (C+(KB*KT*pow(RATIO, 2))/RA+UK*signum(currentDrivetrainVelocity-currentVelocity));
        double maxVelocity = std::min(VELO_MAX, pastTargetVelocity + 1/J*(aMaxTemp+(VOLT_MAX*KT*RATIO)/RA)*A_SCALE*deltaT);
        double minVelocity = std::max(-VELO_MAX, pastTargetVelocity + 1/J*(aMaxTemp-(VOLT_MAX*KT*RATIO)/RA)*A_SCALE*deltaT);
        targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity); 


        //velocity controller
        double velocityError = targetVelocity-currentVelocity;
        double targetRelativeVelocity = targetVelocity-currentDrivetrainVelocity;
        double targetAcceleration = (targetVelocity-pastTargetVelocity)/deltaT;
        pastTargetVelocity = targetVelocity;

        //integral velocity controller
        if(abs(pastOutput) < INT_THRESH || velocityError*buildup < 0){ //saturation detection
            if(velocityError*buildup < 0){ //overshooting
                buildup*=(1-TAKEBACK); //take back not quite half
            } 
            buildup+=velocityError*deltaT; //integrate normally
    
        }
        //calculation for setting target current aka velocity controller
        double targetCurrent = KVISC*targetRelativeVelocity+UK*signum(targetRelativeVelocity)+KA*targetAcceleration+KPV*velocityError+KIV*buildup;

        pastOutput = RA*targetCurrent + KV*targetRelativeVelocity;
        return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);

    }
}
