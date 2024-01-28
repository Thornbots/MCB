#pragma once
/*
* The purpose of this class is to allow easy changing or tweaking of the controller that controls the yaw motor
* All of the constants that are used for the controller shall be placed inside of this class that ultimely only implements one function.
* 
* The function implemented in this class should take quite a few inputs (i.e. going to take: angular velocity of DT and T, angular acc of DT and T, and a number of other things)
* With this information, the function should calculate the voltage level to set the yaw motor at, check that it is within valid bounds, and return that value as a double.
*/
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"

namespace ThornBots {
    class YawController {
        public: //Public Variables

        private: //Private Variables
            constexpr static tap::algorithms::SmoothPidConfig yawPIDConf_rpm = { 0, 0, 0, 0, 20000, 1, 0, 1, 0, 0, 0 };
            constexpr static tap::algorithms::SmoothPidConfig yawPIDConf_angle = { 20, 0, 0, 0, 20000, 1, 0, 1, 0, 0, 0 };
            tap::algorithms::SmoothPid rpmController = tap::algorithms::SmoothPid(yawPIDConf_rpm);
            tap::algorithms::SmoothPid angleController = tap::algorithms::SmoothPid(yawPIDConf_angle);

            double prevDesiredAngle = 0;
            double desiredVoltage = 0;

        public: //Public Methods
            YawController() {}

            double getDesiredVoltage(double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double desiredAngleWorld, double dt) {
                if(dt > 0) {} //Bye bye warnings
                
                //START Getting RPM Error Voltage
                double rpmError = driveTrainRPM + (desiredAngleWorld - prevDesiredAngle) - yawRPM;
                rpmController.runControllerDerivateError(rpmError, 1);
                double rpmVoltage = rpmController.getOutput();
                //STOP Getting RPM Error Voltage

                //START Getting Angle Error Voltage
                double angleError = desiredAngleWorld - yawAngleRelativeWorld;
                angleController.runControllerDerivateError(angleError, 1);
                double angleVoltage = angleController.getOutput();
                //STOP Getting Angle Error Voltage
        
                prevDesiredAngle = desiredAngleWorld;

                return rpmVoltage + angleVoltage;
            }

        private: //Private Methods

    };
}