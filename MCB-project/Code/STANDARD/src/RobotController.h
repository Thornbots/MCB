#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include <cmath>
#include "drivers_singleton.hpp"
#include "DriveTrainController.h"
#include "TurretController.h"

namespace ThornBots {
    //Don't ask me why. Timers only work when global. #Certified taproot Moment
    static tap::arch::PeriodicMilliTimer driveTrainMotorsTimer(2);
    static tap::arch::PeriodicMilliTimer turretMotorsTimer(2);
    static tap::arch::PeriodicMilliTimer IMUTimer(2);

    class RobotController {
        public: //Public Variables
            static constexpr double PI = 3.14159;
            static constexpr double MAX_SPEED = 0;
            static constexpr double FAST_BEYBLADE_FACTOR = 0.7;
            // static constexpr double 
        private: //Private Variables
            tap::Drivers *drivers;
            ThornBots::DriveTrainController *driveTrainController;
            ThornBots::TurretController *turretController;
            double left_stick_horz, left_stick_vert, right_stick_horz, right_stick_vert = 0;
            double leftStickAngle, rightStickAngle, leftStickMagnitude, rightStickMagnitude = 0;
            double wheelValue = 0;
            tap::communication::serial::Remote::SwitchState leftSwitchState, rightSwitchState = tap::communication::serial::Remote::SwitchState::MID;
        public: //Public Methods
            RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController);

            void initialize();

            void update();

            void stopRobot();

            bool toggleKeyboardAndMouse();

        private: //Private Methods
            void updateAllInputVariables();

            double getAngle(double x, double y);

            double getMagnitude(double x, double y);

    };
}