#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include <cmath>

#include "HardwareHandler.h"
#include "RefereeSystem.h"
#include "CommunicationHandler.h"

#include <chrono>
#include <memory>
#include <stdint.h>

namespace ThornBots {
    class DriveTrainController {
        public: //Methods
            DriveTrainController(std::shared_ptr<CommunicationHandler> ch, std::shared_ptr<HardwareHandler> hh, std::shared_ptr<RefereeSystem> rs);
            ~DriveTrainController() = default;
            bool Initialize();
            void Update();
            void EmergencyStop();
            //Use the controller to control the robot
            void OverrideWithController();
            //Use the keyboard to control the robot
            void OverrideWithKeyboard();
            //Reference to turret, turret is 0.
            void SetReferenceAngle(float NewAngle);
            //Rotate CW angle [degrees]
            void RotateOverride(float Angle);
            void RotateSum(float Angle);
            //Where you pass in the remote data to control the robot using manual contorl or sentry control
            void MoveDriveTrain(float Angle, uint32_t Speed);
            //Controls are based off of the turret angle (Purdue Code)
            inline void SetTurretCentric(bool IsTurretCentric) { this->IsTurretCentric = IsTurretCentric; }
            inline void SetBeyblade() { IsBeyblading = true; }
            void ResetBeyblade() { IsBeyblading = false; }
            //Drivetrain keeps its current angle offset and rotates the entire robot to turn, keeping the angleoffset the same
            void LockDrivetrain();
            void UnlockDrivetrain();
            inline void ToggleTurretCentric() { IsTurretCentric = !IsTurretCentric; }
            inline void ToggleBeyblade() { IsBeyblading = !IsBeyblading; }
            inline bool GetIsTurretCentric() { return IsTurretCentric; }
            inline bool GetIsBeyblade() { return IsBeyblading; }
        public: //Variables
        private: //Methods
        bool useWASD = false;
        bool IsBeyblading = false;
        bool IsTurretCentric = false;
        
        private: //Variables
        float BeyBladingConstant = 45.0; //Used for the turning (beyblading) calculation
        float RotatingKeyboardConstant = 45.0; //Used for the turning (using keyboard) calculation
        float RotatingControllerConstant = 90.0; //Used for the turning (using joystick) calculation
        int16_t* FL_WheelSpeed;
        int16_t* FR_WheelSpeed;
        int16_t* BL_WheelSpeed;
        int16_t* BR_WheelSpeed;
        std::shared_ptr<CommunicationHandler> m_CommunicationHandler; 
        std::shared_ptr<RefereeSystem> m_RefereeSystem;
        std::shared_ptr<HardwareHandler> m_HardwareHandler;

    };
}