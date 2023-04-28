#pragma once
#include "HardwareHandler.h"
#include "RefereeSystem.h"
#include "CommunicationHandler.h"

namespace ThornBots {
    class DriveTrainController {
        public: //Methods
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
            void Rotate(float Angle);
            //Where you pass in the remote data to control the robot using manual contorl or sentry control
            void MoveDriveTrain(float Angle, uint32_t Speed);
            //Controls are based off of the turret angle (Purdue Code)
            void SetTurretCentric(bool IsTurretCentric);
            void SetBeyblade(bool IsBeyblading);
            void ResetBeyblade(bool IsBeyblading);
            //Drivetrain keeps its current angle offset and rotates the entire robot to turn, keeping the angleoffset the same
            void LockDrivetrain();
            void UnlockDrivetrain();
            void ToggleTurretCentric();
            void ToggleBeyblade();
            void GetIsTurretCentric();
            inline bool GetIsBeyblade() { return IsBeyblading; }
        public: //Variables
        private: //Methods
        bool IsBeyblading = false;
        bool IsTurretCentric = false;
        
        private: //Variables
        int16_t* FL_Wheel;
        int16_t* FR_Wheel;
        int16_t* BL_Wheel;
        int16_t* BR_Wheel;
    };
}