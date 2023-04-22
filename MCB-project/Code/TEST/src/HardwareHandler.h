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


namespace ThornBots {
    class HardwareHandler {
        public:
            HardwareHandler();
            ~HardwareHandler() = default; //Destructor for a HardwareHandler. Not being used, so watch these otters holding hands: https://youtu.be/epUk3T2Kfno
            //START Variables
            bool m_IsInitialized, m_IsDataSent, m_IsImuCalibrated = false;
            //STOP Variables
            //START Enuuums
            /**
             * Enum for *all* of the motors on the robot (NOT SERVOS). The first digit is for which CAN bus the motor is on.
             * The second digit is for which tap::motor::motorID the motor is on. (Relatively useless for )
            */
            enum Motor {
                MOTOR_FRONT_LEFT = 11,
                MOTOR_FRONT_RIGHT = 12,
                MOTOR_BACK_LEFT = 13,
                MOTOR_BACK_RIGHT = 14,
                MOTOR_YAW = 17,
                MOTOR_PITCH = 26,
                MOTOR_INDEXER = 27,
                FLYWHEEL_LEFT = 28,
                FLYWHEEL_RIGHT = 25,
            };
            //STOP Enuuums
            //START Methods
            bool Initialize();
            void UpdateIMU();
            void CalibrateIMU();
            void SetMotorPowerOutput(Motor motorID, int Output);
            void SetMotorRPMOutput(Motor motorID, int RPM);
            void PollCanData();
            void SendCanData();
            int32_t GetMotorShaftRPM(Motor motorID);
            bool GetIsDataSent();
            float GetMotorAngle(Motor motorID);
            float GetIMUAngle();
            float GetMotorAngle(Motor motorID);
            //STOP Method

        private:
            static constexpr int NUM_MOTORS = 8;
            tap::motor::DjiMotor* MotorArray[1][NUM_MOTORS - 1];
    };
}