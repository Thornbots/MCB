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
             * Enum for *all* of the motors on the robot (NOT SERVOS). The first digit is for which CAN bus the motor is on (MINUS ONE)).
             * The second digit is for which tap::motor::motorID the motor is on (MINUS ONE).
            */
            enum Motor {
                MOTOR_FRONT_LEFT = 00,
                MOTOR_FRONT_RIGHT = 01,
                MOTOR_BACK_LEFT = 02,
                MOTOR_BACK_RIGHT = 03,
                MOTOR_YAW = 06,
                MOTOR_PITCH = 15,
                MOTOR_INDEXER = 16,
                FLYWHEEL_LEFT = 17,
                FLYWHEEL_RIGHT = 14,
            };

            enum IMU_Radial {
                PITCH = 0,
                YAW = 1,
                ROLL = 2
            };

            enum IMU_Cardinal {
                X = 0,
                Y = 1,
                Z = 2
            };
            
            //STOP Enuuums
            //START Methods
            bool Initialize();
            void UpdateIMU();
            void CalibrateIMU();
            void SetMotorPowerOutput(Motor MotorID, int32_t Output);
            void PollCanData();
            void SendCanData();
            int32_t GetMotorShaftRPM(Motor MotorID);
            float GetMotorAngle(Motor MotorID);
            float GetIMUAngle();
            float GetMotorAngle(Motor MotorID);
            float GetIMUAngle(IMU_Radial Axis);
            float GetIMUVelocity(IMU_Cardinal Axis);
            float GetIMUAcceleration(IMU_Cardinal Axis);
            //STOP Methods

        private:
            static constexpr int NUM_MOTORS = 8;
            tap::motor::DjiMotor* MotorArray[1][NUM_MOTORS - 1];
            int getRow(Motor MotorID);
            int getColumn(Motor MotorID);
            tap::motor::DjiMotor* getMotor(Motor MotorID);
    };
}