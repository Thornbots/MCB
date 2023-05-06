#pragma once
#include "Core.h"
#include <inttypes.h>

namespace ThornBots {
    /**
     * @brief Enum for *all* of the motors on the robot (NOT SERVOS). The first digit is for which CAN bus the motor is on (MINUS ONE)).
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

    /**
     * @brief Enum to indicate a certain radial axis of the IMU.
     */
    enum IMU_Radial {
        PITCH = 0,
        YAW = 1,
        ROLL = 2
    };

    /**
     * @brief Enum to indicate a certain cardinal axis of the IMU.
     */
    enum IMU_Cardinal {
        X = 0,
        Y = 1,
        Z = 2
    };

    class HardwareHandler {
        public:
            /**
             * @brief Construct a new Hardware Handler object
             */
            HardwareHandler();

            /**
             * @brief Destructor for a HardwareHandler. Not being used, so watch these otters holding hands: https://youtu.be/epUk3T2Kfno
             */
            ~HardwareHandler() = default;

            /**
             * @brief Initializes all the necessary hardware for the hardware handler. This includes all the motors, the IMU, CAN, and the servos
             * 
             * @return true if successfully initialized
             * @return false if could not initialize correctly
             */
            bool Initialize();

            /**
             * @brief Updates the IMU readings for the current cycle
             */
            void UpdateIMU();

            /**
             * @brief Calibrates the IMU. Note that this function is blocking, so other code will not run until it has finished calibrating
             */
            void CalibrateIMU();

            /**
             * @brief Set the motor speeds to the corresponding motorID. Note that this does not use the pid controller, it is just raw output
             * 
             * @param MotorID The motor to set power output of
             * @param Output the output to set the motor to
             */
            void SetMotorPowerOutput(Motor MotorID, int32_t Output);

            /**
             * @brief Reads the can bus response
             */
            void PollCanData();

            /**
             * @brief Send the can data over the can buses
             */
            void SendCanData();

            /**
             * @brief Get the Motor Shaft RPM of a motor
             * 
             * @param MotorID the ID of the motor
             * @return int32_t the shaft RPM
             */
            int32_t GetMotorShaftRPM(Motor MotorID);

            /**
             * @brief Get the angle of a motor based on the encoder readings in degrees
             * 
             * @param MotorID the ID of the motor
             * @return float the angle in degrees
             */
            float GetMotorAngle(Motor MotorID);

            /**
             * @brief Gets the angle of the IMU
             * 
             * @param Axis the IMU radial axis
             * @return float the angle of the IMU in degrees
             */
            float GetIMUAngle(IMU_Radial Axis);

            /**
             * @brief Gets the velocity of the IMU in a cardinal direction
             * 
             * @param Axis The IMU cardinal axis
             * @return float the velocity of the IMU in ???
             */
            float GetIMUVelocity(IMU_Cardinal Axis);

            /**
             * @brief Gets the acceleration of the IMU in a cardinal direction
             * 
             * @param Axis The IMU cardinal axis
             * @return float the acceleration of the IMU in ???
             */
            float GetIMUAcceleration(IMU_Cardinal Axis);

            /**
             * @brief Gets if the hardware handler is initialized
             * 
             * @return true if the hardware handler is initialied
             * @return false if the hardware handler is not initialized
             */
            bool GetIsInitialized() { return m_IsInitialized; }

            /**
             * @brief Get if the IMU is calibrated
             * 
             * @return true if the IMU is calibrated 
             * @return false if the IMU is not calibrated
             */
            bool GetIsImuCalibrated() { return m_IsImuCalibrated; }
        private:
            /**
             * @brief Helper function to get the row of a motor ID to plug into the 2D Motor array
             * 
             * @param MotorID The ID of the motor
             * @return int row of the motor
             */
            int GetRow(Motor MotorID);

            /**
             * @brief Helper function to get the column of a motor ID to plug into the 2D Motor array
             * 
             * @param MotorID The ID of the motor
             * @return int column of the motor
             */
            int GetColumn(Motor MotorID);

            /**
             * @brief Get the Motor object based on the motorID
             * 
             * @param MotorID The ID of the Motor
             * @return tap::motor::DjiMotor* the motor pointer to the motor object
             */
            tap::motor::DjiMotor* GetMotor(Motor MotorID);
        private:
            bool m_IsInitialized, m_IsImuCalibrated = false; //helper booleans
            tap::motor::DjiMotor* MotorArray[2][8]; //2D array of motor objects
    };
}