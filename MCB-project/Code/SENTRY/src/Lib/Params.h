#pragma once

#define PI 3.14159f
#define CANBUS_ID tap::can::CanBus::CAN_BUS1
#define FL_MOTORID tap::motor::MotorId::MOTOR1
#define FR_MOTORID tap::motor::MotorId::MOTOR2
#define BL_MOTORID tap::motor::MotorId::MOTOR3
#define BR_MOTORID tap::motor::MotorId::MOTOR4

#define START_MOTOR(...) StartMotor(__VA_ARGS__)
#define GET_PID() GetPid()

/**
 * @brief Static method to return a motor object given a motor ID
 * 
 * @param motorID the motor ID to initialize
 */
static void StartMotor( tap::motor::MotorId motorID) {
    return tap::motor::DjiMotor(::DoNotUse_getDrivers(), motorID, CANBUS_ID, false, "swag motor", 0, 0);
}

/**
 * @brief Get the pidcontroller
 */
static void GetPid() {
    return tap::algorithms::SmoothPid(20, 0, 0, 0, 8000, 1, 0, 1, 0);
}