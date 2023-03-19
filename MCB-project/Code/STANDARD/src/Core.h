#pragma once

// General library includes
#include <cmath>
#include <iostream>
#include <string>

// Drivers
#include "drivers_singleton.hpp"
#include "drivers_singleton.hpp"
#include "drivers_singleton.hpp"

// Taproot
#include "tap/architecture/periodic_timer.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/board/board.hpp"

// Global variables
static tap::algorithms::SmoothPidConfig pid_conf = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
static tap::algorithms::SmoothPid default_pid = tap::algorithms::SmoothPid(pid_conf);
static tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
static tap::arch::PeriodicMilliTimer sendImuTimeout(2);
static src::Drivers *drivers = src::DoNotUse_getDrivers();
static tap::motor::DjiMotor motor_1 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "ID1", 0, 0);
static tap::motor::DjiMotor motor_2 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "PURDON'T!", 0, 0);
static tap::motor::DjiMotor motor_3 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, false, "Put the possum in his room", 0, 0);
static tap::motor::DjiMotor motor_4 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "Call 858-267-8107 for a good time!", 0, 0);

// Global functions
static void SendMotorData(int speed, tap::motor::DjiMotor* motor, tap::algorithms::SmoothPid pidController = default_pid) {
    pidController.runControllerDerivateError(speed - motor->getShaftRPM(), 1);
    motor->setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
    drivers->djiMotorTxHandler.encodeAndSendCanData();
}

static int GetSwitchState(const char switch_name) {
    tap::communication::serial::Remote::SwitchState state;
    if(switch_name == 'L') {
        state = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
    } else if(switch_name == 'R') {
        state = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
    }else{
        return -1;
    }
    switch(state) {
        case tap::communication::serial::Remote::SwitchState::DOWN:
            return 0;
        case tap::communication::serial::Remote::SwitchState::MID:
            return 1;
        case tap::communication::serial::Remote::SwitchState::UP:
            return 2;
        default:
            return -1;
    }
}

static float GetStickData(const char* stick_name) {
    tap::communication::serial::Remote::Channel channel;
    if(stick_name[0] == 'L') {
        if (stick_name[1] == 'V') {
            channel = tap::communication::serial::Remote::Channel::LEFT_VERTICAL;
        } else {
            channel = tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL;
        }
    } else if(stick_name[0] == 'R') {
        if (stick_name[1] == 'V') {
            channel = tap::communication::serial::Remote::Channel::RIGHT_VERTICAL;
        } else {
            channel = tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL;
        }
    }
    return drivers->remote.getChannel(channel);
}

static void InitializeCore() {
    Board::initialize();
    drivers->can.initialize();
    //drivers->bmi088.initialize();
}
    
// variable macros
#define PID_CONFIG pid_conf
#define DT_PID default_pid
#define DRIVERS drivers
#define PI 3.14159f
#define MOTOR_TIMER sendMotorTimeout
#define IMU_TIMER sendImuTimeout
#define MOTOR_1 motor_1
#define MOTOR_2 motor_2
#define MOTOR_3 motor_3
#define MOTOR_4 motor_4

// function macros
#define SPIN_MOTOR(...) SendMotorData(__VA_ARGS__)
#define SWITCH_STATE(...) GetSwitchState(__VA_ARGS__)
#define STICK_DATA(...) GetStickData(__VA_ARGS__)
#define INITIALIZE() InitializeCore()
#define DELAY(...) modm::delay_us(__VA_ARGS__)