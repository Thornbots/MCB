// General library includes
#include <cmath>
#include <iostream>
#include <string>

// Drivers
#include "drivers_singleton.hpp"

// Taproot
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

// Our .h's
#include "DriveTrainController.h"
#include "TurretController.h"
#include "ControlsHandler.h"

tap::arch::PeriodicMilliTimer sendDrivetrainTimeout(2);
tap::arch::PeriodicMilliTimer sendTurretTimeout(2);
tap::arch::PeriodicMilliTimer updateIMUTimeout(2);
std::string controlString = "";  // Will be the last two chars in the "WASDstring" string. (What we're actually going to be looking at)
src::Drivers *drivers;

bool KeyboardAndMouseEnabled = false;
bool doBeyblading = false;
double right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz = 0.0;
int16_t wheel_value = 0;
double angleOffset = 0.0;
bool isRightStickMid = false;
bool isLeftStickUp = false;
bool isLeftStickDown = false;
bool turretIndependent = false;
int rightSwitchValue = 1;
int leftSwitchValue = 0;

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);
    drivers->bmi088.requestRecalibration();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::ControlsHandler *controlsHandler = new ThornBots::ControlsHandler(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);

    while (1) {
        
        modm::delay_us(10);
        drivers->canRxHandler.pollCanData();
        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.

        if (updateIMUTimeout.execute())
        {
            drivers->bmi088.periodicIMUUpdate();
            angleOffset =
                drivers->bmi088
                    .getYaw();  // TODO: Make this calculate the AngleOffset and not the raw angle.
        }                       // Stop reading from the IMU

        if (drivers->remote.isConnected()) {  // If the remote is On and connected do the following

            controlsHandler->main();

            // Get Current state of Left Switch on the remote and set the appropriate variables
            auto leftSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
            switch (leftSwitchState) {
                case tap::communication::serial::Remote::SwitchState::UP:
                    isLeftStickDown = false;
                    isLeftStickUp = true;
                    doBeyblading = true;
                    turretIndependent = false;
                    leftSwitchValue = 2;
                    break;

                case tap::communication::serial::Remote::SwitchState::MID:
                    isLeftStickUp = false;
                    isLeftStickDown = false;
                    doBeyblading = false;
                    turretIndependent = true;
                    leftSwitchValue = 1;
                    break;

                case tap::communication::serial::Remote::SwitchState::DOWN:
                    isLeftStickDown = true;
                    isLeftStickUp = false;
                    doBeyblading = false;
                    turretIndependent = false;
                    leftSwitchValue = 0;
                    break;
            }

            // Get Current state of Right Switch on the remote and set the appropriate variables
            auto rightSwitchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
            switch (rightSwitchState) {
                case tap::communication::serial::Remote::SwitchState::MID:
                    // TODO: Make the drivebase align with the turret.
                    // i.e., if the angle offset is negative, make it spin CW or vice versa
                    isRightStickMid = true;
                    rightSwitchValue = 1;
                    break;

                case tap::communication::serial::Remote::SwitchState::UP:
                    // Nothing as of now
                    isRightStickMid = false;
                    rightSwitchValue = 2;
                    break;

                case tap::communication::serial::Remote::SwitchState::DOWN:
                    turretController->reZero();
                    isRightStickMid = false;
                    rightSwitchValue = 0;
                    // TODO: Lock the turret to the front of the drivetrain.
                    // i.e., set the desired angle to be 0 (the center of the robot)
                    break;
                }

                // Get Current state of the Right Stick on the remote and set the appropriate
                right_stick_vert = drivers->remote.getChannel(
                    tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
                right_stick_horz = drivers->remote.getChannel(
                    tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

                // Get Current state of the Left Stick on the remote and set the appropriate
                left_stick_vert = drivers->remote.getChannel(
                    tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
                left_stick_horz = drivers->remote.getChannel(
                    tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);

                // Get Current state of the wheel on the remote and set the appropriate
                wheel_value = drivers->remote.getWheel();

                // Call the setMotorValues and setMotorSpeeds function in the DriveTrainController class
                driveTrainController->setMotorValues(
                    KeyboardAndMouseEnabled,
                    doBeyblading,
                    right_stick_vert,
                    right_stick_horz,
                    left_stick_vert,
                    left_stick_horz,
                    controlString,
                    turretController->getYawEncoderAngle(),
                    isRightStickMid,
                    rightSwitchValue,
                    leftSwitchValue);
                driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());

                // Call the setMotorValues and setMotor Speeds function in the TurretController class
                turretController->setMotorValues(
                    KeyboardAndMouseEnabled,
                    doBeyblading,
                    angleOffset,
                    -right_stick_vert,
                    right_stick_horz,
                    driveTrainController->motor_one.getShaftRPM(),
                    driveTrainController->motor_four.getShaftRPM(),
                    wheel_value,
                    isRightStickMid,
                    isLeftStickUp,
                    rightSwitchValue,
                    leftSwitchValue);
                turretController->setMotorSpeeds(sendTurretTimeout.execute());
        }
        else
        {  // Remote not connected, so have everything turn off (Saftey features!)
            driveTrainController->stopMotors(sendDrivetrainTimeout.execute());
            turretController->stopMotors(sendTurretTimeout.execute());
        }
    }
    return 0;
}