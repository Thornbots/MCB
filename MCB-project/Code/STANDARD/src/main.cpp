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
#include "RobotController.h"

tap::arch::PeriodicMilliTimer sendDrivetrainTimeoutTemp(2);
//tap::arch::PeriodicMilliTimer sendTurretTimeout(2);
tap::arch::PeriodicMilliTimer updateIMUTimeout(2);
src::Drivers *drivers;

//tobe deleted
double right_stick_vert = 0.0;
double right_stick_horz = 0.0;
double left_stick_vert = 0.0;
double left_stick_horz = 0.0;
float temp_yaw_angle = 0.0;
int rightSwitchValue = 0;
int leftSwitchValue = 0;

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);
    drivers->bmi088.requestRecalibration();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::RobotController *RobotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);

    while (1) {
        modm::delay_us(10);
        drivers->canRxHandler.pollCanData();
        drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.

        if (updateIMUTimeout.execute()) {
            drivers->bmi088.periodicIMUUpdate();
        } // Stop reading from the IMU

        if (drivers->remote.isConnected()) {  // If the remote is On and connected do the following
            RobotController->update();

        } else {  // Remote not connected, so have everything turn off (Saftey features!)
            RobotController->stopRobot();
        }

        drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes into can signal

    }
    return 0;
}