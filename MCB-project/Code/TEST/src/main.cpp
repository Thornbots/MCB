#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"

src::Drivers *drivers;

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);

    robotController->inialize();
    while(1) {
        robotController->update();
    }
}