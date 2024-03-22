#include "RobotController.h"
#include "TurretController.h"
#include "ShooterController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"

src::Drivers *drivers;
static tap::arch::PeriodicMicroTimer RunTimer(10); //Don't ask me why. This only works as a global. #Certified Taproot Moment

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::ShooterController *shooterController = new ThornBots::ShooterController(drivers);

    ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController, shooterController);

    robotController->initialize();
    while(1) {
        if(RunTimer.execute()) { //Calling this function every 10 us at max
            robotController->update();
        }
    }
}