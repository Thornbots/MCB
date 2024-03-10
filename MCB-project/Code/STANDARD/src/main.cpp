#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"

#define RX_BUFFER_LEN 128

src::Drivers *drivers;
static tap::arch::PeriodicMicroTimer RunTimer(10); //Don't ask me why. This only works as a global. #Certified Taproot Moment

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    uint8_t readBuff[RX_BUFFER_LEN];
    size_t read = 0;

    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);
    
    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();
    robotController->initialize();
    while(1) {
        read = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart1,
            readBuff,
            RX_BUFFER_LEN);
        
        if(read > 0){
            if(RunTimer.execute()) { //Calling this function every 10 us at max
                robotController->update();
            }
        }
    }
}