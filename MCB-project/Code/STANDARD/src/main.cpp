#include "DrivetrainController.h"

ThornBots::DrivetrainController s_DrivetrainController;

void InitializeRobot() {
    INITIALIZE();
    DRIVERS->remote.initialize();
    s_DrivetrainController.Initialize();
}

void UpdateRobot() {
    s_DrivetrainController.UpdateDrivetrain();
    DRIVERS->canRxHandler.pollCanData();
}

void EmergencyStop() {
    s_DrivetrainController.StopMotors();
}

int main() {
    InitializeRobot();
    while (1) {
        DELAY(10);
        DRIVERS->remote.read();
        if(DRIVERS->remote.isConnected()) {
            UpdateRobot();
        } else {
            EmergencyStop();
        }
    }
    return 0;
}