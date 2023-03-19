#include "DrivetrainController.h"
#include "TurretController.h"

ThornBots::DrivetrainController s_DrivetrainController;
ThornBots::TurretController s_TurretController;

void InitializeRobot() {
    INITIALIZE();
    DRIVERS->remote.initialize();
    s_DrivetrainController.Initialize();
    s_TurretController.Initialize();
}

void UpdateRobot() {
    s_DrivetrainController.UpdateDrivetrain();
    s_TurretController.UpdateTurretController();
    DRIVERS->canRxHandler.pollCanData();
}

void EmergencyStop() {
    s_DrivetrainController.StopMotors();
    s_TurretController.StopMotors();
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