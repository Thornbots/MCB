#include "RobotController.h"
#pragma once

namespace ThornBots{
    RobotController::RobotController(){}
    bool RobotController::Initialize(){
        s_DriveTrainController->Initialize();
        s_TurretController->Initialize();
    }
    
    void RobotController::Update(){
        s_DriveTrainController->Update();
        s_TurretController->Update();
    }

    void RobotController::EmergencyStop(){
        s_DriveTrainController->EmergencyStop();
        s_TurretController->EmergencyStop();
    }
}