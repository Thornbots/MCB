#include "RobotController.h"

namespace ThornBots{
    RobotController::RobotController(){
        src::Drivers *drivers = src::DoNotUse_getDrivers();

        // std::shared_ptr<CommunicationHandler> m_CommunicationHandler = std::make_shared<CommunicationHandler>(CommunicationHandler());
        // std::shared_ptr<RefereeSystem> m_RefereeSystem = std::make_shared<RefereeSystem>(RefereeSystem(drivers));
        // std::shared_ptr<HardwareHandler> m_HardwareHandler = std::make_shared<HardwareHandler>(HardwareHandler(drivers));

        // DriveTrainController dtc =  DriveTrainController(drivers, m_CommunicationHandler, m_HardwareHandler, m_RefereeSystem);
        // TurretController tc = TurretController(m_CommunicationHandler, m_HardwareHandler, m_RefereeSystem);

        // s_DriveTrainController = &dtc;
        // s_TurretController = &tc;
        


    }

    bool RobotController::Initialize(){
        s_DriveTrainController->Initialize();
        s_TurretController->Initialize();
    }
    
    void RobotController::Update() {
        s_TurretController->m_HardwareHandler->SetMotorPowerOutput(Motor::MOTOR_FRONT_LEFT, 1);
        // s_DriveTrainController->setMotorSpeeds()

        // s_DriveTrainController->Update();
        // s_TurretController->Update();
    
    }

    void RobotController::EmergencyStop(){
        s_DriveTrainController->EmergencyStop();
        s_TurretController->EmergencyStop();
    }
}