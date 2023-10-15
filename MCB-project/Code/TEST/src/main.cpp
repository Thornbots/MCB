#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "Lib/HardwareHandler.h"
#include "Lib/CommunicationHandler.h"
#include "Lib/RefereeSystem.h"
#include "Taproot/drivers_singleton.hpp"
#include "Lib/DriveTrainController.h"
#include <memory>

/*
static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM = 3000;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::algorithms::SmoothPidConfig pidConf = tap::algorithms::SmoothPidConfig(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pidConf);
tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();

    drivers->can.initialize();

    motor.initialize();

    while (1)
    {
        if (sendMotorTimeout.execute())
        {
            pidController.runControllerDerivateError(DESIRED_RPM - motor.getShaftRPM(), 1);
            motor.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        drivers->canRxHandler.pollCanData();
        modm::delay_us(10);
    }
    return 0;
}
*/

static constexpr int MAX_DESIRED_RPM = 100;
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::algorithms::SmoothPidConfig pidConf = tap::algorithms::SmoothPidConfig(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pidConf);

int main(){
    
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
        
    using namespace ThornBots;

    HardwareHandler hh = HardwareHandler(drivers);
    CommunicationHandler ch = CommunicationHandler(drivers);
    RefereeSystem rs = RefereeSystem(drivers);
    
    std::shared_ptr<CommunicationHandler> m_CommunicationHandler = std::make_shared<CommunicationHandler>(ch);
    std::shared_ptr<RefereeSystem> m_RefereeSystem = std::make_shared<RefereeSystem>(rs);
    std::shared_ptr<HardwareHandler> m_HardwareHandler = std::make_shared<HardwareHandler>(hh);

    DriveTrainController dtc = DriveTrainController(m_CommunicationHandler, m_HardwareHandler, m_RefereeSystem);

    hh.Initialize();
    ch.Initialize();
    rs.Initialize();
    dtc.Initialize();

    while (1) {
        ch.Update();
        char values[] = "RV";

        int maxRPM = MAX_DESIRED_RPM / 2;
        int rpm = ch.GetStickValue("RH") != 0 ? maxRPM : 0;
        pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_FRONT_LEFT), 1);
        hh.SetMotorPowerOutput(Motor::MOTOR_FRONT_LEFT, static_cast<int32_t>(pidController.getOutput()));

        rpm = ch.GetStickValue(values) != 0 ? maxRPM : 0;
        pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_FRONT_RIGHT), 1);
        hh.SetMotorPowerOutput(Motor::MOTOR_FRONT_RIGHT, static_cast<int32_t>(pidController.getOutput()));

        rpm = ch.GetStickValue("LV") != 0 ? maxRPM : 0;
        pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_LEFT), 1);
        hh.SetMotorPowerOutput(Motor::MOTOR_BACK_LEFT, static_cast<int32_t>(pidController.getOutput()));

        rpm = ch.GetStickValue("LH") != 0 ? maxRPM : 0;
        pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_RIGHT), 1);
        hh.SetMotorPowerOutput(Motor::MOTOR_BACK_RIGHT, static_cast<int32_t>(pidController.getOutput()));


        // rpm = rs.GetRobotHP() != 0 ? MAX_DESIRED_RPM : 0;
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_LEFT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_BACK_LEFT, static_cast<int32_t>(pidController.getOutput()));

        // rpm = rs.IsBlueTeam() != 0 ? MAX_DESIRED_RPM : 0;
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_RIGHT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_BACK_RIGHT, static_cast<int32_t>(pidController.getOutput()));
        
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_FRONT_LEFT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_FRONT_LEFT, static_cast<int32_t>(pidController.getOutput()));
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_FRONT_RIGHT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_FRONT_RIGHT, static_cast<int32_t>(pidController.getOutput()));
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_LEFT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_BACK_LEFT, static_cast<int32_t>(pidController.getOutput()));
        // pidController.runControllerDerivateError(rpm - hh.GetMotorShaftRPM(Motor::MOTOR_BACK_RIGHT), 1);
        // hh.SetMotorPowerOutput(Motor::MOTOR_BACK_RIGHT, static_cast<int32_t>(pidController.getOutput()));
        hh.SendCanData();
        // meem
    }
}