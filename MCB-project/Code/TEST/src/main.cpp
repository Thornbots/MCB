#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"

#include "drivers_singleton.hpp"

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM = 3000;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::algorithms::SmoothPid pidController(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");

int main() {
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
            drivers->djiMotorTxHandler.processCanSendData();
        }

        drivers->canRxHandler.pollCanData();
        modm::delay_us(10);
    }
    return 0;
}

