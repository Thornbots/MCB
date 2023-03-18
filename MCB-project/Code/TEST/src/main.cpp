#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "params.h"
#include "drivers_singleton.hpp"

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
int DESIRED_RPM = 0;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::arch::PeriodicMilliTimer periodicIMUUpdate(2);
tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(PID_CONFIG);

tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->bmi088.initialize(500, 0.5, 0.1);
    drivers->bmi088.requestRecalibration();
    motor.initialize();

    while (1) {
        if(periodicIMUUpdate.execute()) {
            drivers->bmi088.periodicIMUUpdate();
            DESIRED_RPM = drivers->bmi088.getYaw();
        }
        if (sendMotorTimeout.execute()) {
            pidController.runControllerDerivateError(DESIRED_RPM - motor.getShaftRPM(), 1);
            motor.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        drivers->canRxHandler.pollCanData();
        modm::delay_us(10);
    }
    return 0;
}

