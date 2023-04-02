#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include <cmath>
#include <iostream>
#include <string>

src::Drivers *drivers = src::DoNotUse_getDrivers();

void initialize() {
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);

    //TODO: Include ThornBots classes and initialize them
}