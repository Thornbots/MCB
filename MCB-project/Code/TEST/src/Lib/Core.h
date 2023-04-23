#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "../Taproot/drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include <cmath>
#include <iostream>
#include <string>

static src::Drivers *drivers = src::DoNotUse_getDrivers();

static void InitializeCore() {
    Board::initialize();
}