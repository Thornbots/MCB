#pragma once
#include "../Taproot/drivers_singleton.hpp"
#include "tap/motor/dji_motor.hpp"


static src::Drivers *drivers = src::DoNotUse_getDrivers();

static void InitializeCore() {
    Board::initialize();
}