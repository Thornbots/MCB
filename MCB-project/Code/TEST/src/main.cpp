#include "tap/board/board.hpp"
#include "Taproot/drivers_singleton.hpp"

static constexpr uint32_t LED_BLINK_PERIOD = 1000;

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();
    drivers->leds.init();

    bool aSet = true;

    while (1)
    {
        modm::delay_ms(LED_BLINK_PERIOD / 2);
        drivers->leds.set(tap::gpio::Leds::Red, !aSet);
        aSet = !aSet;
    }
    return 0;
}
