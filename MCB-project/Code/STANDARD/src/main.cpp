/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of taproot-template-project.
 *
 * taproot-template-project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * taproot-template-project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with taproot-template-project.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
// static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
// static void updateIo(src::Drivers *drivers);

#define RX_BUFFER_LEN 128
uint8_t readBuff[RX_BUFFER_LEN];
size_t readBuffNumBytes = 0;
size_t read = 0;

char *processRx(src::Drivers *drivers)
{
    read = drivers->uart.read(
        tap::communication::serial::Uart::UartPort::Uart1,
        &(readBuff[readBuffNumBytes]),
        RX_BUFFER_LEN - readBuffNumBytes);

    char *arr;  // = new char[4];
    // char *arr = new char[read];

    if (read > 0)
    {
        arr = new char[read];
        for (size_t i = 0; i < read; i++)
        {
            arr[i] = readBuff[readBuffNumBytes + i];
        }
        readBuffNumBytes += read;
    }
    else
    {
        return NULL;
    }
    if (readBuffNumBytes >= RX_BUFFER_LEN)
    {
        readBuffNumBytes = 0;
    }
    return arr;
}

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();
    // initializeIo(drivers);
    // wsrc::control::initSubsystemCommands(drivers);

    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();

    while (1)
    {
        modm::delay_us(1000);
        char *msg = processRx(drivers);

        // char msg[8] = "hello\n\0"; //to just send msg
        if (msg == NULL) continue;  // dont del

        char *ans = reinterpret_cast<char *>(msg);
        if (strcmp(ans, "bc\n\0") == 0) drivers->leds.set(tap::gpio::Leds::Green, true);

        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            reinterpret_cast<uint8_t *>(msg),
            read);
    }
    return 0;
}