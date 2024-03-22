/*
#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"

#define RX_BUFFER_LEN 128

src::Drivers *drivers;
static tap::arch::PeriodicMicroTimer RunTimer(10); //Don't ask me why. This only works as a global. #Certified Taproot Moment

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    uint8_t readBuff[RX_BUFFER_LEN];
    size_t read = 0;
    const char* msg = "hi\n\0";
    readBuff = (uint8_t)msg;

    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);
    
    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();
    robotController->initialize();
    while(1) {
        read = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart1,
            readBuff,
            RX_BUFFER_LEN);
        
        drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart1
                            readBuff, RX_BUFFER_LEN);
        
        if(read > 0){
            if(RunTimer.execute()) { //Calling this function every 10 us at max
                robotController->update();
            }
        }
    }
}
*/
/*
#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"
#define RX_BUFFER_LEN 128

uint8_t readBuff[RX_BUFFER_LEN];
size_t readBuffNumBytes = 0;
size_t read = 0;

src::Drivers *drivers;

int processRx(src::Drivers *drivers, char* arr) {
    read = drivers->uart.read(
        tap::communication::serial::Uart::UartPort::Uart1,
        &(readBuff[readBuffNumBytes]),
        RX_BUFFER_LEN - readBuffNumBytes
    );

    // char *arr;// = new char[4];
    // char *arr = new char[read];

    if(read>0){
        // arr = new char[read];
        for (size_t i = 0; i < read; i++)
        {
            arr[i] = readBuff[readBuffNumBytes+i];
        }
        readBuffNumBytes += read;
        
    }else{
        return -1;
    }
    if(readBuffNumBytes>=RX_BUFFER_LEN){
        readBuffNumBytes=0;
    }
    // return arr;
    return 0;
}

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();


    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);

    Board::initialize();

    // initializeIo(drivers);
    // src::control::initSubsystemCommands(drivers);
    
    drivers->uart.init< tap::communication::serial::Uart::UartPort::Uart1,115200>();

    drivers->leds.init();

    uint8_t recived_msg[RX_BUFFER_LEN];
    uint8_t* recived_ptr = recived_msg;
    recived_msg[0]=65;//A
    recived_msg[1]=65;
    recived_msg[2]=65;
    for (size_t i = 0; i < RX_BUFFER_LEN; i++)
        recived_msg[i]=65+i;
    // memset(recived_msg,65,RX_BUFFER_LEN);

    int count = 0;
    char msg[28];//="haaaa\n";
    while (1)
    {
        drivers->leds.set(tap::gpio::Leds::LedPin::Green, true);
        drivers->leds.set(tap::gpio::Leds::LedPin::Red, false);
        modm::delay_us(10000);
        // msg = "haaaa\n";
        
        // int result = processRx(drivers,recived_msg);

        // uint8_t rez;
        // int len = drivers->uart.read(tap::communication::serial::Uart::Uart1, &rez);
        // len += drivers->uart.read(tap::communication::serial::Uart::Uart1, &rez);
        // len += drivers->uart.read(tap::communication::serial::Uart::Uart1, &rez);
        // len += drivers->uart.read(tap::communication::serial::Uart::Uart1, &rez);
        // len += drivers->uart.read(tap::communication::serial::Uart::Uart1, &rez);

        int len = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart1,
            recived_ptr,
            // RX_BUFFER_LEN
            5
        );

        // drivers->uart.discardReceiveBuffer(tap::communication::serial::Uart::Uart1);

        // if(msg=="haaaa\n"){
        // if(result==-1){}
        drivers->leds.set(tap::gpio::Leds::LedPin::Blue, len);
        if(len==0){
            // msg = og;// "mcb reply but no msg\n";
            // msg = "mcb reply but no msg\n";
            // strcpy(msg, "mcb reply but no msg\n");

            // sprintf(msg, "mcb reply but no msg: %d\n", ++count%10);

            continue; //dont del
        }else{
            drivers->leds.set(tap::gpio::Leds::LedPin::Red, false);
        }
        drivers->leds.set(tap::gpio::Leds::LedPin::Red, true);
        drivers->leds.set(tap::gpio::Leds::LedPin::Green, false);
        sprintf(msg, "mcb reply but no msg: %3d\n", len);

        // drivers->leds.set(tap::gpio::Leds::LedPin::Green, false);
        char *ans = reinterpret_cast<char *>(recived_msg);
        // if(strcmp(ans,"bc\n\0") == 0)
        //     drivers->leds.set(tap::gpio::Leds::Green, true);
        // else{
        //     drivers->leds.set(tap::gpio::Leds::Green,false);
        // }


        read = 26;
        // msg[strlen(msg)-1] = '\n';
        msg[27-1] = '\n';

        recived_msg[4-1] = '\n';
        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            // reinterpret_cast<uint8_t*>(ans),
            reinterpret_cast<uint8_t*>(msg),
            // recived_msg,
            // read
            27
            );
        while(!drivers->uart.isWriteFinished(tap::communication::serial::Uart::Uart1));
    }
    return 0;
}
*/

#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"
#define RX_BUFFER_LEN 128
uint8_t readBuff[RX_BUFFER_LEN];
size_t readBuffNumBytes = 0;
size_t read = 0;

char* processRx(src::Drivers *drivers) {
    read = drivers->uart.read(
        tap::communication::serial::Uart::UartPort::Uart1,
        &(readBuff[readBuffNumBytes]),
        RX_BUFFER_LEN - readBuffNumBytes
    );

    char *arr;// = new char[4];
    // char *arr = new char[read];

    if(read>0){
        arr = new char[read];
        for (size_t i = 0; i < read; i++)
        {
            arr[i] = readBuff[readBuffNumBytes+i];
        }
        readBuffNumBytes += read;
        
    }else{
        return NULL;
    }
    if(readBuffNumBytes>=RX_BUFFER_LEN){
        readBuffNumBytes=0;
    }
    return arr;
}

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    // ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    // ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    // ThornBots::RobotController *robotController = new ThornBots::RobotController(drivers, driveTrainController, turretController);
    Board::initialize();
    // initializeIo(drivers);
    // src::control::initSubsystemCommands(drivers);
    
    drivers->uart.init< tap::communication::serial::Uart::UartPort::Uart1,115200>();

    while (1)
    {
        modm::delay_us(1000);
        char *msg = processRx(drivers);

        // char msg[8] = "hello\n\0"; //to just send msg
        if(msg==NULL)continue; //dont del

        char *ans = reinterpret_cast<char *>(msg);
        if(strcmp(ans,"bc\n\0") == 0)
            drivers->leds.set(tap::gpio::Leds::Green, true);

        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            reinterpret_cast<uint8_t*>(msg),
            read);
    }
    return 0;
}