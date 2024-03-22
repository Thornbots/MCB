#include <string>
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
    int prev = 0;

    drivers->leds.init();
    while (1)
    {
        modm::delay_us(1000);
        char *msg = processRx(drivers);
        // char* msg = "asdf\n";
        // read = 6;

        // char msg[8] = "hello\n\0"; //to just send msg
        if(msg==NULL)continue; //dont del

        char *ans = reinterpret_cast<char *>(msg);
        int val = atoi(ans);
        val++;
        sprintf(ans,"%d",val%=100);
        if(strcmp(ans,"bc\n\0") == 0)
            drivers->leds.set(tap::gpio::Leds::Green, true);
        drivers->leds.set(tap::gpio::Leds::Green, prev>50);
        prev+=1;
        prev%=100;

        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            // reinterpret_cast<uint8_t*>(msg),
            reinterpret_cast<uint8_t*>(ans),
            read);
    }
    return 0;
}
/* remeber to do sudo pip install pyserial
import time
import serial

#ser = serial.Serial(port='/dev/ttyTHS0', baudrate=115200, timeout=1)

#ser = serial.Serial()
#ser.baudrate = 115200
#ser.port = '/dev/ttyTHS0'
#ser.bytesize=serial.EIGHTBITS
#ser.parity=serial.PARITY_NONE
#ser.STOPBITS_ONE
# try:
#     #ser.open()
#     print ("Port has been opened")
# except Exception:
#     print ("error open serial port: ")
#     exit()

ser = serial.Serial(
    #port="/dev/ttyTHS0",
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
robomaster@ubuntu:~/Documents$ cat uart2.py 
import time
import serial

#ser = serial.Serial(port='/dev/ttyTHS0', baudrate=115200, timeout=1)

#ser = serial.Serial()
#ser.baudrate = 115200
#ser.port = '/dev/ttyTHS0'
#ser.bytesize=serial.EIGHTBITS
#ser.parity=serial.PARITY_NONE
#ser.STOPBITS_ONE
# try:
#     #ser.open()
#     print ("Port has been opened")
# except Exception:
#     print ("error open serial port: ")
#     exit()

ser = serial.Serial(
    #port="/dev/ttyTHS0",
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=5,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    writeTimeout=2
)
if(not ser.isOpen()):
    print("not open")
    ser.open()

past = 1
ser.write(f'{past}\n'.encode())
while 1:
    try:

        data = ser.read()
        if not(data==b'\x00' or data == '\n'):
            #ser.write("asdf\n\0".encode('utf-8'))
            ser.write(f'{past}\n'.encode())
            print(f"sent: {past}")
            past += 1

            #past = ser.readline().decode('uft-8')
            #print("recived:",past)
            print("recive:",data)#, "decode:", data.decode("utf-8"))
            past = int(data.decode("utf-8"))

        time.sleep(0.1)
    except Exception as e:
        print(e)
        ser.close()
        break
*/