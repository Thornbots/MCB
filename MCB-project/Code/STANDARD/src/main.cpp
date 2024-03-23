#include <string>
#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"
#define RX_BUFFER_LEN 128
uint8_t readBuff[RX_BUFFER_LEN];
const size_t msg_size = sizeof(float)*2;

// int getCords(src::Drivers *drivers, float *msg) {
int getCords(src::Drivers *drivers, uint8_t *msg) {
    int read_len = drivers->uart.read(
        tap::communication::serial::Uart::UartPort::Uart1,
        // (uint8_t*)msg,
        msg,
        msg_size
    );

    if(read_len==0)
        return -1;
    else
        return read_len;

    // char *arr;// = new char[4];
    // char *arr = new char[read];

    // if(read>0){
    //     arr = new char[read];
    //     for (size_t i = 0; i < read; i++)
    //     {
    //         arr[i] = readBuff[readBuffNumBytes+i];
    //     }
    //     readBuffNumBytes += read;
        
    // }else{
    //     return -1;
    // }
    // if(readBuffNumBytes>=RX_BUFFER_LEN){
    //     readBuffNumBytes=0;
    // }
    // return 0;
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
        //============================== receive ==============================  
        modm::delay_us(1000);
        float cords[2];
        uint8_t cords_bytes[sizeof(float)*2];
        int read_len = getCords(drivers,cords_bytes);
        // char* msg = "asdf\n";
        // read = 6;

        // char msg[8] = "hello\n\0"; //to just send msg
        if(read_len==-1)continue; //dont del

        //============================== send==============================  

        // char *ans = reinterpret_cast<char *>(msg);
        // int val = atoi(ans);
        // val++;
        size_t buf_size = 62;
        char send_buf[buf_size];
        // sprintf(send_buf,"echo: %.001f, %.001f\n",cords[0], cords[1]);
        sprintf(send_buf,"echo: %#04x, %#04x, %#04x, %#04x, %#04x, %#04x, %#04x, %#04x\n",cords_bytes[0],
            cords_bytes[1],
            cords_bytes[2],
            cords_bytes[3],
            cords_bytes[4],
            cords_bytes[5],
            cords_bytes[6],
            cords_bytes[7]
        );

        // if(strcmp(ans,"bc\n\0") == 0)
        //     drivers->leds.set(tap::gpio::Leds::Green, true);
        // drivers->leds.set(tap::gpio::Leds::Green, prev>50);

        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            // reinterpret_cast<uint8_t*>(msg),
            reinterpret_cast<uint8_t*>(send_buf),
            //read);
            buf_size);
        while(!drivers->uart.isWriteFinished(tap::communication::serial::Uart::Uart1));
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