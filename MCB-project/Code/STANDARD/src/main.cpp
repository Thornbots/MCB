#include <string>
#include "RobotController.h"
#include "TurretController.h"
#include "DriveTrainController.h"
#include "drivers_singleton.hpp"

#define RECEIVE_BUFFER_LEN 128
#define SEND_BUFFER_LEN 29

uint8_t receive_buf[RECEIVE_BUFFER_LEN];//[sizeof(float)*2];
char send_buf[SEND_BUFFER_LEN];

struct data_frame{
    float theta; // rad
    float omega; // rad
    float confidence; // 0.0 to 1.0
    float panel_dist; // meters
};

const size_t msg_len = sizeof(struct data_frame);
int read_from_uart(src::Drivers *drivers, uint8_t *msg) {
    int read_len = drivers->uart.read(
        tap::communication::serial::Uart::UartPort::Uart1,
        msg,
        msg_len
    );

    if(read_len==0)
        return -1;
    else
        return read_len;
}


void write_to_uart(src::Drivers *drivers, char *send_buf){
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart1,
        reinterpret_cast<uint8_t*>(send_buf),
        SEND_BUFFER_LEN);

    while(!drivers->uart.isWriteFinished(tap::communication::serial::Uart::Uart1));
}

int main()
{
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();
    
    drivers->uart.init< tap::communication::serial::Uart::UartPort::Uart1,115200>();

    while (1)
    {
        //============================== receive ==============================  
        modm::delay_us(1000); // sleep 1 sec

        // int read_len = drivers->uart.read(
        //     tap::communication::serial::Uart::UartPort::Uart1,
        //     receive_buf,
        //     msg_size
        // );
        int read_len = read_from_uart(drivers, receive_buf);
        struct data_frame* msg = (struct data_frame*) receive_buf;

        if(read_len==-1)continue; //dont del

        //============================== send==============================  
        // char *ans = reinterpret_cast<char *>(receive_buf);
        // int val = atoi(&ans[4]);
        // val++;

        double val1 = msg->confidence;
        double val2 = msg->omega;
        double val3 = msg->panel_dist;
        double val4 = msg->theta;

        memset(send_buf, 0, SEND_BUFFER_LEN);
        
        // sprintf(send_buf,"echo: %.2f | %.2f | %.2f | %.2f\n", val1, val2, val3, val4);
        sprintf(send_buf,"echo: %d | %d | %d | %d\n", (int) val1, (int) val2, (int) val3, (int) val4);
        sprintf(send_buf,"echo: %d\n", (int) val1);

        write_to_uart(drivers, send_buf);

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