#include "drivers_singleton.hpp"

struct cord_msg;

class JetsonCommunication
{
private:
    bool hasBeenRead = false;
    src::Drivers* drivers;

public:
    struct cord_msg
    {
        float yaw;
        float pitch;
        bool shoot;
    };
    JetsonCommunication(src::Drivers* drivers)
        : drivers(drivers),
          message({0, 0, 0}),
          hasBeenRead(false)
    {
        drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();
    };

    ~JetsonCommunication() = default;

    /*
    run this to read from the uart queue buffer to get the coordinates of the panal
    */
    int update()
    {
        int read_len = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart1,
            (uint8_t*)&this->message,
            sizeof(cord_msg));

        return read_len == 0 ? -1 : read_len;
    }
    cord_msg* getMsg(){return &message};

private:
    cord_msg message;
};