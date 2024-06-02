#pragma once
#include "drivers_singleton.hpp"
#include "tap/communication/serial/dji_serial.hpp"

struct cord_msg;

namespace ThornBots {
class JetsonCommunication: public tap::communication::serial::DJISerial{
    private:
        bool hasBeenRead = false;
        src::Drivers* drivers;

    public:
        using tap::communication::serial::DJISerial::DJISerial;

        struct cord_msg
        {
            float theta; // rad
            float omega; // rad
            float confidence; // 0.0 to 1.0
            float panel_dist; // meters
        };


        /*
        run this to read from the uart queue buffer to get the coordinates of the panal
        */
        void messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
        {
            message = (cord_msg*)completeMessage.data;
            // constexpr int msg_len = 60;
            // char buf[msg_len];
            // // sprintf(buf, "rez: %04x\n", completeMessage.data[0]);
            
            // sprintf(buf,"theta: %1.1f, omega: %1.1f, confidence: %1.1f, dist: %1.1f\n",
            // message->theta,message->omega,message->confidence,message->panel_dist);

            // uint8_t* msg = reinterpret_cast<uint8_t*>(buf);
            // drivers->uart.write(tap::communication::serial::Uart::Uart1, msg,msg_len);
            drivers->leds.set(tap::gpio::Leds::Blue, true);
        };

        cord_msg* getMsg(){return message;};

        // void sendToJetson(uint8_t){


        // }

    private:
        cord_msg *message;
    };

}