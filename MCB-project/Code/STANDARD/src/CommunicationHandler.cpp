#include "CommunicationHandler.h"

#define RX_BUFFER_LEN 128

namespace ThornBots {

    CommunicationHandler::CommunicationHandler() {
        // initialization code for uart
        drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart1, 115200>();
    }

    bool CommunicationHandler::Initialize() {
        drivers->remote.initialize();
        isInitialized = true;
        return isInitialized;
    }

    void CommunicationHandler::Update() {
        drivers->remote.read();
    }

    uint8_t CommunicationHandler::GetSwitchState(const char switchID) {
        switch (switchID)
        {
        case 'R':
            return (uint8_t) drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
        case 'L':
            return (uint8_t) drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
        default:
            return NULL;
        }
        
    }

    float CommunicationHandler::GetStickValue(const char* stickID) {
        tap::communication::serial::Remote::Channel;
        if (stickID[0] == 'R') {
            switch (stickID[1])
            {
            case 'H':
                drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            case 'V':
                drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            default:
                return NULL;
            }
        } else if (stickID[0] == 'L') {
            switch (stickID[1])
            {
            case 'H':
                drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            case 'V':
                drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            default:
                return NULL;
            }
        }
        return NULL;
     }

    float CommunicationHandler::GetWheelValue() {
        return drivers->remote.getWheel();
    }

    bool CommunicationHandler::GetIsInitialized() {
        return isInitialized;
    }

    char* CommunicationHandler::GetKeysPressed() {
        std::vector<char> charVector;

        for (int i = 0; i < intToKey.size(); i++) {
            if (drivers->remote.keyPressed(static_cast<tap::communication::serial::Remote::Key>(i))) {
                charVector.push_back(intToKey.at(i));
            };
        }

        char returnArray[charVector.size()];


        for (int i = 0; i < intToKey.size(); i++) {
            returnArray[i] = charVector.at(i);
        } 

        return returnArray;
    }

    bool CommunicationHandler::GetRightMouseClicked() {
        return drivers->remote.getMouseR();
    }

    bool CommunicationHandler::GetLeftMouseClicked() {
        return drivers->remote.getMouseL();
    }

    std::pair<int, int> CommunicationHandler::GetMouseCoords() {
        int16_t xCoord = drivers->remote.getMouseX();
        int16_t yCoord = drivers->remote.getMouseY();
        return std::pair<int, int>(xCoord, yCoord);
    }

    char* CommunicationHandler::GetUartOutput() {
        uint8_t readBuff[RX_BUFFER_LEN];
        size_t read = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart1,
            &(readBuff[readBuffNumBytes]),
            RX_BUFFER_LEN - readBuffNumBytes);

        char *arr;

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

    void CommunicationHandler::SendUart(const char* message) {
        drivers->uart.write(
            tap::communication::serial::Uart::UartPort::Uart1,
            reinterpret_cast<uint8_t *>(message),
            0);
    }
};