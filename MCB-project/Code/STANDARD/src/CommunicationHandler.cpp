#include "Core.h"
#include "CommunicationHandler.h"

#define RX_BUFFER_LEN 128

namespace ThornBots {

    char* m_UartOutput;

    CommunicationHandler::CommunicationHandler() {
        //TODO
    }

    bool CommunicationHandler::Initialize() {
        drivers->remote.initialize();
        return true;
    }

    void CommunicationHandler::Update() {
        //TODO
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
        
     }

    float CommunicationHandler::GetWheelValue() {
        //TODO
    }

    bool CommunicationHandler::GetIsInitialized() {
        //TODO
    }

    char* CommunicationHandler::GetKeysPressed() {
        //TODO
    }

    bool CommunicationHandler::GetRightMouseClicked() {
        //TODO
    }

    bool CommunicationHandler::GetLeftMouseClicked() {
        //TODO
    }

    std::pair<int, int> CommunicationHandler::GetMouseCoords() {
        //TODO
    }

    char* CommunicationHandler::GetUartOutput() {
        size_t readBuffNumBytes = 0;
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