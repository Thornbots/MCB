#include "Core.h"
#include "CommunicationHandler.h"

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
        //TODO
    }

    void CommunicationHandler::SendUart(const char* message) {
        //TODO
    }

};