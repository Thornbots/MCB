#include "Core.h"
#include "CommunicationHandler.h"

namespace ThornBots {

    char* m_UartOutput;

    bool CommunicationHandler::Initialize() {
        drivers->remote.initialize();
        return true;
    }

    uint8_t CommunicationHandler::GetSwitchState(const char switchID) {
        tap::communication::serial::Remote::Switch::LEFT_SWITCH
        return drivers->remote.getSwitch();
    }

    float CommunicationHandler::GetStickValue(const char stickID) {
        return drivers->remote.getSwitch(stickID);
    }

};