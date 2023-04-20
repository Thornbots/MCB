#include "Core.h"

namespace ThornBots {
    class CommunicationHandler {
        public:
        CommunicationHandler();
        ~CommunicationHandler();
        bool Initialize();                              // WHAT THE HELL AM I INITIALIZING (controllers, keyboard, uart)
        void Update();                                  // Assuming this updates uart values and stuff? 
        uint8_t GetSwitchState(const char switchID);   // Returns state of specified switch
        float GetStickValue(const char stickID);        // Returns value of specified stick
        float GetWheelValue();                          // Returns value of specified wheel
        bool GetIsInitialized();                        // Returns whether or not the CommunicationHandler is initialized
        char* GetKeysPressed();                         // Returns a list of the keys being pressed
        bool GetRightMouseClicked();                    // Returns true if the RMB was clicked
        bool GetLeftMouseClicked();                     // Returns true if the LMB was clicked
        std::pair<int, int> GetMouseCoords();           // Returns an integer pair of the mouse's coordinates
        char* GetUartOutput();                          // wat
        void SendUart(const char* message);             // Sends messages to uart

        private:
        char* m_UartOutput;
    };
};