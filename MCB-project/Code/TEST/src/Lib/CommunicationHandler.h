#include "Core.h"

namespace ThornBots {
    class CommunicationHandler {
        public:
        CommunicationHandler();
        ~CommunicationHandler() = default;

        // Initializes remote
        bool Initialize();                              

        // Reads the remote, should be run every cycle
        void Update();                                  

        // Returns state of specified switch 
        // Parameter should be either be 'R' or 'L'
        uint8_t GetSwitchState(const char switchID);    

        // Returns value of specified stick
        // Parameter should be a character array where the 0th item is either 'R' or 'L' and the 1st item is either 'H' or 'V'
        // e.g. {'R', 'H'} for right stick, horizontal axis
        float GetStickValue(const char* stickID);       

        // Returns value of specified wheel
        float GetWheelValue();                    

        // Returns whether or not the CommunicationHandler is initialized      
        bool GetIsInitialized();       

        // Returns a list of the keys being pressed
        // Refer to the intToKey dictionary at the bottom of this header for available keys
        // ^ is Ctrl    * is Shift       
        char* GetKeysPressed();              

        // Returns true if the RMB was clicked           
        bool GetRightMouseClicked();              

        // Returns true if the LMB was clicked      
        bool GetLeftMouseClicked();      

        // Returns an integer pair of the mouse's coordinates               
        std::pair<int, int> GetMouseCoords();

        // Returns a character array of Uart's output 
        char* GetUartOutput();                      

        // Sends messages to uart   
        void SendUart(const char* message);             

        private:
        char* m_UartOutput;
        size_t readBuffNumBytes = 0;
        bool isInitialized = false;
        std::map<int8_t, char> intToKey = {
            {0, 'W'},
            {1, 'S'},
            {2, 'A'},
            {3, 'D'},
            {4, '*'},
            {5, '^'},
            {6, 'Q'},
            {7, 'E'},
            {8, 'R'},
            {9, 'F'},
            {10, 'G'},
            {11, 'Z'},
            {12, 'X'},
            {13, 'C'},
            {14, 'V'},
            {15, 'B'}
        };
    };
};