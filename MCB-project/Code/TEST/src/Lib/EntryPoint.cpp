#include "EntryPoint.h"

namespace ThornBots{
    bool EntryPoint::Initialize(){
        m_RobotController.Initialize();
    }
    void EntryPoint::Update(uint32_t cycleTimeInUS = 10, bool runRobot = true){
        if(runRobot){
            
        }else{
            m_RobotController.EmergencyStop();
        }
    }
    void EntryPoint::Destroy(){

    }
    int main(){
        return 1;
    }
}