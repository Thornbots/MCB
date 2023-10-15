#include "EntryPoint.h"
#include "RobotController.h"
#include <stdint.h>
#include <iostream>

namespace ThornBots{
    
    void EntryPoint::Update(uint32_t cycleTimeInUS = 10, bool runRobot = true){
        if(runRobot){
            m_RobotController.Update();
        } else{
            m_RobotController.EmergencyStop();
        }
    }

    void EntryPoint::Destroy(){

    }

    static int main(){
        RobotController rc = RobotController();
        rc.Initialize();
         {
            using namespace std;
            cout << "Yee" << endl;
         }
        

        while(1){
            rc.Update();
        } 
        // rc.EmergencyStop();
        return 1;

    }
}