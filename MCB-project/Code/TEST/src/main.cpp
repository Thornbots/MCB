/*
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>

static constexpr tap::motor::MotorId MOTOR_ID7 = tap::motor::MOTOR7;
static constexpr tap::motor::MotorId MOTOR_ID5 = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId MOTOR_ID8 = tap::motor::MOTOR8;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2; // sets up the can bus

float LEFT_STICK_VERT = 0;   // left stick vertical stick ratio [-1,1]
float RIGHT_STICK_VERT = 0;  // right stick vertical stick ratio [-1,1]
int C_INDEXER_RPM = 0;            
int S_INDEXER_RPM = 0;
int C_FLYWHEEL_RPM = 0;            
int S_FLYWHEEL_RPM = 0;
int MAX_RPM = 8000;             
int MOTOR_INC = 1000;
bool LEFT_MID_INC = 1;
bool RIGHT_MID_INC = 1;
int TIME = 0;
int BLINK_PERIOD = 10000;
bool BLINKBOOL = true;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::algorithms::SmoothPid pidController(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::motor::DjiMotor indexer(::DoNotUse_getDrivers(), MOTOR_ID7, CAN_BUS, false, "indexer", 0, 0);
tap::motor::DjiMotor flywheel_L(::DoNotUse_getDrivers(), MOTOR_ID5, CAN_BUS, false, "left flywheel", 0, 0);
tap::motor::DjiMotor flywheel_R(::DoNotUse_getDrivers(), MOTOR_ID8, CAN_BUS, false, "right flywheel", 0, 0);

int main() {

    ::Drivers *drivers = ::DoNotUse_getDrivers();
    drivers->can.initialize();
    drivers->remote.initialize();
    Board::initialize();
    indexer.initialize();
    flywheel_L.initialize();
    flywheel_R.initialize();

    while (1) {  
		
        drivers->remote.read();
        TIME++;
        
        if(drivers->remote.isConnected()) {  //only runs motor if remote is connected
            
            //read remote sticks
            RIGHT_STICK_VERT = drivers->remote.getChannel(drivers->remote.Channel::RIGHT_VERTICAL);
            LEFT_STICK_VERT = drivers->remote.getChannel(drivers->remote.Channel::LEFT_VERTICAL);
        
            //process left switch
            if(drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::MID) { 
                LEFT_MID_INC = 1;
      	    }else if((drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::UP)) { 
                if((LEFT_MID_INC == 1)  && (S_INDEXER_RPM < MAX_RPM)){
                    S_INDEXER_RPM += MOTOR_INC;
                    LEFT_MID_INC = 0;
                }
            }else if(drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::DOWN) { 
                if((LEFT_MID_INC == 1) && (S_INDEXER_RPM > 0)){
                    S_INDEXER_RPM -= MOTOR_INC;
                    LEFT_MID_INC = 0;
                }
            }
            
            //process right switch
            if(drivers->remote.getSwitch(drivers->remote.Switch::RIGHT_SWITCH) == drivers->remote.SwitchState::MID) { 
                RIGHT_MID_INC = 1;
      	    }else if((drivers->remote.getSwitch(drivers->remote.Switch::RIGHT_SWITCH) == drivers->remote.SwitchState::UP)) { 
                if((RIGHT_MID_INC == 1)  && (S_FLYWHEEL_RPM < MAX_RPM)){
                    S_FLYWHEEL_RPM += MOTOR_INC;
                    RIGHT_MID_INC = 0;
                }
            }else if(drivers->remote.getSwitch(drivers->remote.Switch::RIGHT_SWITCH) == drivers->remote.SwitchState::DOWN) { 
                if((RIGHT_MID_INC == 1) && (S_FLYWHEEL_RPM > 0)){
                    S_FLYWHEEL_RPM -= MOTOR_INC;
                    RIGHT_MID_INC = 0;
                }
            }
         
            //set curr rpm to set rpm if sticks are moved
            if (LEFT_STICK_VERT != 0){
                C_INDEXER_RPM = S_INDEXER_RPM;
            }else{
                C_INDEXER_RPM = 0;
            }
            
            //set curr rpm to set rpm if sticks are moved
            if (RIGHT_STICK_VERT != 0){
                C_FLYWHEEL_RPM = S_FLYWHEEL_RPM;
            }else{
                C_FLYWHEEL_RPM = 0;
            }
         
            //run motors block
       	    if(sendMotorTimeout.execute()) {
       
                //indexer
                pidController.runControllerDerivateError(C_INDEXER_RPM - indexer.getShaftRPM(), 1);
                indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                drivers->djiMotorTxHandler.encodeAndSendCanData();
                
                //flywheels
                pidController.runControllerDerivateError(C_FLYWHEEL_RPM - flywheel_L.getShaftRPM(), 1);
                flywheel_L.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                drivers->djiMotorTxHandler.encodeAndSendCanData();
                pidController.runControllerDerivateError((-1 * C_FLYWHEEL_RPM) - flywheel_R.getShaftRPM(), 1);
                flywheel_R.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                drivers->djiMotorTxHandler.encodeAndSendCanData();

                //process can data
                //drivers->djiMotorTxHandler.processCanSendData();
       	    }
       	    drivers->canRxHandler.pollCanData();
            modm::delay_us(10);
            
            //led handling
            int indexer_count = S_INDEXER_RPM/2000;
            int flywheel_count = S_FLYWHEEL_RPM/2000;
        }
        
    }
    return 0;
}
*/
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "TurretController.h"
#include "DriveTrainController.h"

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);//(not completley sure, but I belive its), the time for the Motors to be considered timed out.
tap::arch::PeriodicMilliTimer updateImuTimeout(2); //(not completley sure, but I belive its), the time for the IMU to be considered timed out.

//START General purpose variables / constants
src::Drivers *drivers = src::DoNotUse_getDrivers();
bool do_beyblading = false; 
bool use_exponentional_controlling = false;
bool useWASD = false;
float targetedAngle = 0;
float actualAngle = 0;
float angleOffSet = 0;
std::string WASDstring = ""; //Going to be the actual input string
std::string controlString = ""; //Will be the last two chars in the "WASDstring" string. (What we're actually going to be looking at)
//END General purpose variables / constants
//START Sensor / Input variables
float IMU_yaw_deg_per_sec, IMU_yaw, IMU_pitch = 0; // The yaw motor is the motor who's base is parallel to the ground. For reference: (https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.researchgate.net%2Ffigure%2Fa-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original_fig7_348803228&psig=AOvVaw0eM4KNtIkoRax8b-nyU_N1&ust=1671064576224000&source=images&cd=vfe&ved=0CA8QjRxqFwoTCOjnhOju9_sCFQAAAAAdAAAAABAJ)
double right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz = 0; //The Joysticks' values
float yaw_encoder_value = 0; //Hopefully, the encoder value for the 6020: https://aruw.gitlab.io/controls/taproot/api/classtap_1_1motor_1_1_dji_motor.html
//END Sensor / Input variables

/**
 * Used for reading the WASD keys on the keyboard. Updates two strings.
 * WASDstring: A "queue" of characters for intrepreting WASD controls into robot movement
 * controlString: The last (at most) two characters in the WASDString.
*/
void updateStrings() {
    //START Reading WASD
    if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W)) {
        if (WASDstring.find('w') == std::string::npos) {
            WASDstring += 'w';
        }
    } else {
        size_t pos = WASDstring.find('w');
        if (pos != std::string::npos) {
            WASDstring.erase(pos, 1);
        }
    }
    if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A)) {
        if (WASDstring.find('a') == std::string::npos) {
            WASDstring += 'a';
        }
    } else {
        size_t pos = WASDstring.find('a');
        if (pos != std::string::npos) {
            WASDstring.erase(pos, 1);
        }
    }
    if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S)) {
        if (WASDstring.find('s') == std::string::npos) {
            WASDstring += 's';
        }
    } else {
        size_t pos = WASDstring.find('s');
        if (pos != std::string::npos) {
            WASDstring.erase(pos, 1);
        }
    }
    if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D)) {
        if (WASDstring.find('d') == std::string::npos) {
            WASDstring += 'd';
        }
    } else {
        size_t pos = WASDstring.find('d');
        if (pos != std::string::npos) {
            WASDstring.erase(pos, 1);
        }
    }
    //STOP Reading WASD
    //START Updating the substring
    if (WASDstring.size() >= 2) {
        controlString = WASDstring.substr(WASDstring.size() - 2);
    } else {
        controlString = WASDstring;
    }
    //STOP Updating the substring
}

int main() {
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    drivers->can.initialize(); // Initializes the CAN
    Board::initialize(); // Board initiaizing stuff-mo-bobbers
    drivers->bmi088.initialize(500, 0.5, 0.1); // Initializing for the IMU (Type C)
    drivers->bmi088.requestRecalibration(); //Makes sure the IMU is calibrated correctly (Type C)
    drivers->leds.init();
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    drivers->remote.initialize(); //Remote initialization
    
    while (1) {
        modm::delay_us(2); //Delay so we don't run the board too fast
    	drivers->remote.read(); //Read the inputs of the remote

        //The following if statement is for reading from the wireless remote
        if(drivers->remote.isConnected()) {
            turretController->setMotorValues(false, false, 0, 0, 0);
            turretController->setMotorSpeeds(sendMotorTimeout.execute());
        } //Stop reading from Wireless Remote
        else { // Remote is not detected. So we need to tell the motors to turn off.
            turretController->stopMotors(sendMotorTimeout.execute());
        }

        drivers->canRxHandler.pollCanData();
    } //End of main loop
    return 0;
}
