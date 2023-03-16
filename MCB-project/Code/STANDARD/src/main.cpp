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
    drivers->bmi088.initialize(1600.0f, 1.0f, 1.0f); // Initializing for the IMU (Type C)
    drivers->bmi088.requestRecalibration(); //Makes sure the IMU is calibrated correctly (Type C)
    drivers->leds.init();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);

    drivers->remote.initialize(); //Remote initialization

    while (1) {
        modm::delay_us(2); //Delay so we don't run the board too fast

    	drivers->remote.read(); //Read the inputs of the remote

        //The following if statement is for reading from the IMU (Type A)
        if (updateImuTimeout.execute()) {
            drivers->bmi088.periodicIMUUpdate();
            IMU_yaw = 0;//drivers->mpu6500.getYaw();
            IMU_yaw_deg_per_sec = 0;//drivers->mpu6500.getGx(); // Gets the IMU's deg/sec reading for the x plane (assuming that is the yaw plane)
            actualAngle = drivers->bmi088.getPitch(); // Gets the IMU"s deg reading from the y plane (pitch plane)
        } //Stop reading from IMU (Type A)

        //The following if statement is for reading from the wireless remote
        if(drivers->remote.isConnected()) {
            //START Reading the Joystick Values
            left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            // allocateSpeeds(right_stick_horz);
            //END Reading the Joystick Values
            //START Reading the Switch Values
            if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::MID) { 
                do_beyblading = false;
      	    } else if((drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::UP)) { 
                do_beyblading = true;
            } else if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN) { 
                //This is an undefined state as of now
            }
            //END Reading the Switch Values

            updateStrings();
            driveTrainController->setMotorValues(sendMotorTimeout.execute(), useWASD, do_beyblading, right_stick_vert, right_stick_horz, controlString); //TODO: Might need to replace controlString with a pointer to controlString to prevent memory leaks
            driveTrainController->setMotorSpeeds(sendMotorTimeout.execute());
            turretController->setMotorValues(useWASD, do_beyblading, angleOffSet, right_stick_vert, right_stick_horz);
            turretController->setMotorSpeeds(sendMotorTimeout.execute());
            // yaw_encoder_value = 0.01745329251 * yaw_motor.encoderToDegrees(NULL); //The decimal value is Pi/180. (Converting it to radians)

            drivers->canRxHandler.pollCanData();
        } //Stop reading from Wireless Remote

        else { // Remote is not detected. So we need to tell the motors to turn off.
            driveTrainController->stopMotors(sendMotorTimeout.execute());
            turretController->stopMotors(sendMotorTimeout.execute());
        }
    } //End of main loop
    return 0;
}