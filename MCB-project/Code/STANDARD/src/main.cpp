// General library includes
#include <cmath>
#include <iostream>
#include <string>

// Drivers
#include "drivers_singleton.hpp"
#include "drivers_singleton.hpp"
#include "drivers_singleton.hpp"

// Taproot
#include "tap/architecture/periodic_timer.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/board/board.hpp"

//Our .h's
#include "DriveTrainController.h"
#include "TurretController.h"

tap::arch::PeriodicMilliTimer sendDrivetrainTimeout(2);
tap::arch::PeriodicMilliTimer sendTurretTimeout(2);
tap::arch::PeriodicMilliTimer updateIMUTimeout(2);
std::string WASDstring = ""; //Going to be the actual input string
std::string controlString = ""; //Will be the last two chars in the "WASDstring" string. (What we're actually going to be looking at)
src::Drivers *drivers;
bool useWASD = false;
bool doBeyblading = false;
double right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz = 0.0;
double angleOffset = 0.0;


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

/**
 * Reads inputs from the mouse and handles what do do with that information
 * This function assumes that we are already in WASD mode (useWASD == true)
*/
void micky() {
    
    return;
}

/**
 * Reads inputs from the keyboard and mouse and habdkes what do do with that information
*/
void readKeyboardAndMicky() {
    useWASD = false;
    if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL)) {
        if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT)) {
            if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) {
                useWASD = true; //Toggling between keyboard/mouse and controller input
            } //End R
        } //End shift
    } //End CTRL

    if(useWASD) {
        if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Q)) {
            //TODO: Make this rotate the robot to the left (CCW)
        } else if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::E)) {
            //TODO: Make this rotate the robot to the right (CW)
        }
        if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::F)) {
            doBeyblading = true;
        } else { doBeyblading = false; }
        micky();
    } else {
        //we are not using WASD. So pee pee poo poo. Enjoy the crap that is my code
    }
    return;
}

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);
    drivers->bmi088.requestRecalibration();
    ThornBots::DriveTrainController *driveTrainController = new ThornBots::DriveTrainController(drivers);
    ThornBots::TurretController *turretController = new ThornBots::TurretController(drivers);
    
    while (1) {
        drivers->canRxHandler.pollCanData();
        drivers->remote.read(); //Reading the remote before we check if it is connected yet or not.

        if(updateIMUTimeout.execute()) {
            drivers->bmi088.periodicIMUUpdate();
            angleOffset = drivers->bmi088.getYaw(); //TODO: Make this calculate the AngleOffset and not the raw angle.
        } //Stop reading from the IMU

        if(drivers->remote.isConnected()) { //Doing stuff for the remote every loop
            updateStrings();
            readKeyboardAndMicky();
            if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::MID) { 
                doBeyblading = false;
                turretController->stopShooting();
      	    } else if((drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::UP)) { 
                doBeyblading = true;
                turretController->stopShooting();
            } else if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN) { 
                turretController->startShooting();
                doBeyblading = false;
            }
            if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::MID) { 
                //Nothing as of now
      	    } else if((drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::UP)) { 
                //Nothing as of now
            } else if(drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN) { 
                turretController->reZero();
            }

            right_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
            right_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
            left_stick_vert = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            left_stick_horz = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);

            driveTrainController->setMotorValues(useWASD, doBeyblading, right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz, controlString, turretController->getYawEncoderAngle());
            driveTrainController->setMotorSpeeds(sendDrivetrainTimeout.execute());
            turretController->setMotorValues(useWASD, doBeyblading, angleOffset, right_stick_vert, right_stick_horz, driveTrainController->motor_one.getShaftRPM(), driveTrainController->motor_four.getShaftRPM(), 0.0f);
            turretController->setMotorSpeeds(sendTurretTimeout.execute()); 
            // drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
           

        } else { //Remote not connected, so have everything turn off (Saftey features!)
            driveTrainController->stopMotors(sendDrivetrainTimeout.execute());
            turretController->stopMotors(sendTurretTimeout.execute());
        }
    }
    return 0;
}