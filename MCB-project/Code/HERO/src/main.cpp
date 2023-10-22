#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>
#include <iostream>
#include "tap/architecture/periodic_timer.hpp"
static tap::algorithms::SmoothPidConfig pid_conf_dt = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 69, 0 };
tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
src::Drivers *drivers;
int indexerDesiredRPM = 0;
int indexerMaxRPM = 1000;
int indexerStepSpeed = 200;
bool alreadyChanged = false;
int flywheelDesiredRPM = 0;
int flywheelMaxRPM = 2000;

int main() {
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    drivers->can.initialize();
    drivers->remote.initialize();
    tap::motor::DjiMotor indexer = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "ID1", 0, 0);
    tap::motor::DjiMotor flywheel_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "ID1", 0, 0);
    tap::motor::DjiMotor flywheel_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, true, "ID1", 0, 0);
    indexer.initialize();
    flywheel_one.initialize();
    flywheel_two.initialize();

    while (1) {
        drivers->canRxHandler.pollCanData();
        drivers->remote.read(); //Reading the remote before we check if it is connected yet or not.

        if(drivers->remote.isConnected()) { //Doing stuff for the remote every loop   
            if(drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::MID) { 
                //Do Nothing
                alreadyChanged = false;
      	    } else if((drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::UP)) { 
                //Increment the Indexer Speed
                if(!alreadyChanged) {
                    if(indexerDesiredRPM<indexerMaxRPM) {
                        indexerDesiredRPM+=indexerStepSpeed;
                    }
                    alreadyChanged = true;
                }
            } else if(drivers->remote.getSwitch(drivers->remote.Switch::LEFT_SWITCH) == drivers->remote.SwitchState::DOWN) { 
                //Decrement the Indexer, stopping at 0
                if(!alreadyChanged) {
                    if(-indexerDesiredRPM<0) {
                        indexerDesiredRPM-=indexerStepSpeed;
                    }
                    alreadyChanged = true;
                }
            }
            if(drivers->remote.getSwitch(drivers->remote.Switch::RIGHT_SWITCH) == drivers->remote.SwitchState::UP) { 
                //Set the Flywheel Speed to MaxRPM
                flywheelDesiredRPM = flywheelMaxRPM;
      	    } else{
                //Set the Flywheel Speed to 0
                flywheelDesiredRPM = 0;
            }
            if(sendMotorTimeout.execute()) {
                pidController.runControllerDerivateError(indexerDesiredRPM - indexer.getShaftRPM(), 1);
                indexer.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                pidController.runControllerDerivateError(flywheelDesiredRPM - flywheel_one.getShaftRPM(), 1);
                flywheel_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                pidController.runControllerDerivateError(flywheelDesiredRPM - flywheel_two.getShaftRPM(), 1);
                flywheel_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
                drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
            }
        } else { //Remote not connected, so have everything turn off (Saftey features!)
                indexer.setDesiredOutput(static_cast<int32_t>(0));
                flywheel_one.setDesiredOutput(static_cast<int32_t>(0));
                flywheel_two.setDesiredOutput(static_cast<int32_t>(0));
                drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
        }

    }
    return 0;
}