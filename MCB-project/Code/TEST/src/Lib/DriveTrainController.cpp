#include "DriveTrainController.h"

namespace ThornBots {
    DriveTrainController::DriveTrainController(std::shared_ptr<CommunicationHandler> ch, std::shared_ptr<HardwareHandler> hh, std::shared_ptr<RefereeSystem> rs) {
        m_CommunicationHandler = ch;
        m_HardwareHandler = hh;
        m_RefereeSystem = rs;
    }

    bool DriveTrainController::Initialize() {
        //TODO:
        return true;
    }

    void DriveTrainController::Update() {
        if(useWASD) {
            OverrideWithKeyboard(); //Handling Translating
            //START Handling Rotating with Keyboard
            static char* KeysPressed; //Static so it's only created once.
            KeysPressed = m_CommunicationHandler->GetKeysPressed(); //Will get called every time Update() is called

            for(char* c = KeysPressed; *c != '\0'; c++){
                if(*c == 'q'){ //If q is pressed, rotate left
                    RotateSum(RotatingKeyboardConstant);
                } if(*c == 'e'){ //If e is pressed, rotate right
                    RotateSum(-1.0f * RotatingKeyboardConstant);
                }
            }
            //STOP Handling Rotating with Keyboard
        } else { //Not using Keyboard
            OverrideWithController(); //Handling translating
            //START Handling Rotating with Controller
            static float rightHorizontal; //Static so it's only created once
            rightHorizontal = m_CommunicationHandler->GetStickValue("RH"); //Will get called every time Update() is called
            RotateSum(rightHorizontal * RotatingControllerConstant);
            //STOP Handling Rotating with Controller 
        }

        //Handling Beyblading
        if(IsBeyblading) {
            RotateSum(BeyBladingConstant);
        }
        //TODO: Add logic for TurretCentric, and Locked/Unlocked Drivetrain
    }

    void DriveTrainController::EmergencyStop() {
        FL_WheelSpeed = 0;
        FR_WheelSpeed = 0;
        BL_WheelSpeed = 0;
        BR_WheelSpeed = 0;
        m_HardwareHandler->SetMotorPowerOutput(MOTOR_FRONT_LEFT, 0);
        m_HardwareHandler->SetMotorPowerOutput(MOTOR_FRONT_RIGHT, 0);
        m_HardwareHandler->SetMotorPowerOutput(MOTOR_BACK_LEFT, 0);
        m_HardwareHandler->SetMotorPowerOutput(MOTOR_BACK_RIGHT, 0);
    }

    //Use the controller to control the robot
    void DriveTrainController::OverrideWithController() {
        //TODO
    }

    //Use the keyboard to control the robot
    void DriveTrainController::OverrideWithKeyboard() {
        //TODO
    }

    //Reference to turret, turret is 0.
    void DriveTrainController::SetReferenceAngle(float NewAngle) {
        //TODO
    }

    //Rotate CW angle [degrees] and overwrite the current motor speeds
    void DriveTrainController::RotateOverride(float Angle) {
        //TODO
    }

    //Rotate CW angle [degrees] and add to the current motor speeds
    void DriveTrainController::RotateSum(float Angle) {
        //TODO
    }

    //Where you pass in the remote data to control the robot using manual contorl or sentry control
    void DriveTrainController::MoveDriveTrain(float Angle, uint32_t Speed) {
        //TODO
    }

    //Drivetrain keeps its current angle offset and rotates the entire robot to turn, keeping the angleoffset the same
    void DriveTrainController::LockDrivetrain() {
        //TODO
    }

    void DriveTrainController::UnlockDrivetrain() {
        //TODO
    }
};