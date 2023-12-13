#pragma once
#include "DriveTrainController.h"
#include "TurretController.h"

namespace ThornBots {
    static tap::arch::PeriodicMilliTimer IMUTimer(2);

    class RobotController {
    public: //Public Variables
    constexpr static double TURN_RATIO = 0.5; //TODO: Make this number actualy relevent
    constexpr static double TRANSLATION_RATIO = 1.0; //TODO: Make this number actualy relevent
    constexpr static double FAST_BEYBLADE_FACTOR = 0.7;
    constexpr static double SLOW_BEYBLADE_FACTOR = 0.35;
    constexpr static double MAX_SPEED = 10000; //TODO: Make this number actually relevent
    constexpr static double PI = 3.14159;

    private: //Private Variables
        tap::Drivers* drivers;
        ThornBots::DriveTrainController *driveTrainController;
        ThornBots::TurretController *turretController;
        int leftSwitchValue;
        int rightSwitchValue;
        double driveTrainAngleRelativeToWorld;
        double turretAngleRelativeToWorld;
        double desiredAngleRelativeToWorld;
        tap::communication::serial::Remote::SwitchState rightSwitchState = tap::communication::serial::Remote::SwitchState::DOWN;
        tap::communication::serial::Remote::SwitchState leftSwitchState = tap::communication::serial::Remote::SwitchState::DOWN;


    public: //Public Methods
        RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController);
        ~RobotController() {} //Intentionally left blank

        /*
        * Call this function once before calling update, outside of the control loop.
        * This function will handle creating all of the objects for the RobotController as well as initialize all objects being used.
        */
        void inialize();

        /*
        * Call this function as often as you can. (i.e., in main do: while(1) { robotController.update(); })
        * This function will handle all necessary operations to control the robot outside of initializing and creating objects.
        * To initialize, call the initialize() function once outside of the main loop.
        * This function will handle all inputs and call relevent functions for TurretController and DriveTrainController.
        */
        void update();

        /*
        * This function will call stopMotors for all classes used by the RobotController.
        * This should be called when the remote is not detected, or the robot needs to stop spinning motors or perform an emergency stop for any reason.
        * Will NOT prevent the robot from being able to move again without power cycling. 
        * (i.e., if the remote disconnects, this should be called to make the robot pause everything, and allow the robot to work as normal once the remote reconnects)
        */
        void stopRobot();

        /*
        * Call this to update all control variables that are DIRECTLY derived from the control or keyboard input(s)
        * (i.e., will update ALU_Angle, but not TurretAngleRelativeToDriveTrain)
        * (i.e., will update leftStickVertical, but not translationAngle or translationMagnitude)
        * //TODO: Add Keyboard functionality to this method
        */
        void updateAllInputVariables();

    private: //Private Methods
        double getAngle(double x, double y);

        /*
        * Reads inputs from the keyboard and mouse and checks to see if KBM(keyboard and Mouse) mode should
        * be enabled or not. It requires the pressing of CTRL + SHIFT + R to enable KBM mode.
        */
        bool toggleKeyboardAndMouse();
    };
}