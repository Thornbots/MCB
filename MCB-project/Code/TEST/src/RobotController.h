#pragma once
#include "DriveTrainController.h"
#include "TurretController.h"

namespace ThornBots {
    class RobotController {
    public: //Public Variables
    constexpr static double TURN_RATIO = 0.5; //TODO: Make this number actualy relevent
    constexpr static double TRANSLATION_RATIO = 1.0; //TODO: Make this number actualy relevent
    constexpr static double FAST_BEYBLADE_FACTOR = 0.7;
    constexpr static double SLOW_BEYBLADE_FACTOR = 0.35;
    constexpr static double MAX_SPEED = 10000; //TODO: Make this number actually relevent

    private: //Private Variables
        ThornBots::DriveTrainController *driveTrainController;
        ThornBots::TurretController *turretController;
        int leftSwitchValue;
        int rightSwitchValue;
        double driveTrainAngleRelativeToWorld;
        double turretAngleRelativeToWorld;
        double desiredAngleRelativeToWorld;

    public: //Public Methods
        RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController);
        ~RobotController() {} //Intentionally left blank

        void inialize();
        void update();
        void stopRobot();
        void toggleKeyBoardandMouse();
        void updateAllInputVariables();

    private: //Private Methods
        double getAngle(double x, double y);
        bool toggleKeyboardAndMouse();
        int findLeftSwitchState();
        int findRighTSwitchState();
    };
}