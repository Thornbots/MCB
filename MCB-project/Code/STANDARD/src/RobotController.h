#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include <cmath>
#include "drivers_singleton.hpp"

#include "DriveTrainController.h"
#include "TurretController.h"

tap::arch::PeriodicMilliTimer sendDrivetrainTimeout(2);
tap::arch::PeriodicMilliTimer sendTurretTimeout(2);

namespace ThornBots {
    class RobotController {
    public:
        //Constructor
        RobotController(tap::Drivers* m_driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController);
        //Destructor
        ~RobotController();

        /*
        * Main function for the RobotController class. This function will be called in the main.cpp file.
        * This function will get and read our inputs and determinie what DriveTrainController and TurretController do
        * based on the inputs.
        */
        void update();

        /*
        * This function will call setMotorSpeeds with sendMotorTimeout.execute() as the parameter to DriveTrain 
        * and Turret Controller.
        */
        void stopRobot();

    private:
        //Variables
        static constexpr double BEYBLADE_FACTOR = 0.7; //Change this to change the maximum factor of speed of the beyblading
        static constexpr int MAX_SPEED = 6000; //The abs(maximum speed) we want the drivetrain motors to go to
        static constexpr double PI = 3.14159;

        double beybladeFactor = 0;
        bool keyboardAndMouseEnabled = false;
        int leftSwitchValue;
        int rightSwitchValue;
        double distance = 0.0;
        double turnSpeed = 0.0;
        double translationAngle = 0.0;
        double magnitude = 0.0;
        double translationSpeed = 0.0;
        int16_t wheel_value = 0;

        double right_stick_vert = 0.0;
        double right_stick_horz = 0.0;
        double left_stick_vert = 0.0;
        double left_stick_horz = 0.0;



        tap::Drivers* drivers;
        ThornBots::DriveTrainController *driveTrainController;
        ThornBots::TurretController *turretController;

        //temp to be deleted
        float temp_yaw_angle = 0.0;


        //Functions

        /*
        * This function will find relation of a point to the orgin and return the angle in degrees. Furthermore
        * it will set the 0 refrence aggle to the front of the drivetrain.
        */
        double getAngle(double xPosition, double yPosition);
        
        /**
        * Reads inputs from the keyboard and mouse and checks to see if KBM(keyboard and Mouse) mode should
        * be enabled or not. It requires the pressing of CTRL + SHIFT + R to enable KBM mode.
        */
        bool toggleKeyboardAndMouse();

        /*
        * Reads the state of the left switch on the remote and sets leftSwitchValue to 2 if the switch is up,
        * 1 if the switch is in the middle, and 0 if the switch is down.
        */
        void findLeftSwitchState();

        /*
        * Reads the state of the right switch on the remote and sets rightSwitchValue to 2 if the switch is up,
        * 1 if the switch is in the middle, and 0 if the switch is down.
        */
        void findRightSwitchState();
    };
}