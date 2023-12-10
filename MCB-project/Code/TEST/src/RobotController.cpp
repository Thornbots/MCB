#include "RobotController.h"

namespace ThornBots {
    class RobotController{
        public: //Public Methods
            /*
            * Constructor for RobotController
            */
            RobotController::RobotController(tap::Drivers* driver, ThornBots::DriveTrainController* driveTrainController, ThornBots::TurretController* turretController) {
                this->drivers = driver;
                this->driveTrainController = driveTrainController;
                this->turretController = turretController;
            }


            void inialize() {
                this->driveTrainController.initialize();
                this->turretController.initialize();
                //TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's reading correctly, ect)
            }

            void update() {
                //TODO
            }

            void stopRobot() {
                //TODO
            }

            void toggleKeyBoardandMouse() {
                //TODO
            }

            void updateAllInputVariables() {
                //TODO
            }

        private: //Private Methods
            double getAngle(double x, double y) {
                //TODO
            }

            bool toggleKeyboardAndMouse() {
                //TODO
            }

            int findLeftSwitchState() {
                //TODO
            }

            int findRighTSwitchState() {
                //TODO
            }
    };
}