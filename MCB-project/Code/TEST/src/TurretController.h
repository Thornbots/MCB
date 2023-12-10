namespace ThornBot {
    class TurretController {
        public: //Public Variables

        private: //Private Variables
        public: //Public Methods
            TurretController(tap::Drivers* driver);
            ~TurretController() {} //Intentionally left blank

            void driveTrainMovesTurretFollows();
            void turretMovesDriveTrainFollows();
            void setMotorSpeeds();
            void stopMotors();
            
        private: //Private Methods
    };
}