

namespace ThornBots{
    class RobotController{
        public:
            bool initialize();
            void update();
            void emergencyStop();
        private:
            ThornBots::DriveTrainController* m_DriveTrainController;
            ThornBots::TurretController* m_TurretController;
    }
}