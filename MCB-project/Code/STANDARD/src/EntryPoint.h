namespace ThornBots{
    class EntryPoint{
        public:
            int main();
            bool initialize();
            void update(uint23_t cycleTimeInUS, bool runRobot);
            void destroy();
        private:
            ThornBots::RobotController* m_RobotController;
    }
}