#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"

namespace ThornBots{
    class DriveTrainController{
        public: //Public Variables
        private: //Private Variables
        tap::Drivers *drivers;
        tap::motor::DjiMotor motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "ID1", 0, 0);
        tap::motor::DjiMotor motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "ID2", 0, 0);
        tap::motor::DjiMotor motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, true, "ID3", 0, 0);
        tap::motor::DjiMotor motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "ID4", 0, 0);
        
        static tap::algorithms::SmoothPidConfig pid_conf_dt = { 120, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
        static tap::algorithms::SmoothPidConfig pid_conf_DriveTrainFollowsTurret = {500, 0.5, 0, 0, 6000, 1, 0, 1, 0, 0, 0 }; //TODO: Tune this profile
        tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
        tap::algorithms::SmoothPid pidControllerDTFollowsT = tap::algorithms::SmoothPid(pid_conf_DriveTrainFollowsTurret);

        public: //Public Methods
            DriveTrainController(tap::Drivers* driver);
            ~DriveTrainController() {} //Intentionally blank

            void DriveTrainMovesTurretFollow();
            void TurretMoveDriveTrainFollow();
            void TurretMoveDriveTrainIndependent();
            void setMotorSpeeds();
            void stopMotors();

        private: //Private Methods
            void convertTranslationSpeedToMotorSpeeds();
            void adjustMotorSpeedWithTurnSpeed();
    };
}