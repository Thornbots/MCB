#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"

namespace ThornBots{
    class DriveTrainController{
        public: //Public Variables
            constexpr static double PI = 3.14159; //Everyone likes Pi!
            constexpr static tap::algorithms::SmoothPidConfig pid_conf_dt = { 20, 0, 0, 0, 18000, 1, 0, 1, 0, 0, 0 };
            constexpr static tap::algorithms::SmoothPidConfig pid_conf_DriveTrainFollowsTurret = {500, 0.5, 0, 0, 18000, 1, 0, 1, 0, 0, 0 }; //TODO: Tune this profile
        private: //Private Variables
            tap::Drivers *drivers;
            tap::motor::DjiMotor motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "ID1", 0, 0);
            tap::motor::DjiMotor motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "ID2", 0, 0);
            tap::motor::DjiMotor motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, true, "ID3", 0, 0);
            tap::motor::DjiMotor motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "ID4", 0, 0);
            tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
            tap::algorithms::SmoothPid pidControllerDTFollowsT = tap::algorithms::SmoothPid(pid_conf_DriveTrainFollowsTurret);
            bool robotDisabled = false;

        public: //Public Methods
            DriveTrainController(tap::Drivers* driver);
            ~DriveTrainController() {} //Intentionally blank

            /*
            * Call this function once, outside of the main loop.
            * This function will initalize all of the motors, timers, pidControllers, and any other used object.
            * If you want to know what initializing actually does, ping Teaney in discord, or just Google it. It's pretty cool.
            */
            void initialize();
            
            /*
            * Call this function when you want the Turret to follow the DriveTrain
            * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
            * This will allow the drivetrain to translate with left stick, and turn with the right stick or beyblade depending how this is called.
            */
            void moveDriveTrain(double turnSpeed, double translationSpeed, double translationAngle);

            /*
            * Call this function to convert the desired RPM for all of motors in the DriveTrainController to a voltage level which
            * would then be sent over CanBus to each of the motor controllers to actually set this voltage level on each of the motors.
            * Should be placed inside of the main loop, and called periodically, every 
            */
            void setMotorSpeeds();

            /*
            * Call this function to set all DriveTrain motors to 0 desired RPM. CALL setMotorSpeeds() FOR THIS TO WORK
            */
            void stopMotors();
            void disable();
            void enable();

        private: //Private Methods
            /*
            * Call this function to calculate and OVERWRITE DriveTrain motors' RPMs to translate at the given magnitude and angle
            * Angle should be in radians, with 0 being straight forward relative to the drivetrain and the positive direction being CCW.
            * (i.e. So to translate directly to the left, relative to the drivetrain, you would call this function as: convertTranslationSpeedToMotorSpeeds(0.75, pi/2);)
            */
            void convertTranslationSpeedToMotorSpeeds(double magnitude, double angle);

            /*
            * Call this function to calculate and ADJUST DriveTrain motors' RPMs to rotate at the given turnSpeed. This input is unitless, should be from [0, 1],
            * and simply multiplies it by a constant, adjustable maximum factor of maximum speed.
            */
            void adjustMotorSpeedWithTurnSpeed(double turnSpeed);
    };
}