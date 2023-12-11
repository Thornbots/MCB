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
        private: //Private Variables
        tap::Drivers *drivers;
        tap::motor::DjiMotor motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "ID1", 0, 0);
        tap::motor::DjiMotor motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "ID2", 0, 0);
        tap::motor::DjiMotor motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, true, "ID3", 0, 0);
        tap::motor::DjiMotor motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "ID4", 0, 0);
        
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_dt = { 120, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
        constexpr static tap::algorithms::SmoothPidConfig pid_conf_DriveTrainFollowsTurret = {500, 0.5, 0, 0, 6000, 1, 0, 1, 0, 0, 0 }; //TODO: Tune this profile
        tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
        tap::algorithms::SmoothPid pidControllerDTFollowsT = tap::algorithms::SmoothPid(pid_conf_DriveTrainFollowsTurret);

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
            * This will allow the drivetrain to translate with left stick, and turn with the right stick.
            * This function should be called when the right switch is in the Up state.
            * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
            * and right stick will control pitch and yaw of the turret.
            */
            void DriveTrainMovesTurretFollow();

            /*
            * Call this function when you want the drivetrain to be independent of the Turret.
            * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
            * This will allow the drivetrain to translate with the left stick, and the right stick is for the turret.
            * This function should be called when the right switch is in the Down state.
            * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
            * and right stick will control pitch and yaw of the turret.
            */
            void TurretMoveDriveTrainFollow();

            /*
            * Call this function when you want the drivetrain and turret to move independently. 
            * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
            * This will allow the drivetrain to translate with the left stick, and the right stick is for turning the drivetrain.
            * The turret during this should remain at the same angle relative to world. (It stays stationary)
            * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
            * and right stick will control pitch and yaw of the turret.
            */
            void TurretMoveDriveTrainIndependent();

            /*
            * Call this function to convert the desired RPM for all of motors in the DriveTrainController to a voltage level which
            * would then be sent over CanBus to each of the motor controllers to actually set this voltage level on each of the motors.
            * Should be placed inside of the main loop, and called every time through the loop, ONCE
            */
            void setMotorSpeeds();

            /*
            * Call this function to set all DriveTrain motors to 0 desired RPM, calculate the voltage level in which to achieve this quickly
            * and packages this information for the motors TO BE SENT over CanBus
            */
            void stopMotors();

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