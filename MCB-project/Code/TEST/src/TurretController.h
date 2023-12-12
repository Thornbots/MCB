#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "YawController.hpp"

namespace ThornBots {
    class TurretController {
        public: //Public Variables
            constexpr static int YAW_MOTOR_MAX_SPEED = 1000; //TODO: Make this value relevent
            constexpr static int YAW_MOTOR_MAX_VOLTAGE = 24000; //Should be the voltage of the battery. Unless the motor maxes out below that. //TODO: Check the datasheets
            constexpr static int INDEXER_MOTOR_MAX_SPEED = 6177; //With the 2006, this should give 20Hz
            constexpr static int FLYWHEEL_MOTOR_MAX_SPEED = 8333; //We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
            constexpr static int PITCH_MOTOR_MAX_SPEED = 1000; //TOOD: Make this value relevent
        
        private: //Private Variables
            tap::Drivers* drivers;
            //TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
            tap::motor::DjiMotor motor_Yaw = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "Yaw", 0, 0);
            tap::motor::DjiMotor motor_Pitch = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "Pitch", 0, 0);
            tap::motor::DjiMotor motor_Indexer = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0);
            tap::motor::DjiMotor motor_Flywheel1 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, true, "Flywheel", 0, 0);
            tap::motor::DjiMotor motor_Flywheel2 = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "Flywheel", 0, 0);
        
            constexpr static tap::algorithms::SmoothPidConfig pidControllerTurretMotorsConfig = { 20, 0, 0, 0, 8000, 1, 0, 1, 0, 69, 0 };
            constexpr static tap::algorithms::SmoothPidConfig pidControllerPitchConfig = { 800, 0.06, 80, 1500, 1000000, 1, 0, 1, 0, 0, 0 };
            tap::algorithms::SmoothPid pidControllerTurretMotors = tap::algorithms::SmoothPid(pidControllerTurretMotorsConfig);
            tap::algorithms::SmoothPid pidControllerPitch = tap::algorithms::SmoothPid(pidControllerPitchConfig);

        public: //Public Methods
            TurretController(tap::Drivers* driver);
            ~TurretController() {} //Intentionally left blank

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
            void driveTrainMovesTurretFollows();

            /*
            * Call this function when you want the drivetrain to be independent of the Turret.
            * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
            * This will allow the drivetrain to translate with the left stick, and the right stick is for the turret.
            * This function should be called when the right switch is in the Down state.
            * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
            * and right stick will control pitch and yaw of the turret.
            */
            void turretMovesDriveTrainFollows();

            /*
            * Call this function to convert the desired RPM for all of motors in the TurretController to a voltage level which
            * would then be sent over CanBus to each of the motor controllers to actually set this voltage level on each of the motors.
            * Should be placed inside of the main loop, and called every time through the loop, ONCE
            */
            void setMotorSpeeds();

            /*
            * Call this function to set all Turret motors to 0 desired RPM, calculate the voltage level in which to achieve this quickly
            * and packages this information for the motors TO BE SENT over CanBus
            */
            void stopMotors();

            /*
            * Call this function (any number of times) in order to ALLOW shooting. This does NOT mean that the turret WILL shoot.
            * The idea of this function is to allow implementation of AI auto-shooting easily, by "giving control" of the turret to the
            * communicatons received from the Jetson.
            * This function is not intended to be used for control when the driver is manually aiming/deciding to shoot or not.
            */
            void enableShooting();

            /*
            * Call this function (any number of times) in order to DISALLOW shooting. This does NOT mean that the turret WON'T shoot.
            * The idea of this function is to allow implementation of AI auto-shooting easily, by "giving control" of the turret to the 
            * communications received from the Jetson.
            * This function is not intended to be used for conrtol when the driver is manually aiming/deciding to shoot or not.
            */
            void disableShooting();

            /*
            * Call this function (any number of times) to reZero the yaw motor location. This will be used when first turning on the robot
            * and setting the Turret to where the front of the DriveTrain is. 
            * This function should be called when either in the bootup sequence, or when some, undetermined button is pressed on the keyboard.
            */
            void reZeroYaw();

        private: //Private Methods
    };
}