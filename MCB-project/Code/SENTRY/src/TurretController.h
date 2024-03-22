#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
#include "ModeledTurretController.h"
#include "PitchController.h"

namespace ThornBots {
    static tap::arch::PeriodicMilliTimer turretControllerTimer(2);
    class TurretController {
        public: //Public Variables
            constexpr static double PI = 3.14159;
            constexpr static int YAW_MOTOR_MAX_SPEED = 1000; //TODO: Make this value relevent
            constexpr static int YAW_MOTOR_MAX_VOLTAGE = 24000; //Should be the voltage of the battery. Unless the motor maxes out below that. //TODO: Check the datasheets
         
        private: //Private Variables
            tap::Drivers* drivers;
            //TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
            tap::motor::DjiMotor motor_Yaw = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS1, false, "Yaw", 0, 0);
            tap::motor::DjiMotor motor_Pitch = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "Pitch", 0, 0);
         
            ThornBots::ModeledTurretController yawController = ModeledTurretController();
            ThornBots::PitchController pitchController = PitchController();

            double pitchMotorVoltage, yawMotorVoltage;


            bool robotDisabled = false;

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
            * Should be called within the main loop, so called every time in the main loop when you want the described behavior.
            * This will allow the drivetrain to translate with the left stick, and the right stick is for the turret.
            * This function should be called when the right switch is in the Down state.
            * Enabling beyblading (left switch is not down) will override this state, and left stick will control drivetrain translating
            * and right stick will control pitch and yaw of the turret.
            */
            void turretMove(double desiredYawAngle, double desiredPitchAngle, double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double dt);

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
            void disable();
            void enable();

            /*
            * Call this function (any number of times) in order to ALLOW shooting. This does NOT mean that the turret WILL shoot.
            * The idea of this function is to allow implementation of AI auto-shooting easily, by "giving control" of the turret to the
            * communicatons received from the Jetson.
            * This function is not intended to be used for control when the driver is manually aiming/deciding to shoot or not.
            */
           

            /*
            * Call this function (any number of times) to reZero the yaw motor location. This will be used when first turning on the robot
            * and setting the Turret to where the front of the DriveTrain is. 
            * This function should be called when either in the bootup sequence, or when some, undetermined button is pressed on the keyboard.
            */
            void reZeroYaw();

            inline double getYawEncoderValue() {return tap::motor::DjiMotor::encoderToDegrees(motor_Yaw.getEncoderUnwrapped())*PI/180*(187/7182.0);}
            inline double getPitchEncoderValue() {return tap::motor::DjiMotor::encoderToDegrees(motor_Pitch.getEncoderUnwrapped())*PI/180;}
            inline double getYawVel() {return motor_Yaw.getShaftRPM()*PI/30*(187/7182.0);}
            inline double getPitchVel() {return motor_Pitch.getShaftRPM()*PI/30;}
        private: //Private Methods
            int getPitchVoltage(double targetAngle, double dt);
            int getYawVoltage(double driveTrainRPM, double yawAngleRelativeWorld, double yawRPM, double desiredAngleWorld, double dt);
        
    };
}