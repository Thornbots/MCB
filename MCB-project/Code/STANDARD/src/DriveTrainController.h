#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"
#include "drivers_singleton.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include "tap/architecture/periodic_timer.hpp"
#include "tap/motor/dji_motor.hpp"
#include "drivers_singleton.hpp"
static tap::algorithms::SmoothPidConfig pid_conf_dt = { 120, 0, 0, 0, 8000, 1, 0, 1, 0, 0, 0 };
static tap::algorithms::SmoothPidConfig pid_conf_DriveTrainFollowsTurret = {500, 0.5, 0, 0, 6000, 1, 0, 1, 0, 0, 0 }; //TODO: Tune this profile
static constexpr float REFINED_ANGLE_OFFSET = 210.0f;
static constexpr int MAX_SPEED = 6000; //The abs(maximum speed) we want the drivetrain motors to go to
static constexpr double PI = 3.14159; //Everyone likes Pi!


namespace ThornBots {
    class DriveTrainController {
    public:
        //Contructor and Destructor - Currently destructor is not being used
        DriveTrainController(tap::Drivers* driver);
        ~DriveTrainController();

        //HUH??
        tap::motor::DjiMotor motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "ID1", 0, 0);
        tap::motor::DjiMotor motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "Call 858-267-8107 for a good time!", 0, 0);

        //----------------------------------Functions----------------------------------
        
        /*
        * DriveTrainMovesTurretFollow will take the turnSpeed, translationSpeed and translationAngle and use
        * them to move the drivetrain in the desired direction.
        */
        void DriveTrainMovesTurretFollow(double turnSpeed, double translationSpeed, double translationAngle);

        /*
        * TurretMovesDriveTrainFollows will the turnSpeed, translationSpeed, and translationAngle and use them
        * to move the drivetrain in the desired direction
        */
        void TurretMovesDriveTrainFollow(double translationSpeed, double translationAngle, double driveTrainAngleFromTurret);

        /*
        * Drive trains will move independly of the turret and will need, translationsSpeed, translationAngle, and driveTrainAngleFromTurret
        * to move the drivetrain without affecting the turret.
        */
        void TurretMovesDriveTrainIndependent(double translationSpeed, double translationAngle, double driveTrainAngleFromTurret);
        
        /*
        * Once our global motor speeds are set, call this function with sendMotorTimeout.execute() as the parameter and it will
        * send the motor speeds to the motors with the desired delay from sendMotorTimeout.execute.
        */
        void setMotorSpeeds(bool sendMotorTimeout);

        /*
        * This function will set the motor speeds to 0 then call setMotorSpeeds with sendMotorTimeout.execute() as the parameter.
        */
        void stopMotors(bool sendMotorTimeout);


        //TO BE DELETED - TEMPORARY
        void setMotorValues(double right_stick_vert, double right_stick_horz, double left_stick_vert, double left_stick_horz, float yaw_angle, int rightSwitchState, int leftSwitchValue);


        
    private:

        //----------------------------------Variables----------------------------------

        int motor_one_speed = 0; //Driver's front
        int motor_two_speed = 0; //Passenger's front
        int motor_three_speed = 0; //Driver's back
        int motor_four_speed = 0; //Passenger's back


        //START getters and setters
        inline int getMotorOneSpeed() { return motor_one_speed; }
        inline int getMotorTwoSpeed() { return motor_two_speed; }
        inline int getMotorThreeSpeed() { return motor_three_speed; }
        inline int getMotorFourSpeed() { return motor_four_speed; }
        inline void setMotorOneSpeed(int speed) { motor_one_speed = speed; }
        inline void setMotorTwoSpeed(int speed) { motor_two_speed = speed; }
        inline void setMotorThreeSpeed(int speed) { motor_three_speed = speed; }
        inline void setMotorFourSpeed(int speed) { motor_four_speed = speed; }
        inline int getTranslationSpeed() { return translation_speed; }
        inline int getRotationSpeed() { return rotation_speed; }
        inline int getBeybladingSpeed() { return beyblading_speed; }
        inline void setTranslationSpeed(int speed) { translation_speed = speed; }
        inline void setRotationSpeed(int speed) { rotation_speed = speed; }
        inline void setBeybladingSpeed(int speed) { beyblading_speed = speed; }
        inline int getMaxSpeed() { return MAX_SPEED; }
        //STOP getters and setters

        //Functions
        int getMotorOneSpeedWithCont(bool doBeyblading, double right_stick_vert, double right_stick_horz, double left_stick_vert, double left_stick_horz);
        int getMotorTwoSpeedWithCont(bool doBeyblading, double right_stick_vert, double right_stick_horz, double left_stick_vert, double left_stick_horz);
        int getMotorThreeSpeedWithCont(bool doBeyblading, double right_stick_vert, double right_stick_horz, double left_stick_vert, double left_stick_horz);
        int getMotorFourSpeedWithCont(bool doBeyblading, double right_stick_vert, double right_stick_horz, double left_stick_vert, double left_stick_horz);
        int getMotorSetOneTranslatingSpeed(double xPosition, double yPosition);
        int getMotorSetTwoTranslatingSpeed(double xPosition, double yPosition);
        double getAngle(double xPosition, double yPosition);
        double getMagnitude(double xPosition, double yPosition);
        double getScaledQuadratic(double magnitude);
        double updateMotorSpeeds(double MotorNewSpeed, double MotorCurrentSpeed, int slewRate);

        /*
        * This function will take the translationSpeed and translationAngle and convert them to motor speeds for the drivetrain.
        */
        void convertTranslationSpeedToMotorSpeeds(double translationSpeed, double translationAngle);

        /*
        * This function adjusts the motor speed of each turret based on how turn speed affects each motor.
        */
        void adjustMotorSpeedWithTurnSpeed(double turnSpeed);
        
        
        //float power_limit;
        float yaw_motor_angle = 0.0f;
        bool lockRotation = true;
        bool lockDrivetrain = true;
        int translation_speed, rotation_speed, beyblading_speed = 0;
        double beyblading_factor = 0.7; //How much of the max_speed beyblading will eat up while robot is not translating range this from [0, 1]
        bool use_exponentional_controlling = true;

        tap::Drivers *drivers;
        tap::motor::DjiMotor motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "PURDON'T!", 0, 0);
        tap::motor::DjiMotor motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, true, "Put the possum in his room", 0, 0);
        tap::algorithms::SmoothPid pidController = tap::algorithms::SmoothPid(pid_conf_dt);
        tap::algorithms::SmoothPid pidControllerDTFollowsT = tap::algorithms::SmoothPid(pid_conf_DriveTrainFollowsTurret);

    };
}