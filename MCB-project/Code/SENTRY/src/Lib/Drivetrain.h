#pragma once
#include "Params.h"

namespace Lib {
    /**
     * @brief Class API to control the drivetrain
     */
    class Drivetrain {
    public:
        /**
         * @brief Construct a new Drivetrain object
         */
        Drivetrain(tap::Drivers* driver);
        
        /**
         * @brief Destroy the Drivetrain object
         */
        ~Drivetrain();

        /**
         * @brief Set the reference angle the drivetrain will consider the "front" of the robot
         * 
         * @param angle_in_degrees the angle to set the reference angle to from 0.0f to 360.0f
         */
        void SetReferenceAngle(float angle_in_degrees);

        /**
         * @brief Rotates the drivetrain a number of degrees. Note that this will be overridden by auto rotation if it is toggled on.
         * 
         * @param angle_in_degrees the angle of rotation to rotate the drivetrain
         */
        void RotateDrivetrain(float angle_in_degrees);

        /**
         * @brief Moves the drivetrain given a speed and angle
         * 
         * @param speed the percentage of speed to move the drivetrain at, from 0.0f to 1.0f
         * @param angle_in_degrees the direction the drivetrain will drive towards, in reference to the reference angle
         */
        void MoveDrivetrain(float speed, float angle_in_degrees);

        /**
         * @brief Toggles auto rotation, which will set the drivetrain to constantly orient a corner of the robot towards the reference angle
         */
        void ToggleAutoRotate();

        /**
         * @brief Set auto rotation, which will set the drivetrain to constantly orient a corner of the robot towards the reference angle
         * 
         * @param isAutoRotating true to turn auto rotate on, false to turn it off
         */
        void SetAutoRotate(bool isAutoRotating);

        /**
         * @brief Toggles beyblading, which will set the drivetrain to constantly rotate while moving
         */
        void ToggleBeyblade();

        /**
         * @brief Sets beyblading, which will set the drivetrain to constantly rotate while moving
         * 
         * @param isBeyblading true to turn beyblading on, false to turn it off
         */
        void SetBeyblade(bool isBeyblading);

        /**
         * @brief Set the speed at which to beyblade at
         * 
         * @param speed the speed percentage to rotate at, from 0.0f to 1.0f
         */
        void SetBeybladeSpeed(float speed);

        /**
         * @brief Updates the drivetrain movement. Should be called each cycle to properly control the drivetrain.
         */
        void UpdateDrivetrain();

        /**
         * @brief Returns if auto rotate is turned on or off
         * 
         * @return true auto rotate is turned on
         * @return false auto rotate is turned off
         */
        bool GetAutoRotate();

        /**
         * @brief Returns if beyblading is turned on or off
         * 
         * @return true beyblading is turned on
         * @return false beyblading is turned off
         */
        bool GetBeyblade();
    private:
        const tap::Drivers* m_Driver; // taproot driver

        const tap::motor::DjiMotor m_MotorFL; // front left motor
        const tap::motor::DjiMotor m_MotorFR; // front right motor
        const tap::motor::DjiMotor m_MotorBL; // back left motor
        const tap::motor::DjiMotor m_MotorBR; // back right motor

        const tap::algorithms::SmoothPid m_PidController; // taproot pid controller

        const uint16_t m_MaxDrivetrainSpeed = 10000; // maximum speed the drivetrain will go in RPM
        const uint16_t m_MaxBeybladeSpeed = 10000; // maximum speed the drivetrain will rotate in RPM (note: RPM refers to the rotation speed of the motors, not the drivetrain)

        float m_ReferenceAngle = 0.0f; // the angle the drivetrain will consider "front"
        float m_RotationAngle = 0.0f; // the angle the drivetrain is trying to orient itself towards in reference to m_ReferenceAngle

        float m_DrivetrainSpeed = 0.0f; // the percentage speed of the drivetrain from 0.0f to 1.0f
        float m_RotationSpeed = 0.0f; // the percentage speed of rotation of the drivetrain from 0.0f to 1.0f
        float m_BeybladeSpeed = 0.0f;  // the percentage speed of the drivetrain's beyblade rotation from 0.0f to 1.0f

        float m_DrivetrainDirection = 0.0f; // the direction the drivetrain will drive towards in reference to m_ReferenceAngle using mecanum drive

        bool m_AutoRotate = false; // if true, the drivetrain will automatically rotate when it can to keep a corner at angle 0.
        bool m_Beyblade = false; // if true, the drivetrain will constantly rotate.
    };
}