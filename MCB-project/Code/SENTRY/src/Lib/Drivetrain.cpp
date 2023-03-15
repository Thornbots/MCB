#include "Drivetrain.h"

namespace Lib {
    Drivetrain::Drivetrain(tap::Drivers* driver)
        : m_Driver(driver) {
        m_MotorFL = START_MOTOR(FL_MOTORID);
        m_MotorFR = START_MOTOR(FR_MOTORID);
        m_MotorBL = START_MOTOR(BL_MOTORID);
        m_MotorBR = START_MOTOR(BR_MOTORID);
        m_PidController = GET_PID();
    }

    Drivetrain::~Drivetrain() {

    }

    void Drivetrain::SetReferenceAngle(float angle_in_degrees) {
        m_ReferenceAngle = ( angle_in_degrees % 360.0f );
    }

    void Drivetrain::RotateDrivetrain(float angle_in_degrees) {
        m_RotationAngle = angle_in_degrees;
    }

    void Drivetrain::MoveDrivetrain(float speed, float angle_in_degrees) {
        m_DrivetrainDirection = angle_in_degrees;   
        m_DrivetrainSpeed = speed;
    }

    void Drivetrain::ToggleAutoRotate() {
        m_AutoRotate = !m_AutoRotate;
    }

    void Drivetrain::SetAutoRotate(bool isAutoRotating) {
        m_AutoRotate = isAutoRotating;
    }

    void Drivetrain::ToggleBeyblade() {
        m_Beyblade = !m_Beyblade;
    }

    void Drivetrain::SetBeyblade(bool isBeyblading) {
        m_Beyblade = isBeyblading;
    }

    void Drivetrain::SetBeybladeSpeed(float speed) {
        m_BeybladeSpeed = speed;
    }

    void Drivetrain::UpdateDrivetrain() {
        //update drivetrain here
    }

    bool Drivetrain::GetAutoRotate() {
        return m_AutoRotate;
    }

    bool Drivetrain::GetBeyblade() {
        return m_Beyblade;
    }
}