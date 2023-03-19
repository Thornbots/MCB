#include "DrivetrainController.h"

namespace ThornBots {
    DrivetrainController::DrivetrainController() {
        m_MotorFL = &MOTOR_1;
        m_MotorFR = &MOTOR_2;
        m_MotorBL = &MOTOR_3;
        m_MotorBR = &MOTOR_4;
    }

    void DrivetrainController::Initialize() {
        m_MotorFL->initialize();
        m_MotorFR->initialize();
        m_MotorBL->initialize();
        m_MotorBR->initialize();
    }

    int DrivetrainController::GetMotorSpeed(const char* motor_name) {
        if(motor_name[0] == 'F') {
            if(motor_name[1] == 'R') {
                return m_FRSpeed;
            } else if(motor_name[1] == 'L') {
                return m_FLSpeed;
            }
        } else if(motor_name[0] == 'B') {
            if(motor_name[1] == 'R') {
                return m_BRSpeed;
            } else if(motor_name[1] == 'L') {
                return m_BLSpeed;
            }
        }
        return -1;
    }
    
    bool DrivetrainController::SetMotorSpeed(int speed, std::string motor_name) {
        if(motor_name[0] == 'F') {
            if(motor_name[1] == 'R') {
                m_FRSpeed = speed;
                return true;
            } else if(motor_name[1] == 'L') {
                m_FLSpeed = speed;
                return true;
            }
        } else if(motor_name[0] == 'B') {
            if(motor_name[1] == 'R') {
                m_BRSpeed = speed;
                return true;
            } else if(motor_name[1] == 'L') {
                m_BLSpeed = speed;
                return true;
            }
        }
        return false;
    }
   
    bool DrivetrainController::KeyboardEnabled() {
        return false;
        //return SWITCH_STATE('L') == 0;
    }

    bool DrivetrainController::BeybladingEnabled() {
        return false;
        //return SWITCH_STATE('L') == 2;
    }

    int DrivetrainController::CalculateMotorSpeed(const char* motor_name) {
        if (KeyboardEnabled()) return CalculateMotorSpeedWithKeyboard(motor_name);
        return CalculateMotorSpeedWithController(motor_name);
    }

    int DrivetrainController::CalculateMotorSpeedWithController(const char* motor_name) {
        int translatingSpeed = CalculateTranslatingSpeed(motor_name);
        int inverse = -1;
        if (motor_name[0] == 'F') inverse = 1;
        if(BeybladingEnabled()) {
            float tmp = inverse * -1 * m_BeybladeFactor * m_MaximumSpeed + translatingSpeed;
            return ((int) abs(tmp) <= m_MaximumSpeed) ? tmp : ((int) tmp > 0) ? m_MaximumSpeed : -1 * m_MaximumSpeed;
        }
        float tmp = inverse * STICK_DATA("RH") * m_MaximumSpeed + translatingSpeed;
        return ((int) abs(tmp) <= m_MaximumSpeed) ? tmp : ((int) tmp > 0) ? m_MaximumSpeed : -1 * m_MaximumSpeed;
    }
    
    int DrivetrainController::CalculateMotorSpeedWithKeyboard(const char* motor_name) {
        //TODO
        return 0;
    }

    int DrivetrainController::CalculateTranslatingSpeed(const char* motor_name) {
        float angle = GetStickAngle(STICK_DATA("RH"), STICK_DATA("RV"));
        float magnitude = GetStickMagnitude(STICK_DATA("RH"), STICK_DATA("RV"));
        if((motor_name[0] == 'F' && motor_name[1] == 'L') || (motor_name[0] == 'B' && motor_name[1] == 'R')) {
            return (m_MaximumSpeed * magnitude * sin(angle + (PI / 4.0f)));
        } else {
            return (m_MaximumSpeed * magnitude * sin(angle - (PI / 4.0f)));
        }
    }

    float DrivetrainController::GetStickAngle(float xPosition, float yPosition) {
        float angle = 0;

        if(xPosition == 0 && yPosition == 0) return 0;

        if(yPosition == 0) {
            if(xPosition > 0) return 0;
            return (float) (PI);
        }else if(xPosition == 0) {
            if(yPosition > 0) return (float) (PI / 2);
            return (float) (3 * PI / 2);
        }

        angle += atan(yPosition / xPosition);
        if(xPosition > 0 && yPosition > 0) return (float) (angle);
        if(xPosition > 0 && yPosition < 0) return (float) (angle);
        if(xPosition < 0 && yPosition < 0) return (float) (angle + PI);
        if(xPosition < 0 && yPosition > 0) return (float) (angle + PI);
        return (float) (angle);
    }

    float DrivetrainController::GetStickMagnitude(float xPosition, float yPosition) {
        return sqrt((xPosition * xPosition) + (yPosition * yPosition));
    }

    void DrivetrainController::UpdateDrivetrain() {
        m_FLSpeed = CalculateMotorSpeed("FL");
        m_FRSpeed = CalculateMotorSpeed("FR");
        m_BLSpeed = CalculateMotorSpeed("BL");
        m_BRSpeed = CalculateMotorSpeed("BR");
        SetMotorSpeeds();
    }

    void DrivetrainController::SetMotorSpeeds() {
        if(MOTOR_TIMER.execute()){
            SPIN_MOTOR(m_FLSpeed, m_MotorFL);
            SPIN_MOTOR(m_FRSpeed, m_MotorFR);
            SPIN_MOTOR(m_BLSpeed, m_MotorBL);
            SPIN_MOTOR(m_BRSpeed, m_MotorBR);
        }
    }

    void DrivetrainController::StopMotors() {
        m_FLSpeed = 0;
        m_BLSpeed = 0;
        m_FRSpeed = 0;
        m_BRSpeed = 0;
        SetMotorSpeeds();
    }
}