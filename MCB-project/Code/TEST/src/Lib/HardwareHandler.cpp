#include "HardwareHandler.h"
#include "../Taproot/drivers.hpp"
#include "../Taproot/drivers_singleton.hpp"

namespace ThornBots {
    HardwareHandler::HardwareHandler(src::Drivers *drivers) {
        //Motors on CAN1
        MotorArray[0][0] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 1", 0, 0);
        MotorArray[0][1] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 2", 0, 0);
        MotorArray[0][2] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 3", 0, 0);
        MotorArray[0][3] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 4", 0, 0);
        MotorArray[0][4] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 5", 0, 0);
        MotorArray[0][5] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 6", 0, 0);
        MotorArray[0][6] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 7", 0, 0);
        MotorArray[0][7] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS1, false, "Can 1 motor 8", 0, 0);

        //Motors on CAN2
        MotorArray[1][0] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 1", 0, 0);
        MotorArray[1][1] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 2", 0, 0);
        MotorArray[1][2] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 3", 0, 0);
        MotorArray[1][3] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 4", 0, 0);
        MotorArray[1][4] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 5", 0, 0);
        MotorArray[1][5] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 6", 0, 0);
        MotorArray[1][6] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 7", 0, 0);
        MotorArray[1][7] = new tap::motor::DjiMotor(drivers, tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, false, "Can 2 motor 8", 0, 0);
    }

    int HardwareHandler::GetRow(Motor MotorID) {
        return (int) MotorID % 10 == 1 ? 1 : 0;
    }

    int HardwareHandler::GetColumn(Motor MotorID) {
        return (int) MotorID - 10 > 0 ? (int) MotorID - 10 : (int) MotorID;
    }

    tap::motor::DjiMotor* HardwareHandler::GetMotor(Motor MotorID) {
        return this->MotorArray[GetRow(MotorID)][GetColumn(MotorID)];
    }

    bool HardwareHandler::Initialize() {
        drivers->bmi088.initialize(500, 0.0, 0.0);
        drivers->can.initialize();
        CalibrateIMU();
        
        //Initializing all of the motors (One that we aren't using (or don't even have plugged in) shouldn't affect anything outside of taking up a bit of extra memory)
        int numRows = sizeof(MotorArray) / sizeof(MotorArray[0]); // get the number of rows
        int numCols = sizeof(MotorArray[0]) / sizeof(MotorArray[0][0]);
        for(int k = 0; k < numRows; k++) {
            for(int j = 0; j < numCols; j++) {
                MotorArray[k][j]->initialize();
            }
        }
        m_IsInitialized = true;
        return m_IsInitialized;
    }

    void HardwareHandler::UpdateIMU() {
        drivers->bmi088.periodicIMUUpdate();
    }

    void HardwareHandler::CalibrateIMU() {
        drivers->bmi088.requestRecalibration();
        m_IsImuCalibrated = true;
    }

    void HardwareHandler::SetMotorPowerOutput(Motor MotorID, int32_t Output) {
        GetMotor(MotorID)->setDesiredOutput(Output);
    }

    void HardwareHandler::PollCanData() {
        drivers->canRxHandler.pollCanData();
    }

    void HardwareHandler::SendCanData() {
        drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
    }

    int32_t HardwareHandler::GetMotorShaftRPM(Motor MotorID) {
        return GetMotor(MotorID)->getShaftRPM();
    }

    float HardwareHandler::GetIMUAngle(IMU_Radial Axis) {
        switch(Axis) {
            case(PITCH):
                return drivers->bmi088.getPitch();
                break;
            case(YAW):
                return drivers->bmi088.getYaw();
                break;
            case(ROLL):
                return drivers->bmi088.getRoll();
                break;
            default:
                return 0.0f;
        }
    }

    float HardwareHandler::GetIMUVelocity(IMU_Cardinal Axis) {
        switch(Axis) {
            case(X):
                return drivers->bmi088.getGx();
                break;
            case(Y):
                return drivers->bmi088.getGy();
                break;
            case(Z):
                return drivers->bmi088.getGz();
                break;
            default:
                return 0.0f;
        }
    }

    float HardwareHandler::GetIMUAcceleration(IMU_Cardinal Axis) {
        switch(Axis) {
            case(X):
                return drivers->bmi088.getAx();
                break;
            case(Y):
                return drivers->bmi088.getAy();
                break;
            case(Z):
                return drivers->bmi088.getAz();
                break;
            default:
                return 0.0f;
        }
    }

    float HardwareHandler::GetMotorAngle(Motor MotorID) {
        return tap::motor::DjiMotor::encoderToDegrees(GetMotor(MotorID)->getEncoderWrapped());               
    }
};