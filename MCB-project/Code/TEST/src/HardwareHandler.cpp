#include "HardwareHandler.h"

namespace ThornBots {
    HardwareHandler::HardwareHandler() {
        //START CAN1 Motors
        tap::motor::DjiMotor tmp_motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][0] = &tmp_motor_one;
        tap::motor::DjiMotor tmp_motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][1] = &tmp_motor_two;
        tap::motor::DjiMotor tmp_motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][2] = &tmp_motor_three;
        tap::motor::DjiMotor tmp_motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][3] = &tmp_motor_four;
        tap::motor::DjiMotor tmp_motor_five = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][4] = &tmp_motor_five;
        tap::motor::DjiMotor tmp_motor_six = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][5] = &tmp_motor_six;
        tap::motor::DjiMotor tmp_motor_seven = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][6] = &tmp_motor_seven;
        tap::motor::DjiMotor tmp_motor_eight = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
        MotorArray[0][7] = &tmp_motor_eight;
        //STOP CAN1 Motors
        //START CAN2 Motors
        tap::motor::DjiMotor tmp_motor_oneB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][0] = &tmp_motor_oneB;
        tap::motor::DjiMotor tmp_motor_twoB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR2, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][1] = &tmp_motor_twoB;
        tap::motor::DjiMotor tmp_motor_threeB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR3, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][2] = &tmp_motor_threeB;
        tap::motor::DjiMotor tmp_motor_fourB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][3] = &tmp_motor_fourB;
        tap::motor::DjiMotor tmp_motor_fiveB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR5, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][4] = &tmp_motor_fiveB;
        tap::motor::DjiMotor tmp_motor_sixB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR6, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][5] = &tmp_motor_sixB;
        tap::motor::DjiMotor tmp_motor_sevenB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][6] = &tmp_motor_sevenB;
        tap::motor::DjiMotor tmp_motor_eightB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR8, tap::can::CanBus::CAN_BUS2, false, "pee pee poo poo", 0, 0);
        MotorArray[1][7] = &tmp_motor_eightB;
        //STOP CAN2 Motors   

    }

    int HardwareHandler::getRow(Motor MotorID) {
        return (int) MotorID % 10 == 1 ? 1 : 0;
    }

    int HardwareHandler::getColumn(Motor MotorID) {
        return (int) MotorID - 10 > 0 ? (int) MotorID - 10 : (int) MotorID;
    }

    tap::motor::DjiMotor* HardwareHandler::getMotor(Motor MotorID) {
        return this->MotorArray[getRow(MotorID)][getColumn(MotorID)];
    }

    bool HardwareHandler::Initialize() {
        drivers->bmi088.initialize(500, 0.0, 0.0);
        CalibrateIMU();
        Board::initialize();
        
        //Initializing all of the motors (One that we aren't using (or don't even have plugged in) shouldn't affect anything outside of taking up a bit of extra memory)
        for(int k = 0; k < sizeof(MotorArray), k++;) {
            for(int j = 0; j < sizeof(MotorArray[0]), j++;) {
                MotorArray[k][j]->initialize();
            }
        }

    }

    void HardwareHandler::UpdateIMU() {
        drivers->bmi088.periodicIMUUpdate();
    }

    void HardwareHandler::CalibrateIMU() {
        drivers->bmi088.requestRecalibration();
    }

    void HardwareHandler::SetMotorPowerOutput(Motor MotorID, int32_t Output) {
        getMotor(MotorID)->setDesiredOutput(Output);
    }

    void HardwareHandler::PollCanData() {
        drivers->canRxHandler.pollCanData();
    }

    void HardwareHandler::SendCanData() {
        drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
    }

    int32_t HardwareHandler::GetMotorShaftRPM(Motor MotorID) {
        return getMotor(MotorID)->getShaftRPM();
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
        }
    }

    float HardwareHandler::GetMotorAngle(Motor MotorID) {
        return tap::motor::DjiMotor::encoderToDegrees(getMotor(MotorID)->getEncoderWrapped());               
    }
};