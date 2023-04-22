#include "HardwareHandler.h"

namespace ThornBots {
            HardwareHandler::HardwareHandler() {
                //START CAN1 Motors
                tap::motor::DjiMotor tmp_motor_one = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][0] = &tmp_motor_one;
                tap::motor::DjiMotor tmp_motor_two = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][1] = &tmp_motor_two;
                tap::motor::DjiMotor tmp_motor_three = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][2] = &tmp_motor_three;
                tap::motor::DjiMotor tmp_motor_four = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][3] = &tmp_motor_four;
                tap::motor::DjiMotor tmp_motor_five = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][4] = &tmp_motor_five;
                tap::motor::DjiMotor tmp_motor_six = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][5] = &tmp_motor_six;
                tap::motor::DjiMotor tmp_motor_seven = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][6] = &tmp_motor_seven;
                tap::motor::DjiMotor tmp_motor_eight = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[0][7] = &tmp_motor_eight;
                //STOP CAN1 Motors
                //START CAN2 Motors
                tap::motor::DjiMotor tmp_motor_oneB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][0] = &tmp_motor_oneB;
                tap::motor::DjiMotor tmp_motor_twoB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][1] = &tmp_motor_twoB;
                tap::motor::DjiMotor tmp_motor_threeB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][2] = &tmp_motor_threeB;
                tap::motor::DjiMotor tmp_motor_fourB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][3] = &tmp_motor_fourB;
                tap::motor::DjiMotor tmp_motor_fiveB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][4] = &tmp_motor_fiveB;
                tap::motor::DjiMotor tmp_motor_sixB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][5] = &tmp_motor_sixB;
                tap::motor::DjiMotor tmp_motor_sevenB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][6] = &tmp_motor_sevenB;
                tap::motor::DjiMotor tmp_motor_eightB = tap::motor::DjiMotor(src::DoNotUse_getDrivers(), tap::motor::MotorId::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "pee pee poo poo", 0, 0);
                MotorArray[1][7] = &tmp_motor_eightB;
                //STOP CAN2 Motors   
                           
            }

            bool HardwareHandler::Initialize() {
                //TODO
                //drivers->bmi088.initialize(500, 0.0, 0.0);
                CalibrateIMU();
                //Board::initialize();
                //drivers->remote.initialize();
            }

            void HardwareHandler::UpdateIMU() {
                //TODO
            }

            void HardwareHandler::CalibrateIMU() {
                //TODO
                //drivers->bmi088.requestRecalibration();
            }

            void HardwareHandler::SetMotorPowerOutput(Motor motorID, int output) {
                //TODO
                // motor_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            }

            void HardwareHandler::SetMotorRPMOutput(Motor motorID, int output) {
                //TODO
                //pidController.runControllerDerivateError(motor_one_speed - motor_one.getShaftRPM(), 1);
                //motor_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            }

            void HardwareHandler::PollCanData() {
                //TODO
                //drivers->canRxHandler.pollCanData();
            }

            void HardwareHandler::SendCanData() {
                //TODO
                //drivers->djiMotorTxHandler.encodeAndSendCanData(); //Processes these motor speed changes into can signal
            }

            int32_t HardwareHandler::GetMotorShaftRPM(Motor motorID) {
                //TODO
                //motor_one.getShaftRPM()
            }

            bool HardwareHandler::GetIsDataSent() {
                //TODO //Can we even see this? Taproot doesn't seem to have anything in the CanRxListener Class
            }
 
            float HardwareHandler::GetMotorAngle(Motor motorID) {
                //TODO
            }

            float HardwareHandler::GetIMUAngle() {
                //TODO
            }

            float HardwareHandler::GetMotorAngle(Motor motorID) {
                //float position = tap::motor::DjiMotor::encoderToDegrees(motor_pitch.getEncoderWrapped());
                //TODO: Make this only available for the 6020
            }
};