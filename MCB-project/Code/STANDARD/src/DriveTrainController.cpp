#include "DriveTrainController.h"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers_singleton.hpp"


namespace ThornBots
{
DriveTrainController::DriveTrainController(tap::Drivers* m_driver)
{
    // TODO
    this->drivers = m_driver;
    motor_one.initialize();
    motor_two.initialize();
    motor_three.initialize();
    motor_four.initialize();
    // power_limit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
}

DriveTrainController::~DriveTrainController() {
}  // Not going to use this. So look at a cool video of a doggie instead:
   // https://youtu.be/dQw4w9WgXcQ

/**
 * Returns the angle, in radians, that the vector defined by the
 * given x and y coordinates make from the positive x axis. (Ranging from 0 to 2*Pi in the CCW
 * direction)
 */
double DriveTrainController::getAngle(double xPosition, double yPosition)
{
    double angle = 0;

    if(xPosition == 0) {
        if(yPosition == 0) {
            return angle;
        }
        if(yPosition > 0) {
            return angle;
        }
        return (double)(PI/2);
    }
    if(yPosition == 0) {
        if(xPosition > 0) {
            return angle;
        }
        return (double)(3*PI/2);
    }

    angle += atan(yPosition / xPosition);

    if (xPosition > 0 && yPosition > 0)  // Quadrant 1
        return (double)(angle);

    if (xPosition > 0 && yPosition < 0)  // Quadrant 2
        return (double)(angle);

    if (xPosition < 0 && yPosition < 0)  // Quadrant 3
        return (double)(angle + PI);

    if (xPosition < 0 && yPosition > 0)  // Quadrant 4
        return (double)(angle + PI);

    return (double)(angle);
}

/**
 * Returns the hypotenuse gives two sides to a right triangle
 */
double DriveTrainController::getMagnitude(double xPosition, double yPosition)
{
    return sqrt((xPosition * xPosition) + (yPosition * yPosition));
}

/**
 * Given: A double ranging from [-1, 1]
 * Returns: A scaled version of the input.
 * To be used for changing the controlls from a linear control (using just the getMagnitude()) to a
 * quadratic control This simply returns the input squared (to make the small changes even smaller
 * and the values closer to one relatively unchanged) To increase this effect increase two: Look at
 * the following link to see the differences in the different exponent levels
 * https://www.desmos.com/calculator/smghtoozgk
 */
double DriveTrainController::getScaledQuadratic(double magnitude) { return pow(magnitude, 2); }

/**
 * Calls all the necessary methods to set the drive train motors to their appropiate speeds. (Taking
 * into acount turning, WASD or conroller input,  beyblading, and translating)
 */
void DriveTrainController::setMotorValues(
    bool KeyboardAndMouseEnabled,
    bool doBeyblading,
    double right_stick_vert,
    double right_stick_horz,
    double left_stick_vert,
    double left_stick_horz,
    std::string input,
    float yaw_angle,
    bool isRightStickMid,
    int rightSwitchState,
    int leftSwitchValue)
{

    yaw_motor_angle = yaw_angle;
    lockRotation = isRightStickMid;
    lockDrivetrain = false;  // (rightSwitchState == 2);

    bool beyBladeEnabled = false;
    int slew_rate = 4;

    //Check to see if beyblading is enabled and set the beyblading factor
    if (leftSwitchValue == 2 || leftSwitchValue == 1) {
        beyBladeEnabled = true;
        if (leftSwitchValue == 1) {
            beyblading_factor = 0.3;
        } else {
            beyblading_factor = 0.7;
        }
    }

    if (KeyboardAndMouseEnabled)
    {
        //TODO redo all this
        char prev = '\0';
        if (input.length() >= 2)
        {
            prev = input[input.length() - 2];
        }

        switch (input.back())
        {
            case ('w'):
            {
                right_stick_vert = 1;
                switch (prev)
                {
                    case ('a'):
                    {
                        // Go front left
                        right_stick_horz = -1;
                        break;
                    }
                    case ('s'):
                    {
                        // Ignore this, and just make the robot go spinny forward
                        right_stick_horz = 0;
                        break;
                    }
                    case ('d'):
                    {
                        // Go front right
                        right_stick_horz = 1;
                        break;
                    }
                    default:
                    {
                        // No other character, so make it go spinny forward
                        right_stick_horz = 0;
                        break;
                    }
                }
                break;
            }  // End 'w'

            case ('a'):
            {
                right_stick_horz = -1;
                switch (prev)
                {
                    case ('w'):
                    {
                        // Go front left
                        right_stick_vert = 1;
                        break;
                    }
                    case ('s'):
                    {
                        // Go back left
                        right_stick_vert = -1;
                        break;
                    }
                    case ('d'):
                    {
                        // Ignore this, and just make the robot go spinny leftyy
                        right_stick_vert = 0;
                        break;
                    }
                    default:
                    {
                        // No other character, so make it go spinny left
                        right_stick_vert = 0;
                        break;
                    }
                }
                break;
            }  // End 'a'

            case ('s'):
            {
                right_stick_vert = -1;
                switch (prev)
                {
                    case ('a'):
                    {
                        // Go back left
                        right_stick_horz = -1;
                        break;
                    }
                    case ('w'):
                    {
                        // Ignore this, and just make the robot go spinny backward
                        right_stick_horz = 0;
                        break;
                    }
                    case ('d'):
                    {
                        // Go back right
                        right_stick_horz = 1;
                        break;
                    }
                    default:
                    {
                        // No other character, so make it go spinny backward
                        right_stick_horz = 0;
                        break;
                    }
                }
                break;
            }  // End 's'

            case ('d'):
            {
                right_stick_horz = 1;
                switch (prev)
                {
                    case ('a'):
                    {
                        // Ignore this and make it go spinny right
                        right_stick_vert = 0;
                        break;
                    }
                    case ('w'):
                    {
                        // Go front right
                        right_stick_vert = 1;
                        break;
                    }
                    case ('s'):
                    {
                        // Go back right
                        right_stick_vert = -1;
                        break;
                    }
                    default:
                    {
                        // No other character, so make it go spinny right
                        right_stick_vert = 0;
                        break;
                    }
                }
                break;
            }  // End 'd'
            default:
            {
                // No characters, so just do nothing.
                right_stick_horz = 0;
                right_stick_vert = 0;
                break;
            }  // End default

        }
    } else { //keyboard and mouse not enabled
 
    }

    int motor_one_new_speed =   getMotorOneSpeedWithCont( beyBladeEnabled, right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz);
    int motor_two_new_speed =   getMotorTwoSpeedWithCont( beyBladeEnabled, right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz);
    int motor_three_new_speed = getMotorThreeSpeedWithCont( beyBladeEnabled, right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz);
    int motor_four_new_speed =  getMotorFourSpeedWithCont( beyBladeEnabled, right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz);

    motor_one_speed =   updateMotorSpeeds(motor_one_new_speed, motor_one_speed, slew_rate);
    motor_two_speed =   updateMotorSpeeds(motor_two_new_speed, motor_two_speed, slew_rate);
    motor_three_speed = updateMotorSpeeds(motor_three_new_speed, motor_three_speed, slew_rate);
    motor_four_speed =  updateMotorSpeeds(motor_four_new_speed, motor_four_speed, slew_rate);

    /*
    Power Limiting w/Receiver
    power_limit = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    float current_power = drivers->refSerial.getRobotData().chassis.power;
    if(current_power + (.05 * power_limit) > power_limit){
        motor_one_speed = power_limit/(current_power + (.05 * power_limit)) * motor_one_speed;
        motor_two_speed = power_limit/(current_power + (.05 * power_limit)) * motor_two_speed;
        motor_three_speed = power_limit/(current_power + (.05 * power_limit)) * motor_three_speed;
        motor_four_speed = power_limit/(current_power + (.05 * power_limit)) * motor_four_speed;
    }else if(current_power < power_limit){
        motor_one_speed = getMotorOneSpeedWithCont(doBeyblading, right_stick_vert, right_stick_horz,
    left_stick_vert, left_stick_horz); motor_two_speed = getMotorTwoSpeedWithCont(doBeyblading,
    right_stick_vert, right_stick_horz, left_stick_vert, left_stick_horz); motor_three_speed =
    getMotorThreeSpeedWithCont(doBeyblading, right_stick_vert, right_stick_horz, left_stick_vert,
    left_stick_horz); motor_four_speed = getMotorFourSpeedWithCont(doBeyblading, right_stick_vert,
    right_stick_horz, left_stick_vert, left_stick_horz);
    }
    */
}

/* 
* Updates the motor speed by the slew rate
* If the difference between the new speed and the current speed is greater than the slew rate
* then the motor speed is updated by the slew rate
* Otherwise the motor speed is updated to the new speed
*/
double DriveTrainController::updateMotorSpeeds(double MotorNewSpeed, double MotorCurrentSpeed, int slewRate) {
    if (abs(MotorNewSpeed - MotorCurrentSpeed) > slewRate)
    {
        if (MotorNewSpeed > MotorCurrentSpeed) {
            return MotorCurrentSpeed += slewRate;
        }
        else {
            return MotorCurrentSpeed -= slewRate;
        }
    }
        return MotorNewSpeed;
}

/**
 * Tells all the motors to go to their assined speeds
 * i.e. Tells motor_one to to go motor_one_speed and so on.
 * sendMotorTimeout should be the method call sendMotorTimeout.execute() when calling this method
 */
void DriveTrainController::setMotorSpeeds(bool sendMotorTimeout)
{
    if (sendMotorTimeout)
    {
        drivers->canRxHandler.pollCanData();

        // Motor1 (The driver's front wheel)
        pidController.runControllerDerivateError(motor_one_speed - motor_one.getShaftRPM(), 1);
        motor_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor2 (The passenger's front wheel)
        pidController.runControllerDerivateError(motor_two_speed - motor_two.getShaftRPM(), 1);
        motor_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor3 (The driver's back wheel)
        pidController.runControllerDerivateError(motor_three_speed - motor_three.getShaftRPM(), 1);
        motor_three.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor4 (The passenger's back wheel)
        pidController.runControllerDerivateError(motor_four_speed - motor_four.getShaftRPM(), 1);
        motor_four.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        drivers->djiMotorTxHandler
            .encodeAndSendCanData();  // Processes these motor speed changes into can signal
    }                                 // STOP Updating motor speeds
}

/**
 * Tells all of the motors of the drivetrain to go to 0 RPM.
 * We are hard coding this as well as updating the values to just ensure that the motors stop.
 * sendMotorTimeout should be the method call sendMotorTimeout.execute() when calling this method
 */
void DriveTrainController::stopMotors(bool sendMotorTimeout)
{
    motor_one_speed = 0;
    motor_two_speed = 0;
    motor_three_speed = 0;
    motor_four_speed = 0;
    // START Updating motor speeds
    if (sendMotorTimeout)
    {
        drivers->canRxHandler.pollCanData();

        // Motor1 (The driver's front wheel)
        pidController.runControllerDerivateError(0 - motor_one.getShaftRPM(), 1);
        motor_one.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor2 (The passenger's front wheel)
        pidController.runControllerDerivateError(0 - motor_two.getShaftRPM(), 1);
        motor_two.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor3 (The driver's back wheel)
        pidController.runControllerDerivateError(0 - motor_three.getShaftRPM(), 1);
        motor_three.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        // Motor4 (The passenger's back wheel)
        pidController.runControllerDerivateError(0 - motor_four.getShaftRPM(), 1);
        motor_four.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));

        drivers->djiMotorTxHandler
            .encodeAndSendCanData();  // Processes these motor speed changes into can signal
    }                                 // STOP Updating motor speeds
}

/**
 * Returns the speed that the first motor set should go to (the first motor set
 * is the first and fourth motor. Or in other words, the driver's front wheel and the passenger's
 * back wheel) If you want a visual check, it's the blue wheels at
 * https://seamonsters-2605.github.io/archive/mecanum/
 *
 * To move in relation to the front of the vechile (motors 1 and 2 are in the front), the equation
 * for the speed of the MotorSetOne is sin(angle + pi/4) * MaxSpeed * magnitude
 */
int DriveTrainController::getMotorSetOneTranslatingSpeed(double xPosition, double yPosition)
{
    float refined_angle = yaw_motor_angle + REFINED_ANGLE_OFFSET;
    if (refined_angle < 0) refined_angle += 360.0f;
    double angle = getAngle(xPosition, yPosition) + (PI * refined_angle / 180.0f);
    if (lockDrivetrain) angle = yaw_motor_angle;
    double magnitude = getMagnitude(xPosition, yPosition);
    // if (lockDrivetrain) magnitude = 0.5;
    return (max_speed * magnitude * sin(angle + (PI / (double)4.0)));
}

/**
 * Returns the speed that the second motor set should go to (the second motor set
 * is the second and third motor. Or in other words, the driver's back wheel and the passenger's
 * front wheel) If you want a visual intrepretation, it's the red wheels at
 * https://seamonsters-2605.github.io/archive/mecanum/
 */
int DriveTrainController::getMotorSetTwoTranslatingSpeed(double xPosition, double yPosition)
{
    float refined_angle = yaw_motor_angle + REFINED_ANGLE_OFFSET;
    if (refined_angle < 0) refined_angle += 360.0f;
    double angle = getAngle(xPosition, yPosition) + (PI * refined_angle / 180.0f);
    if (lockDrivetrain) angle = yaw_motor_angle;
    double magnitude = getMagnitude(xPosition, yPosition);
    // if (lockDrivetrain) magnitude = 0.5;
    return (max_speed * magnitude * sin(angle - (PI / (double)4.0)));
}

int DriveTrainController::getMotorOneSpeedWithCont(
    bool isBeyblading,
    double right_stick_vert,
    double right_stick_horz,
    double left_stick_vert,
    double left_stick_horz)
{
    if (isBeyblading)
    {
        double tmp = beyblading_factor * max_speed + getMotorSetOneTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
    else
    {
        double rotation_speed = right_stick_horz * max_speed;
        if (lockRotation) rotation_speed = 0;
        double tmp =
            rotation_speed + getMotorSetOneTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
}

int DriveTrainController::getMotorTwoSpeedWithCont(
    bool doBeyblading,
    double right_stick_vert,
    double right_stick_horz,
    double left_stick_vert,
    double left_stick_horz)
{
    if (doBeyblading)
    {
        double tmp = -1 * beyblading_factor * max_speed +
                     getMotorSetTwoTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
    else
    {
        double rotation_speed = -1 * right_stick_horz * max_speed;
        if (lockRotation) rotation_speed = 0;
        double tmp =
            rotation_speed + getMotorSetTwoTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
}

int DriveTrainController::getMotorThreeSpeedWithCont(
    bool doBeyblading,
    double right_stick_vert,
    double right_stick_horz,
    double left_stick_vert,
    double left_stick_horz)
{
    if (doBeyblading)
    {
        double tmp = beyblading_factor * max_speed +
                     getMotorSetTwoTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
    else
    {
        double rotation_speed = right_stick_horz * max_speed;
        if (lockRotation) rotation_speed = 0;
        double tmp =
            rotation_speed + getMotorSetTwoTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
}

int DriveTrainController::getMotorFourSpeedWithCont(
    bool doBeyblading,
    double right_stick_vert,
    double right_stick_horz,
    double left_stick_vert,
    double left_stick_horz)
{
    if (doBeyblading)
    {
        double tmp = -1.0 * beyblading_factor * max_speed +
                     getMotorSetOneTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
    else
    {
        double rotation_speed = -1 * right_stick_horz * max_speed;
        if (lockRotation) rotation_speed = 0;
        double tmp =
            rotation_speed + getMotorSetOneTranslatingSpeed(left_stick_horz, left_stick_vert);
        return ((int)abs(tmp) <= max_speed) ? tmp : ((int)tmp > 0) ? max_speed : -1 * max_speed;
    }
}
}  // namespace ThornBots