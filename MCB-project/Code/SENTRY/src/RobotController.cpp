#include "RobotController.h"

#include <cmath>

namespace ThornBots
{
double stickLeftHorz, stickLeftVert, stickRightHorz, stickRightVert, stickLeftAngle, stickLeftMagn,
    stickRightAngle, stickRightMagn = 0.0;
double yawEncoderValue, IMUAngle = 0.0;
/*
 * Constructor for RobotController
 */
RobotController::RobotController(
    tap::Drivers* driver,
    ThornBots::DriveTrainController* driveTrainController,
    ThornBots::TurretController* turretController)
{
    this->drivers = driver;
    this->driveTrainController = driveTrainController;
    this->turretController = turretController;
}

void RobotController::initialize()
{
    Board::initialize();
    drivers->can.initialize();
    drivers->bmi088.initialize(500, 0.0, 0.0);
    drivers->bmi088.requestRecalibration();
    drivers->remote.initialize();
    driveTrainController->initialize();
    turretController->initialize();
    modm::delay_ms(
        2500);  // Delay 2.5s to allow the IMU to turn on and get working before we move it around
    // TODO: Finish this (Add creating timers, maybe some code to setup the IMU and make sure it's
    // reading correctly, ect)
}

void RobotController::update()
{

    drivers->canRxHandler.pollCanData();
    updateAllInputVariables();

    toggleKeyboardAndMouse();


    if(drivers->remote.isConnected())
        enableRobot();
    else
        disableRobot();
        

    if (useKeyboardMouse)
    {
        if(robotDisabled) 
            return;
        turretController->enableShooting(); 
        updateWithMouseKeyboard(turretController);
    }
    else
    {
        if(robotDisabled) 
            return;
        turretController->disableShooting();
        updateWithController();
    }


    if (drivers->remote.isConnected())
    {
        if (driveTrainMotorsTimer.execute())
        {
            driveTrainController->setMotorSpeeds();
        }
        if (turretMotorsTimer.execute())
        {
            turretController->setMotorSpeeds();
        }
    }
    else
    {
        turretController->disableShooting();
        stopRobot();
    }

    // drivers->djiMotorTxHandler.encodeAndSendCanData();  // Processes these motor speed changes
    // into can signal
}

void RobotController::stopRobot()
{
    driveTrainController->stopMotors();
    turretController->stopMotors();
    robotDisabled = true;
}

void RobotController::disableRobot()
{
    stopRobot();
    driveTrainController->disable();
    turretController->disable();
}

void RobotController::enableRobot()
{
    robotDisabled = false;
    driveTrainController->enable();
    turretController->enable();
}

void RobotController::updateAllInputVariables()
{
    drivers->remote.read();  // Reading the remote before we check if it is connected yet or not.
    if (IMUTimer.execute())
    {
        drivers->bmi088.periodicIMUUpdate();
    }

    // START Updating stick values
    // Actually Reading from remote
    rightSwitchState =
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
    leftSwitchState =
        drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH);
    left_stick_horz =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    left_stick_vert =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
    right_stick_horz =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    right_stick_vert =
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    // Turning the remote raw values into values we can use more easily (circular cordinates)
    leftStickAngle = getAngle(left_stick_horz, left_stick_vert);
    rightStickAngle = getAngle(right_stick_horz, right_stick_vert);
    leftStickMagnitude = getMagnitude(left_stick_horz, left_stick_vert);
    rightStickMagnitude = getMagnitude(right_stick_horz, right_stick_vert);
    // STOP Updating stick values

    driveTrainRPM = 0;  // TODO: get this. Either power from DT motors, using yaw encoder and IMU,
                        // or something else
    yawRPM = PI / 180 * drivers->bmi088.getGz();
    yawAngleRelativeWorld = PI / 180 * drivers->bmi088.getYaw();

    wheelValue = drivers->remote.getWheel();

    stickLeftHorz = left_stick_horz;
    stickLeftVert = left_stick_vert;
    stickRightHorz = right_stick_horz;
    stickRightVert = right_stick_vert;
    stickLeftAngle = leftStickAngle;
    stickLeftMagn = leftStickMagnitude;
    stickRightAngle = rightStickAngle;
    stickRightMagn = rightStickMagnitude;
}

double RobotController::getAngle(double x, double y)
{
    // error handling to prevent runtime errors in atan2
    if (x == 0 && y == 0)
    {
        return ((double)0.0);
    }

    return atan2(y, x);  // Return (double) [pi, pi] which we want. Doing x/y to rotate the unit
                         // circle 90 degrees CCW (make 0 straight ahead)
}

double RobotController::getMagnitude(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

bool RobotController::toggleKeyboardAndMouse()
{
    // only gets set to false the first time this funtion is called
    static bool hasBeenReleased = true;  

    if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL)
    &&drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT)
    &&drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R))
    {  
        if (hasBeenReleased)
        {
            hasBeenReleased = false;
            useKeyboardMouse = !useKeyboardMouse;
        }
    }
    else
    {
        hasBeenReleased = true;
    }

    return useKeyboardMouse;
}

void RobotController::updateWithController()
{
    if (updateInputTimer.execute())
    {
        double temp = right_stick_horz * YAW_TURNING_PROPORTIONAL;
        driveTrainEncoder = turretController->getYawEncoderValue();

        switch (leftSwitchState)
        {
            case (tap::communication::serial::Remote::SwitchState::UP):
                // Left Switch is up. So need to beyblade at fast speed, and let right stick control
                // turret yaw and pitch
                targetYawAngleWorld += temp;
                targetDTVelocityWorld = (FAST_BEYBLADE_FACTOR * MAX_SPEED);
                yawEncoderCache = driveTrainEncoder;
                break;
            case (tap::communication::serial::Remote::SwitchState::MID):
                targetYawAngleWorld += temp;
                targetDTVelocityWorld = (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
                yawEncoderCache = driveTrainEncoder;
                // Left Switch is mid. So need to beyblade at slow speed, and let right stick
                // control turret yaw and pitch
                break;
            case (tap::communication::serial::Remote::SwitchState::DOWN):
                // Left Switch is down. So need to not beyblade, and let right stick be decided on
                // the right switch value
                switch (rightSwitchState)
                {
                    case (tap::communication::serial::Remote::SwitchState::MID):
                        // Left switch is down, and right is mid. So move turret independently of
                        // drivetrain
                        targetYawAngleWorld += temp;
                        targetDTVelocityWorld = 0;
                        yawEncoderCache = driveTrainEncoder;

                        break;
                    case (tap::communication::serial::Remote::SwitchState::DOWN):
                        yawEncoderCache = 3 * PI / 4;
                    case (tap::communication::serial::Remote::SwitchState::UP):
                        // Left switch is down, and right is up. So driveTrainFollows Turret
                        targetYawAngleWorld =
                            yawAngleRelativeWorld + (yawEncoderCache - driveTrainEncoder);
                        targetDTVelocityWorld = right_stick_horz * MAX_SPEED * TURNING_CONSTANT;
                        break;
                    default:
                        // Should not be in this state. So if we are, just tell robot to do nothing.
                        stopRobot();
                        break;
                }
                break;
            default:
                // Should not be in this state. So if we are, just tell robot to do nothing.
                stopRobot();
                break;
        }
    }
    targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
    driveTrainController->moveDriveTrain(
        targetDTVelocityWorld,
        (leftStickMagnitude * MAX_SPEED),
        driveTrainEncoder + leftStickAngle - 3 * PI / 4);
    turretController->turretMove(
        targetYawAngleWorld,
        0.1 * PI * right_stick_vert - 0.5 * PI,
        driveTrainRPM,
        yawAngleRelativeWorld,
        yawRPM,
        dt);
}

void RobotController::updateWithMouseKeyboard(ThornBots::TurretController* turretController)
{
    if (updateInputTimer.execute())
    {
        //shooting
        if(drivers->remote.getMouseL()){
            turretController->enableIndexer();
        } else {
            turretController->disableIndexer();
        }

        //beyblade
        static bool rHasBeenReleased = true; //r sets fast 
        static bool fHasBeenReleased = true; //f sets slow 
        static bool cHasBeenReleased = true; //c sets off 
        static double currentBeybladeFactor = 0;

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::R)) { 
            if (rHasBeenReleased){
                rHasBeenReleased = false;
                currentBeybladeFactor = FAST_BEYBLADE_FACTOR;
            }
        } else{
            rHasBeenReleased = true;
        }

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::F)){ 
            if (fHasBeenReleased){
                fHasBeenReleased = false;
                currentBeybladeFactor = SLOW_BEYBLADE_FACTOR;
            }
        } else {
            fHasBeenReleased = true;
        }
        
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::C)){ 
            if (cHasBeenReleased){
                cHasBeenReleased = false;
                currentBeybladeFactor = 0;
            }
        } else {
            cHasBeenReleased = true;
        }

        if(currentBeybladeFactor!=0)
            targetDTVelocityWorld = (currentBeybladeFactor * MAX_SPEED);
        else{
            targetDTVelocityWorld=0;
            if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::Q)){ //rotate left
                targetDTVelocityWorld -= (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
            }
            if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::E)){ //rotate right
                targetDTVelocityWorld += (SLOW_BEYBLADE_FACTOR * MAX_SPEED);
            }
        }


        //movement
        int moveHorizonal = 0;
        int moveVertical = 0;

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W))
            moveVertical++;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A))
            moveHorizonal--;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S))
            moveVertical--;
        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D))
            moveHorizonal++;

        double moveAngle = getAngle(moveHorizonal, moveVertical);
        double moveMagnitude = getMagnitude(moveHorizonal, moveVertical);

        if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::SHIFT))  // slow
            moveMagnitude *= SLOW_SPEED;
        else if (drivers->remote.keyPressed(tap::communication::serial::Remote::Key::CTRL))  // fast
            moveMagnitude *= FAST_SPEED;
        else   // medium
            moveMagnitude *= MED_SPEED;
    
        driveTrainEncoder = turretController->getYawEncoderValue();
        yawEncoderCache = driveTrainEncoder;

        driveTrainController->moveDriveTrain(
            targetDTVelocityWorld,
            moveMagnitude,
            driveTrainEncoder + moveAngle - 3 * PI / 4); //driveTrainEncoder + moveAngle - 3 * PI / 4);
            //also try targetYawAngleWorld, yawEncoderCache


        // mouse
        static int mouseXOffset = drivers->remote.getMouseX();
        static int mouseYOffset = drivers->remote.getMouseY();
        int mouseX = drivers->remote.getMouseX() - mouseXOffset;
        int mouseY = drivers->remote.getMouseY() - mouseYOffset;
        static double accumulatedMouseY=0;
        accumulatedMouseY+=mouseY / 10000.0;

        if(accumulatedMouseY>0.4) accumulatedMouseY=0.4;
        if(accumulatedMouseY<-0.3) accumulatedMouseY=-0.3;

        targetYawAngleWorld -= mouseX / 10000.0;

        targetYawAngleWorld = fmod(targetYawAngleWorld, 2 * PI);
        turretController->turretMove(
            targetYawAngleWorld,
            (accumulatedMouseY) - 0.5 * PI,
            driveTrainRPM,
            yawAngleRelativeWorld,
            yawRPM,
            dt);

        
    }
}
}  // namespace ThornBots