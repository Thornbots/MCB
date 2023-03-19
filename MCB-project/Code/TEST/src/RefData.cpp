#include "RefData.h"
namespace ThornBots{
    
    RefData::RefData(){}

    //General
    uint16_t RefData::getRobotHP(){
        return DRIVERS->refSerial.getRobotData().currentHp;
    }

    //Chasis
    float RefData::getPower(){ //in W
        return DRIVERS->refSerial.getRobotData().chassis.power;
    }

    float RefData::getPowerLimit(){ //in W
        return DRIVERS->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }

    //Turret

    uint8_t RefData::getFiringFrequency(){ // in Hz
        return DRIVERS->refSerial.getRobotData().turret.firingFreq;
    }

    uint8_t RefData::getBulletSpeed(){ //in m/s
        return DRIVERS->refSerial.getRobotData().turret.bulletSpeed;
    }

    uint8_t RefData::getYaw(){
        return DRIVERS->refSerial.getRobotData().turret.yaw;
    }

    //Small Shooter
    uint16_t RefData::getSmallShooterHeat(){
        return DRIVERS->refSerial.getRobotData().turret.heat17ID1;
    }

    uint16_t RefData::getSmallShooterCoolingRate(){
        return DRIVERS->refSerial.getRobotData().turret.heatCoolingRate17ID1;
    }

    uint16_t RefData::getSmallShooterHeatLimit(){
        return DRIVERS->refSerial.getRobotData().turret.heatLimit17ID1;
    }

    uint16_t RefData::getSmallShooterBarrelSpeedLimit(){
        return DRIVERS->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;
    }

    //Big Shooter
    uint16_t RefData::getBigShooterHeat(){
        return DRIVERS->refSerial.getRobotData().turret.heat42;
    }

    uint16_t RefData::getBigShooterCoolingRate(){
        return DRIVERS->refSerial.getRobotData().turret.heatCoolingRate42;
    }

    uint16_t RefData::getBigShooterHeatLimit(){
        return DRIVERS->refSerial.getRobotData().turret.heatLimit42;
    }

    uint16_t RefData::getBigShooterBarrelSpeedLimit(){
        return DRIVERS->refSerial.getRobotData().turret.barrelSpeedLimit42;
    }
    
}