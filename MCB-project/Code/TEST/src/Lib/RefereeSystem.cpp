#include "RefereeSystem.h"

namespace ThornBots{
    RefereeSystem::RefereeSystem(){
        m_isInitialized = true;
    }

    bool RefereeSystem::Initialize(){
        return m_isInitialized;
    }
    //General
    uint16_t RefereeSystem::GetRobotHP(){
        return drivers->refSerial.getRobotData().currentHp;
    }

    bool RefereeSystem::IsBlueTeam(){
        return tap::communication::serial::RefSerial::isBlueTeam(drivers->refSerial.getRobotData().robotId);
    }

    float RefereeSystem::ReceivedDPS(){
        return drivers->refSerial.getRobotData().receivedDps;
    }

    //Chasis
    float RefereeSystem::GetChasisPower(){ //in W
        return drivers->refSerial.getRobotData().chassis.power;
    }

    float RefereeSystem::GetChasisMaxPower(){ //in W
        return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }

    //Turret

    uint8_t RefereeSystem::GetFiringFrequency(){ // in Hz
        return drivers->refSerial.getRobotData().turret.firingFreq;
    }

    uint8_t RefereeSystem::GetBulletSpeed(){ //in m/s
        return drivers->refSerial.getRobotData().turret.bulletSpeed;
    }

    //Small Shooter
    uint16_t RefereeSystem::GetSmallShooterHeat(){
        return drivers->refSerial.getRobotData().turret.heat17ID1;
    }

    uint16_t RefereeSystem::GetSmallShooterCoolingRate(){
        return drivers->refSerial.getRobotData().turret.heatCoolingRate17ID1;
    }

    uint16_t RefereeSystem::GetSmallShooterHeatLimit(){
        return drivers->refSerial.getRobotData().turret.heatLimit17ID1;
    }

    uint16_t RefereeSystem::GetSmallShooterBarrelSpeedLimit(){
        return drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;
    }

    //Big Shooter
    uint16_t RefereeSystem::GetBigShooterHeat(){
        return drivers->refSerial.getRobotData().turret.heat42;
    }

    uint16_t RefereeSystem::GetBigShooterCoolingRate(){
        return drivers->refSerial.getRobotData().turret.heatCoolingRate42;
    }

    uint16_t RefereeSystem::GetBigShooterHeatLimit(){
        return drivers->refSerial.getRobotData().turret.heatLimit42;
    }

    uint16_t RefereeSystem::GetBigShooterBarrelSpeedLimit(){
        return drivers->refSerial.getRobotData().turret.barrelSpeedLimit42;
    }
}