namespace ThornBots{
    class RefData{
    public:
    RefData::RefData();

    //General
    uint16_t getRobotHP();

    //Chasis
    float getPower();

    float getPowerLimit();

    //Turret

    uint8_t getFiringFrequency();

    uint8_t getBulletSpeed();

    uint8_t getYaw();

    //Small Shooter
    uint16_t getSmallShooterHeat();

    uint16_t getSmallShooterCoolingRate();

    uint16_t getSmallShooterHeatLimit();

    uint16_t getSmallShooterBarrelSpeedLimit();

    //Big Shooter
    uint16_t getBigShooterHeat();

    uint16_t getBigShooterCoolingRate();

    uint16_t getBigShooterHeatLimit();

    uint16_t getBigShooterBarrelSpeedLimit();
    }
}