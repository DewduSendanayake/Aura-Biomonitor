#ifndef HEALTHENGINE_H
#define HEALTHENGINE_H

#include <Arduino.h>

#include "DataTypes.h"

struct HealthPacket
{
    uint32_t timestamp;

    uint16_t heartRate;

    uint8_t respiration;

    float bodyTemperature;

    uint32_t steps;

    bool moving;
};

class HealthEngine
{
public:

    HealthEngine();

    void update(
        const ECGData& ecg,
        const IMUData& imu,
        const TemperatureData& temp,
        uint32_t timestamp);

    const HealthPacket& getPacket();

    String toCSV();

private:

    HealthPacket packet;

    uint32_t lastPacketTime;

    uint32_t stepCounter;

    bool moving;

    void detectMotion(const IMUData& imu);

};

#endif