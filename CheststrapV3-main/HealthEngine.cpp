#include "HealthEngine.h"

HealthEngine::HealthEngine()
{
    memset(&packet,0,sizeof(packet));

    lastPacketTime = 0;

    stepCounter = 0;

    moving = false;
}

void HealthEngine::update(
    const ECGData& ecg,
    const IMUData& imu,
    const TemperatureData& temp,
    uint32_t timestamp)
{
    detectMotion(imu);

    packet.timestamp = timestamp;

    packet.bodyTemperature = temp.bodyTemperature;

    packet.steps = stepCounter;

    packet.moving = moving;

    // Placeholder values
    packet.heartRate = 72;

    packet.respiration = 16;
}

void HealthEngine::detectMotion(const IMUData& imu)
{
    int32_t magnitude =
        abs(imu.ax) +
        abs(imu.ay) +
        abs(imu.az);

    if(magnitude > 5000)
    {
        moving = true;

        stepCounter++;
    }
    else
    {
        moving = false;
    }
}

const HealthPacket& HealthEngine::getPacket()
{
    return packet;
}

String HealthEngine::toCSV()
{
    String csv;

    csv.reserve(80);

    csv += String(packet.timestamp);
    csv += ",";

    csv += String(packet.heartRate);
    csv += ",";

    csv += String(packet.respiration);
    csv += ",";

    csv += String(packet.bodyTemperature,2);
    csv += ",";

    csv += String(packet.steps);
    csv += ",";

    csv += String(packet.moving);

    return csv;
}