#ifndef HEALTHENGINE_H
#define HEALTHENGINE_H

#include <Arduino.h>

#include "DataTypes.h"
#include "QRSDetector.h"
#include "HRVProcessor.h"
#include "StatsWindow.h"

struct HealthPacket
{
    uint32_t timestamp;
    float meanHR;
    float meanRR;
    float sdnn;
    float rmssd;
    float meanBR;
    float stdBR;
    float meanTemp;
    float stdTemp;
    float meanAccMag;
    float stdAccMag;
    bool  isWorn;       // true = on-body, false = off-body / lead-off
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

    bool getIsWorn() const;

    String toCSV();

private:

    HealthPacket packet;

    uint32_t lastPacketTime;

    uint32_t stepCounter;

    bool moving;

    bool isWorn;
    bool wasWorn;       // previous state for edge detection
    uint16_t offBodyCount;
    
    QRSDetector qrs;
    HRVProcessor hrv;
    StatsWindow<float, 10> tempStats; // 10s window (1 Hz temp data)
    StatsWindow<float, 100> accStats; // 1s window (100 Hz IMU data)

    // Respiration tracking state
    int32_t lp_z_sum;
    int32_t lp_z_buffer[32];
    int lp_z_ptr;
    bool breathingIn;
    uint32_t lastBreathTime;
    StatsWindow<float, 5> brStats; // 5 breath window

    void detectMotion(const IMUData& imu);
    void detectRespiration(const IMUData& imu, uint32_t timestamp);

};

#endif