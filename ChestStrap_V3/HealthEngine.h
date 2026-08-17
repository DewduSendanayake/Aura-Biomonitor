#ifndef HEALTHENGINE_H
#define HEALTHENGINE_H

#include <Arduino.h>
#include <math.h>

#include "DataTypes.h"
#include "QRSDetector.h"
#include "HRVProcessor.h"
#include "StatsWindow.h"

/**********************************************************
 * Health Packet
 **********************************************************/

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

    bool isWorn;       // true = lead connected / on body
                       // false = lead disconnected
};

/**********************************************************
 * Health Engine
 **********************************************************/

class HealthEngine
{
public:

    HealthEngine();

    void update(
        const ECGData& ecg,
        const IMUData& imu,
        const TemperatureData& temp,
        uint32_t timestamp
    );

    const HealthPacket& getPacket();

    bool getIsWorn() const;

    String toCSV();

private:

    /******************************************************
     * Output packet
     ******************************************************/

    HealthPacket packet;

    /******************************************************
     * General state
     ******************************************************/

    uint32_t lastPacketTime;

    uint32_t stepCounter;

    bool moving;

    /******************************************************
     * Wear / Lead state
     ******************************************************/

    bool isWorn;
    bool wasWorn;

    uint16_t offBodyCount;

    /******************************************************
     * ECG / HRV
     ******************************************************/

    QRSDetector qrs;
    HRVProcessor hrv;

    /******************************************************
     * Temperature
     *
     * Temperature sample rate = 1 Hz
     * 10 samples = approximately 10 second window
     ******************************************************/

    StatsWindow<float, 10> tempStats;

    bool temperatureValid;

    /******************************************************
     * Acceleration
     *
     * 100 Hz × 100 samples = approximately 1 second
     ******************************************************/

    StatsWindow<float, 100> accStats;

    /******************************************************
     * Respiration
     *
     * Orientation-independent acceleration magnitude
     ******************************************************/

    float gravityMagnitude;

    float respirationSignal;

    float previousRespirationSignal;

    float respirationPeak;

    float respirationNoise;

    float respirationThreshold;

    bool respirationAboveThreshold;

    uint32_t respirationCandidateTime;

    uint32_t respirationPeakTime;

    uint32_t lastBreathTime;

    bool hasFirstBreath;

    /******************************************************
     * Respiration interval statistics
     *
     * Five-breath rolling window
     ******************************************************/

    StatsWindow<float, 5> brStats;

    /******************************************************
     * Motion
     ******************************************************/

    void detectMotion(const IMUData& imu);

    /******************************************************
     * Respiration
     ******************************************************/

    void detectRespiration(
        const IMUData& imu,
        uint32_t timestamp
    );

    /******************************************************
     * Reset physiological processing
     ******************************************************/

    void resetPhysiologicalState();

    /******************************************************
     * Wear detection
     ******************************************************/

    void updateWearState(
        const ECGData& ecg
    );
};

#endif