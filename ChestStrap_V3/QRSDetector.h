#ifndef QRSDETECTOR_H
#define QRSDETECTOR_H

#include <Arduino.h>
#include <math.h>

class QRSDetector
{
public:

    QRSDetector();

    // Process one ECG sample.
    // sample    = raw signed ADS1292R CH1 sample
    // timestamp = timestamp in milliseconds
    //
    // Returns true only when a valid RR interval has been detected.
    bool processSample(int32_t sample, uint32_t timestamp);

    // Last accepted RR interval in milliseconds.
    uint32_t getLastRR() const;

    // Reset detector state.
    void reset();

private:

    /************************************************
     * Sampling
     ************************************************/

    static constexpr float SAMPLE_RATE = 500.0f;

    /************************************************
     * ECG Band-pass
     *
     * 5 Hz high-pass
     * 15 Hz low-pass
     ************************************************/

    float hp_x1;
    float hp_x2;
    float hp_y1;
    float hp_y2;

    float lp_x1;
    float lp_x2;
    float lp_y1;
    float lp_y2;

    float bandPassFilter(float sample);

    /************************************************
     * QRS processing
     ************************************************/

    static constexpr int MWI_SIZE = 15;

    float mwiBuffer[MWI_SIZE];
    int mwiIndex;
    float mwiSum;

    float movingWindowIntegral(float sample);

    /************************************************
     * Adaptive threshold
     ************************************************/

    float signalPeak;
    float noisePeak;
    float threshold;

    bool initialized;

    /************************************************
     * Peak detection
     ************************************************/

    float previousMWI;
    float currentMWI;
    float nextMWI;

    uint32_t candidatePeakTime;

    bool aboveThreshold;

    /************************************************
     * R-peak timing
     ************************************************/

    uint32_t lastPeakTime;
    uint32_t currentRR;

    bool hasLastPeak;

    /************************************************
     * Refractory period
     ************************************************/

    uint32_t refractoryUntil;

    static constexpr uint32_t REFRACTORY_MS = 250;

    /************************************************
     * Physiological limits
     ************************************************/

    static constexpr uint32_t MIN_RR_MS = 300;
    static constexpr uint32_t MAX_RR_MS = 2000;

    /************************************************
     * Peak confirmation
     *
     * We wait for the MWI to come back down before
     * confirming the local maximum.
     ************************************************/

    static constexpr uint32_t MAX_PEAK_SEARCH_MS = 120;

    float candidatePeakValue;
    uint32_t candidateStartTime;

    /************************************************
     * Helpers
     ************************************************/

    void updateThreshold(float peakValue, bool isSignal);

    bool acceptPeak(uint32_t peakTime);

    float preprocessSample(int32_t sample);
};

#endif