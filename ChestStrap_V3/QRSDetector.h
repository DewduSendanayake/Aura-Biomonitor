#ifndef QRSDETECTOR_H
#define QRSDETECTOR_H

#include <Arduino.h>

class QRSDetector {
public:
    QRSDetector();

    bool processSample(int32_t sample, uint32_t timestamp);
    uint32_t getLastRR() const;

private:
    uint32_t lastPeakTime;
    uint32_t currentRR;
    bool hasLastPeak;

    int32_t lp_buffer[12];
    int lp_ptr;
    int32_t lp_sum;

    int32_t hp_buffer[32];
    int hp_ptr;
    int32_t hp_sum;

    int32_t deriv_buffer[4];
    int deriv_ptr;

    #define MWI_SIZE 15
    int32_t mwi_buffer[MWI_SIZE];
    int mwi_ptr;
    int32_t mwi_sum;

    int32_t threshold1;
    int32_t threshold2;
    int32_t signalPeak;
    int32_t noisePeak;

    int missedBeats;
    uint32_t lastSampleTime;
    bool peakDetected;
    int refractoryTimer;

    int32_t lowPassFilter(int32_t sample);
    int32_t highPassFilter(int32_t sample);
    int32_t derivative(int32_t sample);
    int32_t movingWindowIntegral(int32_t sample);
};

#endif
