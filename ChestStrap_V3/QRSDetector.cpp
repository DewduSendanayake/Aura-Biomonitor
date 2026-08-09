#include "QRSDetector.h"

QRSDetector::QRSDetector() {
    lastPeakTime = 0;
    currentRR = 0;
    hasLastPeak = false;

    lp_ptr = 0;
    lp_sum = 0;
    memset(lp_buffer, 0, sizeof(lp_buffer));
    hp_ptr = 0;
    hp_sum = 0;
    memset(hp_buffer, 0, sizeof(hp_buffer));
    deriv_ptr = 0;
    memset(deriv_buffer, 0, sizeof(deriv_buffer));
    mwi_ptr = 0;
    mwi_sum = 0;
    memset(mwi_buffer, 0, sizeof(mwi_buffer));

    threshold1 = 0;
    threshold2 = 0;
    signalPeak = 0;
    noisePeak = 0;
    missedBeats = 0;
    lastSampleTime = 0;
    peakDetected = false;
    refractoryTimer = 0;
}

int32_t QRSDetector::lowPassFilter(int32_t sample) {
    int32_t out = lp_buffer[lp_ptr];
    lp_sum -= out;
    lp_buffer[lp_ptr] = sample;
    lp_sum += sample;
    lp_ptr = (lp_ptr + 1) % 12;
    return lp_sum;
}

int32_t QRSDetector::highPassFilter(int32_t sample) {
    int32_t out = hp_buffer[hp_ptr];
    hp_sum -= out;
    hp_buffer[hp_ptr] = sample;
    hp_sum += sample;
    hp_ptr = (hp_ptr + 1) % 32;
    return sample - (hp_sum / 32);
}

int32_t QRSDetector::derivative(int32_t sample) {
    int32_t out = (2 * sample) + deriv_buffer[(deriv_ptr + 3) % 4]
                - deriv_buffer[(deriv_ptr + 1) % 4]
                - (2 * deriv_buffer[deriv_ptr]);
    deriv_buffer[deriv_ptr] = sample;
    deriv_ptr = (deriv_ptr + 1) % 4;
    return out / 8;
}

int32_t QRSDetector::movingWindowIntegral(int32_t sample) {
    int32_t out = mwi_buffer[mwi_ptr];
    mwi_sum -= out;
    mwi_buffer[mwi_ptr] = sample;
    mwi_sum += sample;
    mwi_ptr = (mwi_ptr + 1) % MWI_SIZE;
    return mwi_sum / MWI_SIZE;
}

bool QRSDetector::processSample(int32_t sample, uint32_t timestamp) {
    sample = sample / 1024;

    int32_t lp = lowPassFilter(sample);
    int32_t hp = highPassFilter(lp);
    int32_t deriv = derivative(hp);
    int32_t sq = deriv * deriv;
    int32_t mwi = movingWindowIntegral(sq);

    bool rPeak = false;

    if (refractoryTimer > 0) {
        refractoryTimer--;
    } else if (mwi > threshold1) {
        if (!hasLastPeak || timestamp - lastPeakTime > 250) {
            rPeak = true;

            // The first detected peak establishes timing only. It is not an
            // RR interval because there is no preceding heartbeat yet.
            if (hasLastPeak) {
                currentRR = timestamp - lastPeakTime;
            }

            lastPeakTime = timestamp;
            hasLastPeak = true;

            signalPeak = (int32_t)(0.125f * mwi + 0.875f * signalPeak);
            threshold1 = (int32_t)(noisePeak + 0.25f * (signalPeak - noisePeak));
            threshold2 = threshold1 / 2;
            refractoryTimer = 100;
        }
    } else {
        noisePeak = (int32_t)(0.125f * mwi + 0.875f * noisePeak);
        threshold1 = (int32_t)(noisePeak + 0.25f * (signalPeak - noisePeak));
        threshold2 = threshold1 / 2;
    }

    if (signalPeak == 0 && mwi > 1000) {
        signalPeak = mwi;
        threshold1 = signalPeak / 4;
    }

    // Only expose a detection when a real RR interval exists.
    return rPeak && hasLastPeak && currentRR >= 300 && currentRR <= 2000;
}

uint32_t QRSDetector::getLastRR() const {
    return currentRR;
}
