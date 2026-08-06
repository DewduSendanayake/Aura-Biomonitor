#include "HRVProcessor.h"

HRVProcessor::HRVProcessor() {
    head = 0;
    count = 0;
    meanRR = 0;
    sdnn = 0;
    rmssd = 0;
    meanHR = 0;
    for(int i = 0; i < HRV_WINDOW_SIZE; i++) {
        rr_buffer[i] = 0.0f;
    }
}

void HRVProcessor::addRRInterval(float rr_ms) {
    if (rr_ms < 300 || rr_ms > 2000) return; // Ignore physiological outliers
    
    rr_buffer[head] = rr_ms;
    head = (head + 1) % HRV_WINDOW_SIZE;
    if (count < HRV_WINDOW_SIZE) count++;
    
    recalculate();
}

void HRVProcessor::recalculate() {
    if (count == 0) return;
    
    float sumRR = 0;
    for (int i = 0; i < count; i++) {
        sumRR += rr_buffer[i];
    }
    meanRR = sumRR / count;
    
    if (meanRR > 0) {
        meanHR = 60000.0f / meanRR;
    }
    
    if (count > 1) {
        float sumSqDiff = 0;
        float sumSuccDiffSq = 0;
        
        for (int i = 0; i < count; i++) {
            float diff = rr_buffer[i] - meanRR;
            sumSqDiff += (diff * diff);
            
            if (i > 0) {
                float succDiff = rr_buffer[i] - rr_buffer[i-1];
                sumSuccDiffSq += (succDiff * succDiff);
            } else if (count == HRV_WINDOW_SIZE) {
                // Wrap around difference
                float succDiff = rr_buffer[0] - rr_buffer[HRV_WINDOW_SIZE-1];
                sumSuccDiffSq += (succDiff * succDiff);
            }
        }
        
        sdnn = sqrt(sumSqDiff / (count - 1));
        rmssd = sqrt(sumSuccDiffSq / (count - 1));
    }
}

float HRVProcessor::getMeanHR() const { return meanHR; }
float HRVProcessor::getMeanRR() const { return meanRR; }
float HRVProcessor::getSDNN() const { return sdnn; }
float HRVProcessor::getRMSSD() const { return rmssd; }

void HRVProcessor::reset() {
    head = 0;
    count = 0;
    meanRR = 0;
    sdnn = 0;
    rmssd = 0;
    meanHR = 0;
    for(int i = 0; i < HRV_WINDOW_SIZE; i++) {
        rr_buffer[i] = 0.0f;
    }
}
