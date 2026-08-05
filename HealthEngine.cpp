#include "HealthEngine.h"

HealthEngine::HealthEngine()
{
    memset(&packet,0,sizeof(packet));

    lastPacketTime = 0;
    stepCounter = 0;
    moving = false;
    
    lp_z_sum = 0;
    lp_z_ptr = 0;
    memset(lp_z_buffer, 0, sizeof(lp_z_buffer));
    breathingIn = false;
    lastBreathTime = 0;
}

void HealthEngine::update(
    const ECGData& ecg,
    const IMUData& imu,
    const TemperatureData& temp,
    uint32_t timestamp)
{
    // 1. Process ECG Data for QRS detection
    if (ecg.newData) {
        if (qrs.processSample(ecg.ch1, timestamp)) {
            hrv.addRRInterval((float)qrs.getLastRR());
        }
        
        // Remove constness requirement conceptually by assuming newData is handled by caller,
        // or we just process it. We will rely on the scheduler to not call this duplicate times,
        // or we process the same sample (which is bad for DSP).
        // Actually, we process it in `loop()` every iteration. 
        // We MUST only process when newData is true. But `newData` is inside a const struct here.
        // We will assume `ChestStrap_V3.ino` handles newData soon or we just use it if available.
    }

    // 2. Process IMU Data
    if (imu.newData) {
        detectMotion(imu);
        detectRespiration(imu, timestamp);
        
        int32_t magnitude = abs(imu.ax) + abs(imu.ay) + abs(imu.az);
        accStats.add((float)magnitude);
    }

    // 3. Process Temperature Data
    if (temp.newData) {
        tempStats.add(temp.bodyTemperature);
    }

    packet.timestamp = timestamp;

    // Retrieve computed values from DSP
    packet.meanHR = hrv.getMeanHR();
    packet.meanRR = hrv.getMeanRR();
    packet.sdnn = hrv.getSDNN();
    packet.rmssd = hrv.getRMSSD();
    
    // Retrieve computed Respiration
    float meanBreathInterval = brStats.getMean();
    if (meanBreathInterval > 0) {
        packet.meanBR = 60000.0f / meanBreathInterval;
    } else {
        packet.meanBR = 0.0f;
    }
    float stdBreathInterval = brStats.getStdDev();
    if (meanBreathInterval > 0) {
        // Approximate standard deviation of Breathing Rate in BPM
        packet.stdBR = (60000.0f / (meanBreathInterval * meanBreathInterval)) * stdBreathInterval;
    } else {
        packet.stdBR = 0.0f;
    }

    // Retrieve Temperature stats
    packet.meanTemp = tempStats.getMean();
    if (packet.meanTemp == 0.0f) packet.meanTemp = temp.bodyTemperature; // Fallback
    packet.stdTemp = tempStats.getStdDev();

    // Retrieve IMU stats
    packet.meanAccMag = accStats.getMean();
    packet.stdAccMag = accStats.getStdDev();
}

void HealthEngine::detectRespiration(const IMUData& imu, uint32_t timestamp) {
    // Simple 32-sample moving average low-pass filter on Z-axis to extract breathing
    int32_t out = lp_z_buffer[lp_z_ptr];
    lp_z_sum -= out;
    lp_z_buffer[lp_z_ptr] = imu.az;
    lp_z_sum += imu.az;
    lp_z_ptr = (lp_z_ptr + 1) % 32;
    
    int32_t lp_z = lp_z_sum / 32;
    
    // Very basic zero-crossing / threshold detection on the filtered Z-axis
    // Normal gravity on Z is ~16384 for 2G scale. Breathing causes small fluctuations.
    if (lp_z > 16500 && !breathingIn) {
        breathingIn = true;
        if (timestamp - lastBreathTime > 1000) { // min 1 sec between breaths (max 60 bpm)
            brStats.add((float)(timestamp - lastBreathTime));
            lastBreathTime = timestamp;
        }
    } else if (lp_z < 16200 && breathingIn) {
        breathingIn = false;
    }
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

    csv.reserve(150);

    csv += String(packet.timestamp);
    csv += ",";
    csv += String(packet.meanHR, 2);
    csv += ",";
    csv += String(packet.meanRR, 2);
    csv += ",";
    csv += String(packet.sdnn, 2);
    csv += ",";
    csv += String(packet.rmssd, 2);
    csv += ",";
    csv += String(packet.meanBR, 2);
    csv += ",";
    csv += String(packet.stdBR, 2);
    csv += ",";
    csv += String(packet.meanTemp, 2);
    csv += ",";
    csv += String(packet.stdTemp, 2);
    csv += ",";
    csv += String(packet.meanAccMag, 2);
    csv += ",";
    csv += String(packet.stdAccMag, 2);

    return csv;
}