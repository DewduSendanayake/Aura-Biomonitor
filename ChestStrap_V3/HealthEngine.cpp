#include "HealthEngine.h"
#include "Config.h"

HealthEngine::HealthEngine()
{
    memset(&packet,0,sizeof(packet));

    lastPacketTime = 0;
    stepCounter = 0;
    moving = false;

    isWorn = false;
    wasWorn = false;
    offBodyCount = 0;
    
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
    // -------------------------------------------------------
    // 1. Determine isWorn status
    //    - ECG lead-off must NOT be detected
    //    - AND temperature must be at or above skin threshold
    // -------------------------------------------------------

    bool ecgContact = !ecg.leadOff;
    bool tempOnBody = (temp.bodyTemperature >= SKIN_TEMP_THRESHOLD);
    bool currentWornState = ecgContact && tempOnBody;

    if (ecg.newData) { // Only evaluate debounce on new ECG samples (250Hz)
        if (currentWornState) {
            offBodyCount = 0;
            isWorn = true;
        } else {
            if (offBodyCount < 250) { // Require 1 second (250 samples) of continuous off-body to toggle
                offBodyCount++;
            } else {
                isWorn = false;
            }
        }
    }

    // Edge detection: just came off body → reset HRV buffer
    if (wasWorn && !isWorn) {
        hrv.reset();
    }
    wasWorn = isWorn;

    // -------------------------------------------------------
    // 2. Process ECG Data for QRS detection (always process to keep filters stable)
    // -------------------------------------------------------

    if (ecg.newData) {
        if (qrs.processSample(ecg.ch1, timestamp)) {
            hrv.addRRInterval((float)qrs.getLastRR());
        }
    }

    // -------------------------------------------------------
    // 3. Process IMU Data (always — useful for diagnostics)
    // -------------------------------------------------------

    if (imu.newData) {
        detectMotion(imu);
        detectRespiration(imu, timestamp);
        
        int32_t magnitude = abs(imu.ax) + abs(imu.ay) + abs(imu.az);
        accStats.add((float)magnitude);
    }

    // -------------------------------------------------------
    // 4. Process Temperature Data (always — real readings)
    // -------------------------------------------------------

    if (temp.newData) {
        tempStats.add(temp.bodyTemperature);
    }

    // -------------------------------------------------------
    // 5. Build packet
    // -------------------------------------------------------

    packet.timestamp = timestamp;
    packet.isWorn = isWorn;

    if (isWorn) {
        // On-body: use real DSP-computed values
        packet.meanHR = hrv.getMeanHR();
        packet.meanRR = hrv.getMeanRR();
        packet.sdnn = hrv.getSDNN();
        packet.rmssd = hrv.getRMSSD();

        // Respiration
        float meanBreathInterval = brStats.getMean();
        if (meanBreathInterval > 0) {
            packet.meanBR = 60000.0f / meanBreathInterval;
        } else {
            packet.meanBR = 0.0f;
        }
        float stdBreathInterval = brStats.getStdDev();
        if (meanBreathInterval > 0) {
            packet.stdBR = (60000.0f / (meanBreathInterval * meanBreathInterval)) * stdBreathInterval;
        } else {
            packet.stdBR = 0.0f;
        }

        // Temperature stats
        packet.meanTemp = tempStats.getMean();
        packet.stdTemp = tempStats.getStdDev();
    } else {
        // Off-body: zero out physiological values and temperature
        packet.meanHR = 0.0f;
        packet.meanRR = 0.0f;
        packet.sdnn = 0.0f;
        packet.rmssd = 0.0f;
        packet.meanBR = 0.0f;
        packet.stdBR = 0.0f;
        packet.meanTemp = 0.0f;
        packet.stdTemp = 0.0f;
    }

    // Accelerometer: always real readings
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

bool HealthEngine::getIsWorn() const
{
    return isWorn;
}

String HealthEngine::toCSV()
{
    String csv;

    csv.reserve(160);

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
    csv += ",";
    csv += String(packet.isWorn ? 1 : 0);
    csv += "\n";

    return csv;
}