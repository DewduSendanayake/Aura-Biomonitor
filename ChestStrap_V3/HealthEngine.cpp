#include "HealthEngine.h"
#include "Config.h"
#include <math.h>

HealthEngine::HealthEngine()
{
    memset(&packet, 0, sizeof(packet));
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
    hasLastBreath = false;
    lastBreathTime = 0;
}

void HealthEngine::resetPhysiologicalWindows()
{
    hrv.reset();
    tempStats.reset();
    accStats.reset();
    brStats.reset();

    lp_z_sum = 0;
    lp_z_ptr = 0;
    memset(lp_z_buffer, 0, sizeof(lp_z_buffer));
    breathingIn = false;
    hasLastBreath = false;
    lastBreathTime = 0;
}

void HealthEngine::update(
    const ECGData& ecg,
    const IMUData& imu,
    const TemperatureData& temp,
    uint32_t timestamp)
{
    const int32_t SATURATION_THRESHOLD = 7500000;

    if (ecg.newData) {
        const bool ecgSaturated =
            (ecg.ch1 > SATURATION_THRESHOLD || ecg.ch1 < -SATURATION_THRESHOLD);

        if (!ecg.leadOff) {
            if (ecgSaturated) {
                if (offBodyCount < 30) offBodyCount++;
            } else {
                if (offBodyCount > 0) offBodyCount--;
            }
        } else {
            if (offBodyCount < 500) offBodyCount++;
        }

        if (offBodyCount >= 100) {
            isWorn = false;
        } else if (offBodyCount <= 10) {
            isWorn = true;
        }
    }

    if (wasWorn && !isWorn) {
        resetPhysiologicalWindows();
    }
    wasWorn = isWorn;

    if (ecg.newData) {
        if (qrs.processSample(ecg.ch1, timestamp)) {
            hrv.addRRInterval((float)qrs.getLastRR());
        }
    }

    if (imu.newData) {
        detectMotion(imu);
        detectRespiration(imu, timestamp);

        // MPU6500 is configured for ±2 g, where 16384 counts = 1 g.
        // The previous code used abs(ax)+abs(ay)+abs(az) in raw ADC counts,
        // producing values around 16384 at rest while the model feature is
        // defined as acceleration magnitude in g (around 1.0 at rest).
        const float ax_g = (float)imu.ax / 16384.0f;
        const float ay_g = (float)imu.ay / 16384.0f;
        const float az_g = (float)imu.az / 16384.0f;
        const float accMagnitude = sqrtf(
            ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        accStats.add(accMagnitude);
    }

    if (temp.newData) {
        tempStats.add(temp.bodyTemperature);
    }

    packet.timestamp = timestamp;
    packet.isWorn = isWorn;

    if (isWorn) {
        packet.meanHR = hrv.getMeanHR();
        packet.meanRR = hrv.getMeanRR();
        packet.sdnn = hrv.getSDNN();
        packet.rmssd = hrv.getRMSSD();

        const float meanBreathInterval = brStats.getMean();
        if (meanBreathInterval > 0) {
            packet.meanBR = 60000.0f / meanBreathInterval;
            packet.stdBR =
                (60000.0f / (meanBreathInterval * meanBreathInterval)) *
                brStats.getStdDev();
        } else {
            packet.meanBR = 0.0f;
            packet.stdBR = 0.0f;
        }

        packet.meanTemp = tempStats.getMean();
        packet.stdTemp = tempStats.getStdDev();
    } else {
        packet.meanHR = 0.0f;
        packet.meanRR = 0.0f;
        packet.sdnn = 0.0f;
        packet.rmssd = 0.0f;
        packet.meanBR = 0.0f;
        packet.stdBR = 0.0f;
        packet.meanTemp = 0.0f;
        packet.stdTemp = 0.0f;
        packet.meanAccMag = 0.0f;
        packet.stdAccMag = 0.0f;
        return;
    }

    packet.meanAccMag = accStats.getMean();
    packet.stdAccMag = accStats.getStdDev();
}

void HealthEngine::detectRespiration(const IMUData& imu, uint32_t timestamp)
{
    int32_t out = lp_z_buffer[lp_z_ptr];
    lp_z_sum -= out;
    lp_z_buffer[lp_z_ptr] = imu.az;
    lp_z_sum += imu.az;
    lp_z_ptr = (lp_z_ptr + 1) % 32;

    const int32_t lp_z = lp_z_sum / 32;

    if (lp_z > 16500 && !breathingIn) {
        breathingIn = true;

        if (hasLastBreath && timestamp - lastBreathTime > 1000) {
            brStats.add((float)(timestamp - lastBreathTime));
        }

        lastBreathTime = timestamp;
        hasLastBreath = true;
    } else if (lp_z < 16200 && breathingIn) {
        breathingIn = false;
    }
}

void HealthEngine::detectMotion(const IMUData& imu)
{
    const int32_t magnitude =
        abs(imu.ax) + abs(imu.ay) + abs(imu.az);

    if(magnitude > 5000) {
        moving = true;
        stepCounter++;
    } else {
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
