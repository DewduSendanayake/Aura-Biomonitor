#include "HealthEngine.h"
#include "Config.h"

/**********************************************************
 * Constants
 **********************************************************/

/*
 * MPU6500 is configured for ±2 g.
 *
 * Therefore:
 *
 * 1 g ≈ 16384 counts
 *
 * We use vector magnitude rather than AZ so that
 * rotation of the chest strap does not change the
 * respiration baseline.
 */

static constexpr float ACC_COUNTS_PER_G = 16384.0f;


/**********************************************************
 * Wear detection
 **********************************************************/

static constexpr int32_t ECG_SATURATION_THRESHOLD = 7500000;

/*
 * ECG is sampled at 500 Hz.
 *
 * 100 samples ≈ 200 ms
 */

static constexpr uint16_t OFF_BODY_LIMIT = 100;

/*
 * Require approximately 20 ms of good signal
 * before declaring the strap worn again.
 */

static constexpr uint16_t ON_BODY_LIMIT = 10;


/**********************************************************
 * Respiration
 **********************************************************/

/*
 * IMU is sampled at 100 Hz.
 */

static constexpr float GRAVITY_ALPHA = 0.02f;

/*
 * Respiration signal smoothing.
 *
 * This is a simple low-pass stage after gravity removal.
 */

static constexpr float RESP_ALPHA = 0.08f;

/*
 * Minimum time between breaths.
 *
 * 1.5 seconds corresponds to 40 breaths/min.
 *
 * This prevents multiple detections from one breath.
 */

static constexpr uint32_t MIN_BREATH_INTERVAL_MS = 1500;

/*
 * Maximum physiologically useful interval.
 *
 * 15 seconds = 4 breaths/min.
 */

static constexpr uint32_t MAX_BREATH_INTERVAL_MS = 15000;


/**********************************************************
 * Constructor
 **********************************************************/

HealthEngine::HealthEngine()
{
    memset(&packet, 0, sizeof(packet));

    lastPacketTime = 0;

    stepCounter = 0;

    moving = false;

    temperatureValid = false;

    /******************************************************
     * Wear state
     ******************************************************/

    isWorn = false;
    wasWorn = false;

    offBodyCount = 500;

    /******************************************************
     * Respiration state
     ******************************************************/

    gravityMagnitude = ACC_COUNTS_PER_G;

    respirationSignal = 0.0f;
    previousRespirationSignal = 0.0f;

    respirationPeak = 0.0f;
    respirationNoise = 0.0f;

    respirationThreshold = 0.0f;

    respirationAboveThreshold = false;

    respirationCandidateTime = 0;
    respirationPeakTime = 0;

    lastBreathTime = 0;

    hasFirstBreath = false;
}


/**********************************************************
 * Reset physiological state
 **********************************************************/

void HealthEngine::resetPhysiologicalState()
{
    /******************************************************
     * ECG detector
     ******************************************************/

    qrs.reset();

    /******************************************************
     * HRV
     ******************************************************/

    hrv.reset();

    /******************************************************
     * Respiration state
     ******************************************************/

    brStats = StatsWindow<float, 5>();

    gravityMagnitude = ACC_COUNTS_PER_G;

    respirationSignal = 0.0f;
    previousRespirationSignal = 0.0f;

    respirationPeak = 0.0f;
    respirationNoise = 0.0f;

    respirationThreshold = 0.0f;

    respirationAboveThreshold = false;

    respirationCandidateTime = 0;
    respirationPeakTime = 0;

    lastBreathTime = 0;

    hasFirstBreath = false;

    /******************************************************
     * Motion / statistics
     ******************************************************/

    moving = false;

    stepCounter = 0;

    tempStats = StatsWindow<float, 10>();
    accStats = StatsWindow<float, 100>();
    temperatureValid = false;
}


/**********************************************************
 * Wear / lead detection
 **********************************************************/

void HealthEngine::updateWearState(
    const ECGData& ecg)
{
    if (!ecg.newData)
        return;

    /*
     * Check ECG saturation.
     */

    bool ecgSaturated =
        (ecg.ch2 > ECG_SATURATION_THRESHOLD) ||
        (ecg.ch2 < -ECG_SATURATION_THRESHOLD);

    /******************************************************
     * Lead OFF
     ******************************************************/

    if (ecg.leadOff)
    {
        if (offBodyCount < 500)
            offBodyCount++;

        if (offBodyCount >= OFF_BODY_LIMIT)
        {
            isWorn = false;
        }
    }

    /******************************************************
     * Lead ON
     ******************************************************/

    else
    {
        /*
         * Saturation is not treated as a complete
         * disconnect because an attached ECG can
         * temporarily saturate during movement.
         */

        if (ecgSaturated)
        {
            if (offBodyCount < 30)
                offBodyCount++;
        }
        else
        {
            if (offBodyCount > 0)
                offBodyCount--;
        }

        if (offBodyCount <= ON_BODY_LIMIT)
        {
            isWorn = true;
        }
    }

    /******************************************************
     * Detect transition from worn → not worn
     ******************************************************/

    if (wasWorn && !isWorn)
    {
        resetPhysiologicalState();
    }

    wasWorn = isWorn;
}


/**********************************************************
 * Main update
 **********************************************************/

void HealthEngine::update(
    const ECGData& ecg,
    const IMUData& imu,
    const TemperatureData& temp,
    uint32_t timestamp)
{
    /******************************************************
     * 1. Update lead / worn status
     ******************************************************/

    updateWearState(ecg);


    /******************************************************
     * 2. ECG / HRV
     *
     * IMPORTANT:
     *
     * Only process ECG for HR when the electrodes are
     * actually connected.
     *
     * This prevents putting-on / removing the strap from
     * generating false RR intervals.
     ******************************************************/

    if (ecg.newData && isWorn && !ecg.leadOff)
    {
        if (qrs.processSample(
                ecg.ch2,
                timestamp))
        {
            uint32_t rr = qrs.getLastRR();

            /*
             * Extra sanity check.
             *
             * QRSDetector already validates this, but
             * keeping this boundary here protects HRV.
             */

            if (rr >= 300 && rr <= 2000)
            {
                hrv.addRRInterval(
                    (float)rr
                );
            }
        }
    }


    /******************************************************
     * 3. IMU
     ******************************************************/

    if (imu.newData)
    {
        detectMotion(imu);

        /*
         * Respiration can only be useful when the strap
         * is worn.
         */

        if (isWorn)
        {
            detectRespiration(
                imu,
                timestamp
            );
        }

        /**************************************************
         * Acceleration magnitude
         *
         * IMPORTANT:
         *
         * Old code used:
         *
         * abs(ax) + abs(ay) + abs(az)
         *
         * which is not acceleration magnitude.
         *
         * We now calculate:
         *
         * sqrt(ax² + ay² + az²)
         *
         * and convert it to g.
         **************************************************/

        float ax = (float)imu.ax;
        float ay = (float)imu.ay;
        float az = (float)imu.az;

        float magnitude =
            sqrt(
                ax * ax +
                ay * ay +
                az * az
            );

        float magnitudeG =
            magnitude / ACC_COUNTS_PER_G;

        accStats.add(magnitudeG);
    }


    /******************************************************
     * 4. Temperature
     ******************************************************/

    if (temp.newData && isWorn)
    {
        if (isfinite(temp.bodyTemperature) &&
            temp.bodyTemperature >= MIN_VALID_TEMP_C &&
            temp.bodyTemperature <= MAX_VALID_TEMP_C)
        {
            tempStats.add(
                temp.bodyTemperature
            );
            temperatureValid = true;
        }
        else
        {
            tempStats = StatsWindow<float, 10>();
            temperatureValid = false;
        }
    }


    /******************************************************
     * 5. Build health packet
     ******************************************************/

    packet.timestamp = timestamp;

    packet.isWorn = isWorn;


    /******************************************************
     * Physiological values
     ******************************************************/

    if (isWorn)
    {
        /**************************************************
         * Heart rate
         **************************************************/

        packet.meanHR =
            hrv.getMeanHR();

        packet.meanRR =
            hrv.getMeanRR();

        packet.sdnn =
            hrv.getSDNN();

        packet.rmssd =
            hrv.getRMSSD();


        /**************************************************
         * Respiration
         **************************************************/

        float meanBreathInterval =
            brStats.getMean();

        if (meanBreathInterval > 0.0f)
        {
            packet.meanBR =
                60000.0f /
                meanBreathInterval;
        }
        else
        {
            packet.meanBR = 0.0f;
        }


        /**************************************************
         * BR standard deviation
         *
         * Convert interval SD into approximate
         * breathing-rate SD using:
         *
         * BR = 60000 / RR
         **************************************************/

        float stdBreathInterval =
            brStats.getStdDev();

        if (meanBreathInterval > 0.0f)
        {
            packet.stdBR =
                (60000.0f /
                 (meanBreathInterval *
                  meanBreathInterval))
                *
                stdBreathInterval;
        }
        else
        {
            packet.stdBR = 0.0f;
        }


        /**************************************************
         * Temperature
         **************************************************/

        if (temperatureValid)
        {
            packet.meanTemp =
                tempStats.getMean();

            packet.stdTemp =
                tempStats.getStdDev();
        }
        else
        {
            packet.meanTemp = NAN;
            packet.stdTemp = NAN;
        }
    }

    else
    {
        /*
         * Never report physiological values when the
         * electrodes are disconnected.
         */

        packet.meanHR = 0.0f;
        packet.meanRR = 0.0f;

        packet.sdnn = 0.0f;
        packet.rmssd = 0.0f;

        packet.meanBR = 0.0f;
        packet.stdBR = 0.0f;

        packet.meanTemp = 0.0f;
        packet.stdTemp = 0.0f;
    }


    /******************************************************
     * Acceleration statistics
     ******************************************************/

    packet.meanAccMag =
        accStats.getMean();

    packet.stdAccMag =
        accStats.getStdDev();
}


/**********************************************************
 * Respiration detection
 **********************************************************/

void HealthEngine::detectRespiration(
    const IMUData& imu,
    uint32_t timestamp)
{
    /******************************************************
     * 1. Calculate acceleration magnitude
     *
     * This removes dependence on sensor orientation.
     *
     * Instead of:
     *
     *     az
     *
     * we use:
     *
     *     sqrt(ax² + ay² + az²)
     ******************************************************/

    float ax = (float)imu.ax;
    float ay = (float)imu.ay;
    float az = (float)imu.az;

    float magnitude =
        sqrt(
            ax * ax +
            ay * ay +
            az * az
        );


    /******************************************************
     * 2. Estimate gravity
     *
     * Gravity changes slowly.
     *
     * Respiration is a smaller dynamic variation around
     * the gravity vector.
     ******************************************************/

    gravityMagnitude =
        gravityMagnitude +
        GRAVITY_ALPHA *
        (magnitude - gravityMagnitude);


    /******************************************************
     * 3. Remove gravity
     ******************************************************/

    float dynamicAcceleration =
        magnitude -
        gravityMagnitude;


    /******************************************************
     * 4. Low-pass respiration signal
     ******************************************************/

    respirationSignal =
        respirationSignal +
        RESP_ALPHA *
        (dynamicAcceleration -
         respirationSignal);


    /******************************************************
     * 5. Estimate signal amplitude
     ******************************************************/

    float absoluteSignal =
        fabs(respirationSignal);


    /******************************************************
     * Adaptive noise estimate
     ******************************************************/

    if (absoluteSignal <
        respirationThreshold)
    {
        respirationNoise =
            0.995f * respirationNoise +
            0.005f * absoluteSignal;
    }


    /******************************************************
     * Adaptive respiration threshold
     ******************************************************/

    float estimatedThreshold =
        respirationNoise * 3.0f;

    /*
     * Minimum threshold prevents tiny sensor noise
     * from being detected as breathing.
     */

    const float MIN_THRESHOLD = 10.0f;

    if (estimatedThreshold < MIN_THRESHOLD)
        estimatedThreshold = MIN_THRESHOLD;

    respirationThreshold =
        estimatedThreshold;


    /******************************************************
     * Positive respiration peak
     ******************************************************/

    bool above =
        respirationSignal >
        respirationThreshold;


    /******************************************************
     * Start of respiration event
     ******************************************************/

    if (above && !respirationAboveThreshold)
    {
        respirationAboveThreshold = true;

        respirationCandidateTime =
            timestamp;

        respirationPeakTime =
            timestamp;

        respirationPeak =
            respirationSignal;
    }


    /******************************************************
     * Continue tracking current respiration peak
     ******************************************************/

    if (above &&
        respirationAboveThreshold)
    {
        if (respirationSignal >
            respirationPeak)
        {
            respirationPeak =
                respirationSignal;

            respirationPeakTime =
                timestamp;
        }
    }


    /******************************************************
     * Respiration event finished
     ******************************************************/

    if (!above &&
        respirationAboveThreshold)
    {
        respirationAboveThreshold = false;

        uint32_t breathTime =
            respirationPeakTime;


        /**************************************************
         * First breath establishes timing.
         **************************************************/

        if (!hasFirstBreath)
        {
            lastBreathTime =
                breathTime;

            hasFirstBreath = true;
        }

        else
        {
            uint32_t interval =
                breathTime -
                lastBreathTime;


            /**********************************************
             * Validate breath interval
             **********************************************/

            if (interval >=
                    MIN_BREATH_INTERVAL_MS &&
                interval <=
                    MAX_BREATH_INTERVAL_MS)
            {
                brStats.add(
                    (float)interval
                );

                lastBreathTime =
                    breathTime;
            }
        }


        respirationPeak = 0.0f;

        respirationCandidateTime = 0;
        respirationPeakTime = 0;
    }


    previousRespirationSignal =
        respirationSignal;
}


/**********************************************************
 * Motion detection
 **********************************************************/

void HealthEngine::detectMotion(
    const IMUData& imu)
{
    float ax = (float)imu.ax;
    float ay = (float)imu.ay;
    float az = (float)imu.az;

    float magnitude =
        sqrt(
            ax * ax +
            ay * ay +
            az * az
        );

    /*
     * Compare magnitude against gravity.
     *
     * A stationary sensor should be approximately
     * 1 g.
     */

    float dynamicMagnitude =
        fabs(
            magnitude -
            ACC_COUNTS_PER_G
        );

    /*
     * Approximately 0.15 g dynamic acceleration.
     *
     * This is only a basic motion indicator, not a
     * medical-grade activity classifier.
     */

    if (dynamicMagnitude >
        (0.15f * ACC_COUNTS_PER_G))
    {
        moving = true;

        stepCounter++;
    }
    else
    {
        moving = false;
    }
}


/**********************************************************
 * Get packet
 **********************************************************/

const HealthPacket&
HealthEngine::getPacket()
{
    return packet;
}


/**********************************************************
 * Get worn status
 **********************************************************/

bool HealthEngine::getIsWorn() const
{
    return isWorn;
}


/**********************************************************
 * CSV output
 **********************************************************/

String HealthEngine::toCSV()
{
    String csv;

    csv.reserve(180);


    /******************************************************
     * Time
     ******************************************************/

    csv += String(packet.timestamp);
    csv += ",";


    /******************************************************
     * HR
     ******************************************************/

    csv += String(packet.meanHR, 2);
    csv += ",";

    csv += String(packet.meanRR, 2);
    csv += ",";

    csv += String(packet.sdnn, 2);
    csv += ",";

    csv += String(packet.rmssd, 2);
    csv += ",";


    /******************************************************
     * Respiration
     ******************************************************/

    csv += String(packet.meanBR, 2);
    csv += ",";

    csv += String(packet.stdBR, 2);
    csv += ",";


    /******************************************************
     * Temperature
     ******************************************************/

    csv += String(packet.meanTemp, 2);
    csv += ",";

    csv += String(packet.stdTemp, 2);
    csv += ",";


    /******************************************************
     * Acceleration
     ******************************************************/

    csv += String(packet.meanAccMag, 2);
    csv += ",";

    csv += String(packet.stdAccMag, 2);
    csv += ",";


    /******************************************************
     * Lead connected
     *
     * 1 = connected
     * 0 = disconnected
     ******************************************************/

    csv += String(
        packet.isWorn ? 1 : 0
    );

    csv += "\n";

    return csv;
}
