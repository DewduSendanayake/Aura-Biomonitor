#include "QRSDetector.h"

/************************************************
 * Constructor
 ************************************************/

QRSDetector::QRSDetector()
{
    reset();
}

/************************************************
 * Reset
 ************************************************/

void QRSDetector::reset()
{
    /**********************************************
     * Filter states
     **********************************************/

    hp_x1 = 0.0f;
    hp_x2 = 0.0f;
    hp_y1 = 0.0f;
    hp_y2 = 0.0f;

    lp_x1 = 0.0f;
    lp_x2 = 0.0f;
    lp_y1 = 0.0f;
    lp_y2 = 0.0f;

    /**********************************************
     * MWI
     **********************************************/

    mwiIndex = 0;
    mwiSum = 0.0f;

    for (int i = 0; i < MWI_SIZE; i++)
    {
        mwiBuffer[i] = 0.0f;
    }

    /**********************************************
     * Threshold
     **********************************************/

    signalPeak = 0.0f;
    noisePeak = 0.0f;
    threshold = 0.0f;

    initialized = false;

    /**********************************************
     * Peak detection
     **********************************************/

    previousMWI = 0.0f;
    currentMWI = 0.0f;
    nextMWI = 0.0f;

    candidatePeakTime = 0;
    candidatePeakValue = 0.0f;
    candidateStartTime = 0;

    aboveThreshold = false;

    /**********************************************
     * RR timing
     **********************************************/

    lastPeakTime = 0;
    currentRR = 0;

    hasLastPeak = false;

    refractoryUntil = 0;
}

/************************************************
 * Band-pass filter
 *
 * Butterworth:
 *
 * High-pass = 5 Hz
 * Low-pass  = 15 Hz
 * Fs        = 500 Hz
 *
 * This removes:
 *
 * - baseline wander
 * - respiration/DC movement
 * - high-frequency noise
 ************************************************/

float QRSDetector::bandPassFilter(float sample)
{
    /**********************************************
     * 2nd order Butterworth 5 Hz HIGH-PASS
     **********************************************/

    const float hp_b0 = 0.95654323f;
    const float hp_b1 = -1.91308645f;
    const float hp_b2 = 0.95654323f;

    const float hp_a1 = -1.91119707f;
    const float hp_a2 = 0.91497583f;

    float hp_y =
        hp_b0 * sample +
        hp_b1 * hp_x1 +
        hp_b2 * hp_x2 -
        hp_a1 * hp_y1 -
        hp_a2 * hp_y2;

    hp_x2 = hp_x1;
    hp_x1 = sample;

    hp_y2 = hp_y1;
    hp_y1 = hp_y;

    /**********************************************
     * 2nd order Butterworth 15 Hz LOW-PASS
     **********************************************/

    const float lp_b0 = 0.00782021f;
    const float lp_b1 = 0.01564042f;
    const float lp_b2 = 0.00782021f;

    const float lp_a1 = -1.73472577f;
    const float lp_a2 = 0.76600660f;

    float lp_y =
        lp_b0 * hp_y +
        lp_b1 * lp_x1 +
        lp_b2 * lp_x2 -
        lp_a1 * lp_y1 -
        lp_a2 * lp_y2;

    lp_x2 = lp_x1;
    lp_x1 = hp_y;

    lp_y2 = lp_y1;
    lp_y1 = lp_y;

    return lp_y;
}

/************************************************
 * Preprocess ECG sample
 ************************************************/

float QRSDetector::preprocessSample(int32_t sample)
{
    /*
     * The ADS1292R produces a 24-bit signed value.
     *
     * Scale it down before floating-point processing.
     *
     * This does NOT affect RR timing because the
     * detector uses adaptive thresholds.
     */

    float x = (float)sample / 1024.0f;

    return bandPassFilter(x);
}

/************************************************
 * Moving Window Integral
 ************************************************/

float QRSDetector::movingWindowIntegral(float sample)
{
    /*
     * Store squared ECG derivative energy.
     */

    float oldValue = mwiBuffer[mwiIndex];

    mwiSum -= oldValue;

    mwiBuffer[mwiIndex] = sample;

    mwiSum += sample;

    mwiIndex++;

    if (mwiIndex >= MWI_SIZE)
        mwiIndex = 0;

    return mwiSum / (float)MWI_SIZE;
}

/************************************************
 * Adaptive threshold update
 ************************************************/

void QRSDetector::updateThreshold(
    float peakValue,
    bool isSignal)
{
    if (peakValue <= 0.0f)
        return;

    if (isSignal)
    {
        /*
         * Signal peak follows actual QRS amplitude.
         */

        if (signalPeak == 0.0f)
        {
            signalPeak = peakValue;
        }
        else
        {
            signalPeak =
                0.125f * peakValue +
                0.875f * signalPeak;
        }
    }
    else
    {
        /*
         * Noise estimate.
         */

        if (noisePeak == 0.0f)
        {
            noisePeak = peakValue;
        }
        else
        {
            noisePeak =
                0.125f * peakValue +
                0.875f * noisePeak;
        }
    }

    /*
     * Standard adaptive threshold concept:
     *
     * threshold = noise + 25% of signal-noise range
     */

    if (signalPeak > noisePeak)
    {
        threshold =
            noisePeak +
            0.25f * (signalPeak - noisePeak);
    }
    else
    {
        threshold = noisePeak;
    }
}

/************************************************
 * Accept R peak
 ************************************************/

bool QRSDetector::acceptPeak(uint32_t peakTime)
{
    /**********************************************
     * First peak
     **********************************************/

    if (!hasLastPeak)
    {
        lastPeakTime = peakTime;

        hasLastPeak = true;

        currentRR = 0;

        refractoryUntil =
            peakTime + REFRACTORY_MS;

        return false;
    }

    /**********************************************
     * Calculate RR
     **********************************************/

    uint32_t rr = peakTime - lastPeakTime;

    /**********************************************
     * Physiological validation
     **********************************************/

    if (rr < MIN_RR_MS)
    {
        /*
         * Too short.
         *
         * This is probably:
         * - T-wave
         * - noise
         * - motion artifact
         * - duplicate detection
         */

        return false;
    }

    if (rr > MAX_RR_MS)
    {
        /*
         * Very long interval.
         *
         * We still update the reference peak because
         * the next heartbeat should be measured from
         * this real peak.
         */

        lastPeakTime = peakTime;

        currentRR = 0;

        refractoryUntil =
            peakTime + REFRACTORY_MS;

        return false;
    }

    /**********************************************
     * Accept RR
     **********************************************/

    currentRR = rr;

    lastPeakTime = peakTime;

    refractoryUntil =
        peakTime + REFRACTORY_MS;

    return true;
}

/************************************************
 * Process ECG sample
 ************************************************/

bool QRSDetector::processSample(
    int32_t sample,
    uint32_t timestamp)
{
    /**********************************************
     * Filter
     **********************************************/

    float filtered = preprocessSample(sample);

    /**********************************************
     * Derivative
     *
     * Simple first derivative.
     **********************************************/

    static float previousFiltered = 0.0f;

    float derivative =
        filtered - previousFiltered;

    previousFiltered = filtered;

    /**********************************************
     * Square
     **********************************************/

    float squared =
        derivative * derivative;

    /**********************************************
     * Moving Window Integration
     **********************************************/

    float mwi =
        movingWindowIntegral(squared);

    /**********************************************
     * Startup period
     *
     * Allow filters and MWI to settle.
     *
     * First ~1 second is used for noise learning.
     **********************************************/

    static uint32_t startupTime = 0;

    if (startupTime == 0)
    {
        startupTime = timestamp;
    }

    uint32_t startupElapsed =
        timestamp - startupTime;

    /**********************************************
     * Learn initial signal/noise level
     **********************************************/

    if (!initialized)
    {
        /*
         * Track the maximum observed MWI during
         * startup.
         */

        if (mwi > signalPeak)
        {
            signalPeak = mwi;
        }

        /*
         * Estimate noise from lower-level samples.
         */

        if (mwi > 0.0f)
        {
            if (noisePeak == 0.0f)
            {
                noisePeak = mwi;
            }
            else
            {
                noisePeak =
                    0.01f * mwi +
                    0.99f * noisePeak;
            }
        }

        if (startupElapsed >= 1000)
        {
            if (signalPeak <= 0.0f)
            {
                signalPeak = noisePeak * 2.0f;
            }

            threshold =
                noisePeak +
                0.25f *
                (signalPeak - noisePeak);

            initialized = true;
        }

        /*
         * Don't detect beats during initialization.
         */

        return false;
    }

    /**********************************************
     * Refractory check
     **********************************************/

    if (timestamp < refractoryUntil)
    {
        /*
         * Still inside refractory period.
         *
         * Do not accept another beat.
         */

        if (mwi > threshold)
        {
            /*
             * It is still useful to track the
             * signal level, but never declare a peak.
             */

            if (mwi > signalPeak)
            {
                updateThreshold(mwi, true);
            }
        }

        return false;
    }

    /**********************************************
     * Local maximum detection
     **********************************************/

    /*
     * We don't declare an R peak merely because
     * MWI crossed the threshold.
     *
     * We first find a local maximum.
     */

    bool result = false;

    /**********************************************
     * Above threshold
     **********************************************/

    if (mwi > threshold)
    {
        if (!aboveThreshold)
        {
            /*
             * Start of QRS candidate.
             */

            aboveThreshold = true;

            candidatePeakValue = mwi;
            candidatePeakTime = timestamp;
            candidateStartTime = timestamp;
        }
        else
        {
            /*
             * Still inside candidate.
             *
             * Keep the highest MWI value.
             */

            if (mwi > candidatePeakValue)
            {
                candidatePeakValue = mwi;
                candidatePeakTime = timestamp;
            }
        }
    }

    /**********************************************
     * Candidate ended
     **********************************************/

    else if (aboveThreshold)
    {
        /*
         * MWI has fallen below threshold.
         *
         * The highest value inside the candidate
         * region is now our QRS peak.
         */

        aboveThreshold = false;

        uint32_t candidateDuration =
            timestamp - candidateStartTime;

        /******************************************
         * Reject extremely long candidates
         ******************************************/

        if (candidateDuration <= MAX_PEAK_SEARCH_MS)
        {
            /*
             * Update signal estimate.
             */

            updateThreshold(
                candidatePeakValue,
                true);

            /****************************************
             * Accept actual R peak
             ****************************************/

            result =
                acceptPeak(candidatePeakTime);
        }
        else
        {
            /*
             * Probably movement / artifact.
             */

            updateThreshold(
                candidatePeakValue,
                false);
        }

        candidatePeakValue = 0.0f;
        candidatePeakTime = 0;
        candidateStartTime = 0;
    }

    /**********************************************
     * Noise estimation
     **********************************************/

    if (!aboveThreshold && mwi > 0.0f)
    {
        /*
         * Only update noise estimate when the signal
         * is below the QRS threshold.
         */

        updateThreshold(
            mwi,
            false);
    }

    /**********************************************
     * Return only when a VALID RR was generated
     **********************************************/

    return result;
}

/************************************************
 * Get last RR
 ************************************************/

uint32_t QRSDetector::getLastRR() const
{
    return currentRR;
}