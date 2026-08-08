#include "lidar_time_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdint.h>

LidarTimeEstimator::LidarTimeEstimator(
    double nominalRateHz
) {
    if(!configureRateHz(nominalRateHz)) {
        configureRateHz(500.0);
    }
}

bool LidarTimeEstimator::configureRateHz(
    double nominalRateHz
) {
    if(!std::isfinite(nominalRateHz)
        || nominalRateHz < MINIMUM_RATE_HZ
        || nominalRateHz > MAXIMUM_RATE_HZ) {
        return false;
    }

    nominalRateHz_ = nominalRateHz;
    nominalPeriodUs_ = 1000000.0 / nominalRateHz_;
    estimatedPeriodUs_ = nominalPeriodUs_;

    minimumValidPeriodUs_ =
        nominalPeriodUs_ * MINIMUM_PERIOD_RATIO;

    maximumValidPeriodUs_ =
        nominalPeriodUs_ * MAXIMUM_PERIOD_RATIO;

    windowSize_ = static_cast<uint64_t>(
        std::llround(nominalRateHz_ * 0.1)
    );

    if(windowSize_ < 10ULL) {
        windowSize_ = 10ULL;
    }

    const double calculatedThresholdUs =
        nominalPeriodUs_ * 10.0;

    discontinuityThresholdUs_ =
        static_cast<uint64_t>(
            std::llround(
                std::max(20000.0, calculatedThresholdUs)
            )
        );

    reset();
    return true;
}

void LidarTimeEstimator::reset() {
    initialized_ = false;
    ready_ = false;

    estimatedTimeUs_ = 0.0;
    estimatedPeriodUs_ = nominalPeriodUs_;

    lastSequenceNumber_ = 0;
    lastReceiveOsalTimeUs_ = 0;

    windowStartSequence_ = 0;
    windowStartReceiveOsalTimeUs_ = 0;
}

LidarTimeEstimator::Result LidarTimeEstimator::update(
    uint64_t sequenceNumber,
    uint64_t receiveOsalTimeUs
) {
    Result result{};

    if(!initialized_) {
        initialized_ = true;

        estimatedTimeUs_ = std::max(
            0.0,
            static_cast<double>(receiveOsalTimeUs)
                - nominalPeriodUs_
        );

        lastSequenceNumber_ = sequenceNumber;
        lastReceiveOsalTimeUs_ = receiveOsalTimeUs;

        windowStartSequence_ = sequenceNumber;
        windowStartReceiveOsalTimeUs_ =
            receiveOsalTimeUs;

        result.estimatedTimeUs =
            static_cast<uint64_t>(
                std::llround(estimatedTimeUs_)
            );

        result.estimatedPeriodUs = estimatedPeriodUs_;
        result.ready = false;
        result.discontinuityDetected = false;

        return result;
    }

    const bool invalidSequence =
        sequenceNumber <= lastSequenceNumber_;

    const bool skippedSequence =
        !invalidSequence
        && sequenceNumber != lastSequenceNumber_ + 1ULL;

    const bool invalidReceiveTime =
        receiveOsalTimeUs <= lastReceiveOsalTimeUs_;

    bool excessiveReceiveGap = false;

    if(!invalidReceiveTime) {
        excessiveReceiveGap =
            receiveOsalTimeUs - lastReceiveOsalTimeUs_
            > discontinuityThresholdUs_;
    }

    const bool discontinuityDetected =
        invalidSequence
        || skippedSequence
        || invalidReceiveTime
        || excessiveReceiveGap;

    if(discontinuityDetected) {
        estimatedTimeUs_ = std::max(
            0.0,
            static_cast<double>(receiveOsalTimeUs)
                - nominalPeriodUs_
        );

        lastSequenceNumber_ = sequenceNumber;
        lastReceiveOsalTimeUs_ = receiveOsalTimeUs;

        windowStartSequence_ = sequenceNumber;
        windowStartReceiveOsalTimeUs_ =
            receiveOsalTimeUs;

        ready_ = false;

        result.estimatedTimeUs =
            static_cast<uint64_t>(
                std::llround(estimatedTimeUs_)
            );

        result.estimatedPeriodUs = estimatedPeriodUs_;
        result.ready = false;
        result.discontinuityDetected = true;

        return result;
    }

    const uint64_t sequenceDifference =
        sequenceNumber - lastSequenceNumber_;

    estimatedTimeUs_ +=
        estimatedPeriodUs_
        * static_cast<double>(sequenceDifference);

    const uint64_t windowSequenceDifference =
        sequenceNumber - windowStartSequence_;

    if(windowSequenceDifference >= windowSize_) {
        const uint64_t receiveTimeDifference =
            receiveOsalTimeUs
            - windowStartReceiveOsalTimeUs_;

        const double observedPeriodUs =
            static_cast<double>(receiveTimeDifference)
            / static_cast<double>(
                windowSequenceDifference
            );

        if(observedPeriodUs >= minimumValidPeriodUs_
            && observedPeriodUs <= maximumValidPeriodUs_) {

            if(!ready_) {
                estimatedPeriodUs_ = observedPeriodUs;
            } else {
                estimatedPeriodUs_ =
                    estimatedPeriodUs_
                        * (1.0 - PERIOD_SMOOTHING_FACTOR)
                    + observedPeriodUs
                        * PERIOD_SMOOTHING_FACTOR;
            }

            const double targetTimeUs =
                static_cast<double>(receiveOsalTimeUs)
                - nominalPeriodUs_;

            const double phaseErrorUs =
                targetTimeUs - estimatedTimeUs_;

            const double maximumCorrectionUs =
                nominalPeriodUs_
                * MAX_PHASE_CORRECTION_RATIO;

            const double phaseCorrectionUs =
                std::clamp(
                    phaseErrorUs
                        * PHASE_CORRECTION_FACTOR,
                    -maximumCorrectionUs,
                    maximumCorrectionUs
                );

            estimatedTimeUs_ += phaseCorrectionUs;
            ready_ = true;
        }

        windowStartSequence_ = sequenceNumber;
        windowStartReceiveOsalTimeUs_ =
            receiveOsalTimeUs;
    }

    lastSequenceNumber_ = sequenceNumber;
    lastReceiveOsalTimeUs_ = receiveOsalTimeUs;

    result.estimatedTimeUs =
        static_cast<uint64_t>(
            std::llround(estimatedTimeUs_)
        );

    result.estimatedPeriodUs = estimatedPeriodUs_;
    result.ready = ready_;
    result.discontinuityDetected = false;

    return result;
}

double LidarTimeEstimator::getNominalRateHz() const {
    return nominalRateHz_;
}

double LidarTimeEstimator::getEstimatedPeriodUs() const {
    return estimatedPeriodUs_;
}

bool LidarTimeEstimator::isReady() const {
    return ready_;
}