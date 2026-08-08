#pragma once

#include <stdint.h>

class LidarTimeEstimator
{
public:
    struct Result
    {
        uint64_t estimatedTimeUs = 0;
        double estimatedPeriodUs = 0.0;
        bool ready = false;
        bool discontinuityDetected = false;
    };

    explicit LidarTimeEstimator(
        double nominalRateHz = 500.0
    );

    bool configureRateHz(double nominalRateHz);
    void reset();

    Result update(
        uint64_t sequenceNumber,
        uint64_t receiveOsalTimeUs
    );

    double getNominalRateHz() const;
    double getEstimatedPeriodUs() const;
    bool isReady() const;

private:
    bool initialized_ = false;
    bool ready_ = false;

    double nominalRateHz_ = 500.0;
    double nominalPeriodUs_ = 2000.0;
    double estimatedPeriodUs_ = 2000.0;
    double estimatedTimeUs_ = 0.0;

    double minimumValidPeriodUs_ = 1000.0;
    double maximumValidPeriodUs_ = 3000.0;

    uint64_t windowSize_ = 500;
    uint64_t discontinuityThresholdUs_ = 20000;

    uint64_t lastSequenceNumber_ = 0;
    uint64_t lastReceiveOsalTimeUs_ = 0;

    uint64_t windowStartSequence_ = 0;
    uint64_t windowStartReceiveOsalTimeUs_ = 0;

    static constexpr double PERIOD_SMOOTHING_FACTOR = 0.2;
    static constexpr double PHASE_CORRECTION_FACTOR = 0.1;
    static constexpr double MAX_PHASE_CORRECTION_RATIO = 0.1;

    static constexpr double MINIMUM_PERIOD_RATIO = 0.5;
    static constexpr double MAXIMUM_PERIOD_RATIO = 1.5;

    static constexpr double MINIMUM_RATE_HZ = 1.0;
    static constexpr double MAXIMUM_RATE_HZ = 10000.0;
};