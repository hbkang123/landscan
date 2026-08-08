#pragma once

#include <stdint.h>

struct LidarPoint {
    uint64_t receiveOsalTimeUs = 0;
    uint64_t estimatedMeasurementOsalTimeUs = 0;
    uint64_t sequenceNumber = 0;

    double estimatedPeriodUs = 0.0;
    bool timeEstimateReady = false;
    bool timeDiscontinuityDetected = false;

    float angleDeg = 0.0f;
    float distanceM = 0.0f;
    float signalStrength = 0.0f;
    bool measurementValid = false;
};