#pragma once

#include "drone_telemetry.hpp"

#include <stdint.h>

struct TimedPositionSample
{
    uint64_t timestampUs = 0;

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;

    PositionSource source = PositionSource::NONE;
};

struct TimedRtkStatusSample
{
    uint64_t timestampUs = 0;
    uint8_t positionInfo = 0;
};

struct TimedAttitudeSample
{
    uint64_t timestampUs = 0;

    double q0W = 1.0;
    double q1X = 0.0;
    double q2Y = 0.0;
    double q3Z = 0.0;
};

struct InterpolatedPose
{
    uint64_t timestampUs = 0;

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;

    double q0W = 1.0;
    double q1X = 0.0;
    double q2Y = 0.0;
    double q3Z = 0.0;

    PositionSource positionSource = PositionSource::NONE;

    bool positionValid = false;
    bool attitudeValid = false;
};

struct TimedLidarSample
{
    uint64_t timestampUs = 0;
    uint64_t receiveTimestampUs = 0;
    uint64_t sequenceNumber = 0;

    double estimatedPeriodUs = 0.0;

    bool timeEstimateReady = false;
    bool timeDiscontinuityDetected = false;

    double angleDeg = 0.0;
    double distanceM = 0.0;
    double signalStrength = 0.0;

    bool measurementValid = false;
};

struct CartesianPoint
{
    double xM = 0.0;
    double yM = 0.0;
    double zM = 0.0;
};

struct GeneratedLidarPoint
{
    uint64_t timestampUs = 0;
    uint64_t sequenceNumber = 0;

    double northM = 0.0;
    double eastM = 0.0;
    double downM = 0.0;

    double angleDeg = 0.0;
    double distanceM = 0.0;
    double signalStrength = 0.0;

    uint8_t rtkPositionInfo = 0;
};

struct ProjectedLidarPoint
{
    uint64_t timestampUs = 0;
    uint64_t sequenceNumber = 0;

    double xM = 0.0;
    double yM = 0.0;
    double elevationM = 0.0;

    double angleDeg = 0.0;
    double distanceM = 0.0;
    double signalStrength = 0.0;

    uint8_t rtkPositionInfo = 0;
};