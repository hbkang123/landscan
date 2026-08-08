#pragma once

#include <cstdint>

enum class PositionSource {
    NONE,
    GNSS,
    RTK
};

struct DroneState {
    bool flightStatusValid = false;
    bool displayModeValid = false;

    uint8_t flightStatus = 0;
    uint8_t displayMode = 0;
};

struct DronePosition {
    bool valid = false;

    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;

    PositionSource source = PositionSource::GNSS;
};

struct DroneAttitude {
    bool valid = false;

    double q0 = 1.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    
    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;
};

struct DroneVelocity {
    bool valid = false;

    float x = 0;
    float y = 0;
    float z = 0;

    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;
};

struct DroneAcceleration {
    bool valid = false;

    float x = 0;
    float y = 0;
    float z = 0;

    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;
};

struct DroneAngularRate {
    bool valid = false;

    float x = 0;
    float y = 0;
    float z = 0;

    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;
};

// struct DroneRtkPosition {
//     bool valid = false;
    
//     double latitude = 0;
//     double longitude = 0;
//     double altitude = 0;

//     uint32_t timestampMs = 0;
//     uint64_t aircraftTimeUs = 0;

//     PositionSource source = PositionSource::RTK;
// };

struct DroneRtkStatus {
    bool valid = false;

    uint8_t positionInfo = 0;

    uint32_t timestampMs = 0;
    uint64_t timestampUs = 0;
    uint64_t aircraftTimeUs = 0;
};

struct DroneTelemetry {
    DroneState state;
    DronePosition position;
    DroneAttitude attitude;
    
    DroneVelocity velocity;
    DroneAcceleration acceleration;
    DroneAngularRate angularRate;
    
    //DroneRtkPosition rtkPosition;
    DronePosition rtkPosition;

    DroneRtkStatus rtkStatus;

    bool velocityOk = false;
    bool accelerationOk = false;
    bool angularRateOk = false;

    uint64_t localUpdatedMs = 0;
};