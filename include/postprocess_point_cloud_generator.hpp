#pragma once

#include "postprocess_types.hpp"
#include "runtime_config.hpp"

#include <vector>

class PostprocessPointCloudGenerator
{
public:
    bool generateLocalNedPoints(
        const std::vector<TimedLidarSample>& lidarSamples,
        const std::vector<TimedPositionSample>& rtkSamples,
        const std::vector<TimedAttitudeSample>& attitudeSamples,
        const std::vector<TimedRtkStatusSample>& statusSamples,
        const RuntimeConfig& runtimeConfig,
        std::vector<GeneratedLidarPoint>& generatedPoints,
        TimedPositionSample& referencePosition
    ) const;
};