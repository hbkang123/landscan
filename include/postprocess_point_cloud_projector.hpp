#pragma once

#include "postprocess_types.hpp"

#include <vector>

class PostprocessPointCloudProjector
{
public:
    bool projectPoints(
        const std::vector<GeneratedLidarPoint>& localPoints,
        const TimedPositionSample& referencePosition,
        int sourceEpsgCode,
        int targetEpsgCode,
        std::vector<ProjectedLidarPoint>& projectedPoints
    ) const;
};