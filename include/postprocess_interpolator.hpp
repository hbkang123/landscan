#pragma once

#include "postprocess_types.hpp"

#include <stdint.h>
#include <vector>

class PostprocessInterpolator
{
public:
    bool findNearestRtkStatus(uint64_t timestampUs, const std::vector<TimedRtkStatusSample>& samples, uint64_t maximumTimeDifferenceUs, TimedRtkStatusSample& matchedSample) const;
    bool findLatestRtkStatus(uint64_t timestampUs, const std::vector<TimedRtkStatusSample>& samples, uint64_t maximumAgeUs, TimedRtkStatusSample& matchedSample) const;
    bool interpolatePosition(uint64_t timestampUs, const std::vector<TimedPositionSample>& samples, uint64_t maximumSampleGapUs, TimedPositionSample& interpolatedSample) const;
    bool interpolateAttitude(uint64_t timestampUs, const std::vector<TimedAttitudeSample>& samples, uint64_t maximumSampleGapUs, TimedAttitudeSample& interpolatedSample) const;
    bool interpolatePose(uint64_t timestampUs, const std::vector<TimedPositionSample>& positionSamples, const std::vector<TimedAttitudeSample>& attitudeSamples, uint64_t maximumPositionGapUs, uint64_t maximumAttitudeGapUs, InterpolatedPose& interpolatedPose) const;
};