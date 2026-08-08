#include "postprocess_interpolator.hpp"

#include <algorithm>
#include <iterator>
#include <cmath>

bool PostprocessInterpolator::findNearestRtkStatus(uint64_t timestampUs, const std::vector<TimedRtkStatusSample>& samples, uint64_t maximumTimeDifferenceUs, TimedRtkStatusSample& matchedSample) const {
    if(samples.empty()) {
        return false;
    }

    const auto nextIterator = std::lower_bound(samples.begin(), samples.end(), timestampUs,[](const TimedRtkStatusSample& sample, uint64_t value){
        return sample.timestampUs < value;
    });

    const TimedRtkStatusSample* selectedSample = nullptr;
    uint64_t selectedTimeDifferenceUs = 0;

    if(nextIterator == samples.begin()) {
        selectedSample = &(*nextIterator);

        selectedTimeDifferenceUs = nextIterator->timestampUs - timestampUs;
    }
    else if(nextIterator == samples.end()) {
        const auto previousIterator = std::prev(nextIterator);

        selectedSample = &(*previousIterator);

        selectedTimeDifferenceUs = timestampUs - previousIterator->timestampUs;
    }
    else {
        const auto previousIterator = std::prev(nextIterator);

        const uint64_t previousTimeDifferenceUs = timestampUs - previousIterator->timestampUs;

        const uint64_t nextTimeDifferenceUs = nextIterator->timestampUs - timestampUs;

        if(previousTimeDifferenceUs <= nextTimeDifferenceUs) {
            selectedSample = &(*previousIterator);
            selectedTimeDifferenceUs = previousTimeDifferenceUs;
        }
        else {
            selectedSample = &(*nextIterator);
            selectedTimeDifferenceUs = nextTimeDifferenceUs;
        }
    }

    if(selectedSample == nullptr || selectedTimeDifferenceUs > maximumTimeDifferenceUs) {
        return false;
    }

    matchedSample = *selectedSample;
    return true;
}

bool PostprocessInterpolator::findLatestRtkStatus(uint64_t timestampUs, const std::vector<TimedRtkStatusSample>& samples, uint64_t maximumAgeUs, TimedRtkStatusSample& matchedSample) const {
    if(samples.empty()) {
        return false;
    }

    const auto nextIterator = std::upper_bound(
        samples.begin(), 
        samples.end(), 
        timestampUs, 
        [](uint64_t value, const TimedRtkStatusSample sample) { 
            return value < sample.timestampUs;
        }
    );

    if(nextIterator == samples.begin()) {
        return false;
    }

    const auto selectedIterator = std::prev(nextIterator);
    const uint64_t ageUs = timestampUs - selectedIterator->timestampUs;

    if(ageUs > maximumAgeUs) {
        return false;
    }

    matchedSample = *selectedIterator;
    return true;
}

bool PostprocessInterpolator::interpolatePosition(uint64_t timestampUs, const std::vector<TimedPositionSample>& samples, uint64_t maximumSampleGapUs, TimedPositionSample& interpolatedSample) const {
    if(samples.empty()) {
        return false;
    }

    const auto nextIterator = std::lower_bound(
        samples.begin(), 
        samples.end(), 
        timestampUs, 
        [](const TimedPositionSample& sample, uint64_t value){
            return sample.timestampUs < value;
        }
    );

    if(nextIterator != samples.end() && nextIterator->timestampUs == timestampUs) {
        interpolatedSample = *nextIterator;
        return true;
    }

    if(nextIterator == samples.begin() || nextIterator == samples.end()) {
        return false;
    }

    const auto previousIterator = std::prev(nextIterator);

    const uint64_t previousTimeUs = previousIterator->timestampUs;
    const uint64_t nextTimeUs = nextIterator->timestampUs;

    if(nextTimeUs <= previousTimeUs) {
        return false;
    }

    const uint64_t sampleGapUs = nextTimeUs - previousTimeUs;

    if(sampleGapUs > maximumSampleGapUs) {
        return false;
    }

    const double ratio = static_cast<double>(timestampUs - previousTimeUs) / static_cast<double>(sampleGapUs);

    interpolatedSample.timestampUs = timestampUs;

    interpolatedSample.latitudeDeg = previousIterator->latitudeDeg + (nextIterator->latitudeDeg - previousIterator->latitudeDeg) * ratio;
    interpolatedSample.longitudeDeg = previousIterator->longitudeDeg + (nextIterator->longitudeDeg - previousIterator->longitudeDeg) * ratio;
    interpolatedSample.altitudeM = previousIterator->altitudeM + (nextIterator->altitudeM - previousIterator->altitudeM) * ratio;

    if(previousIterator->source == nextIterator->source) {
        interpolatedSample.source = previousIterator->source;
    }
    else{
        interpolatedSample.source = PositionSource::NONE;
    }

    return true;
}

bool PostprocessInterpolator::interpolateAttitude(uint64_t timestampUs, const std::vector<TimedAttitudeSample>& samples, uint64_t maximumSampleGapUs, TimedAttitudeSample& interpolatedSample) const {
    if(samples.empty()) {
        return false;
    }

    const auto nextIterator = std::lower_bound(
        samples.begin(),
        samples.end(),
        timestampUs,
        [](const TimedAttitudeSample& sample, uint64_t value) {
            return sample.timestampUs < value;
        }
    );

    if(nextIterator != samples.end() && nextIterator->timestampUs == timestampUs) {
        interpolatedSample = *nextIterator;

        return true;
    }

    if(nextIterator == samples.begin() || nextIterator == samples.end()) {
        return false;
    }

    const auto previousIterator = std::prev(nextIterator);
    const uint64_t previousTimeUs = previousIterator->timestampUs;
    const uint64_t nextTimeUs = nextIterator->timestampUs;

    if(nextTimeUs <= previousTimeUs) {
        return false;
    }

    const uint64_t sampleGapUs = nextTimeUs - previousTimeUs;

    if(sampleGapUs > maximumSampleGapUs) {
        return false;
    }

    const double ratio = static_cast<double>(timestampUs - previousTimeUs) / static_cast<double>(sampleGapUs);

    double previousW = previousIterator->q0W;
    double previousX = previousIterator->q1X;
    double previousY = previousIterator->q2Y;
    double previousZ = previousIterator->q3Z;

    double nextW = nextIterator->q0W;
    double nextX = nextIterator->q1X;
    double nextY = nextIterator->q2Y;
    double nextZ = nextIterator->q3Z;

    const double previousNorm = std::sqrt(previousW * previousW + previousX * previousX + previousY * previousY + previousZ * previousZ);

    const double nextNorm = std::sqrt(nextW * nextW + nextX * nextX + nextY * nextY + nextZ * nextZ);

    if(previousNorm <= 1.0e-12 || nextNorm <= 1.0e-12) {
        return false;
    }

    previousW /= previousNorm;
    previousX /= previousNorm;
    previousY /= previousNorm;
    previousZ /= previousNorm;

    nextW /= nextNorm;
    nextX /= nextNorm;
    nextY /= nextNorm;
    nextZ /= nextNorm;

    double dot = previousW * nextW + previousX * nextX + previousY * nextY + previousZ * nextZ;

    if(dot < 0.0) {
        nextW = -nextW;
        nextX = -nextX;
        nextY = -nextY;
        nextZ = -nextZ;

        dot = -dot;
    }

    dot = std::clamp(dot, 0.0, 1.0);

    double resultW = 0.0;
    double resultX = 0.0;
    double resultY = 0.0;
    double resultZ = 0.0;

    if(dot > 0.9995) {
        resultW = previousW + (nextW - previousW) * ratio;
        resultX = previousX + (nextX - previousX) * ratio;
        resultY = previousY + (nextY - previousY) * ratio;
        resultZ = previousZ + (nextZ - previousZ) * ratio;
    }
    else{
        const double angle = std::acos(dot);
        const double sinAngle = std::sin(angle);

        if(std::abs(sinAngle) <= 1.0e-12) {
            return false;
        }

        const double previousWeight = std::sin((1.0 - ratio) * angle) / sinAngle;
        const double nextWeight = std::sin(ratio * angle) / sinAngle;

        resultW = previousWeight * previousW + nextWeight * nextW;
        resultX = previousWeight * previousX + nextWeight * nextX;
        resultY = previousWeight * previousY + nextWeight * nextY;
        resultZ = previousWeight * previousZ + nextWeight * nextZ;
    }

    const double resultNorm = std::sqrt(resultW * resultW + resultX * resultX + resultY * resultY + resultZ * resultZ);

    if(resultNorm <= 1.0e-12) {
        return false;
    }

    interpolatedSample.timestampUs = timestampUs;
    interpolatedSample.q0W = resultW / resultNorm;
    interpolatedSample.q1X = resultX / resultNorm;
    interpolatedSample.q2Y = resultY / resultNorm;
    interpolatedSample.q3Z = resultZ / resultNorm;

    return true;
}

bool PostprocessInterpolator::interpolatePose(
    uint64_t timestampUs, 
    const std::vector<TimedPositionSample>& positionSamples, 
    const std::vector<TimedAttitudeSample>& attitudeSamples, 
    uint64_t maximumPositionGapUs, 
    uint64_t maximumAttitudeGapUs, 
    InterpolatedPose& interpolatedPose) const {
    interpolatedPose = InterpolatedPose{};
    interpolatedPose.timestampUs = timestampUs;

    TimedPositionSample positionSample{};
    TimedAttitudeSample attitudeSample{};

    const bool positionInterpolated = interpolatePosition(timestampUs, positionSamples, maximumPositionGapUs, positionSample);
    const bool attitudeInterpolated = interpolateAttitude(timestampUs, attitudeSamples, maximumAttitudeGapUs, attitudeSample);

    interpolatedPose.positionValid = positionInterpolated;
    interpolatedPose.attitudeValid = attitudeInterpolated;

    if(positionInterpolated) {
        interpolatedPose.latitudeDeg = positionSample.latitudeDeg;
        interpolatedPose.longitudeDeg = positionSample.longitudeDeg;
        interpolatedPose.altitudeM = positionSample.altitudeM;
        interpolatedPose.positionSource = positionSample.source;
    }

    if(attitudeInterpolated) {
        interpolatedPose.q0W = attitudeSample.q0W;
        interpolatedPose.q1X = attitudeSample.q1X;
        interpolatedPose.q2Y = attitudeSample.q2Y;
        interpolatedPose.q3Z = attitudeSample.q3Z;
    }

    return positionInterpolated && attitudeInterpolated;
}