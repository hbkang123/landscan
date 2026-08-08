#include "postprocess_point_cloud_generator.hpp"

#include "app_config.hpp"
#include "postprocess_coordinate_transformer.hpp"
#include "postprocess_interpolator.hpp"

bool PostprocessPointCloudGenerator::generateLocalNedPoints(
    const std::vector<TimedLidarSample>& lidarSamples,
    const std::vector<TimedPositionSample>& rtkSamples,
    const std::vector<TimedAttitudeSample>& attitudeSamples,
    const std::vector<TimedRtkStatusSample>& statusSamples,
    const RuntimeConfig& runtimeConfig,
    std::vector<GeneratedLidarPoint>& generatedPoints,
    TimedPositionSample& referencePosition
) const {
    generatedPoints.clear();
    referencePosition = TimedPositionSample{};

    if(lidarSamples.empty() || rtkSamples.empty() || attitudeSamples.empty() || statusSamples.empty()) {
        return false;
    }

    PostprocessInterpolator interpolator;
    PostprocessCoordinateTransformer transformer;

    if(!transformer.configureLidarOffset(runtimeConfig.lidarOffsetForwardM, runtimeConfig.lidarOffsetRightM, runtimeConfig.lidarOffsetDownM)) {
        return false;
    }

    bool referenceFound = false;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched = interpolator.findLatestRtkStatus(
            rtkSample.timestampUs, 
            statusSamples, 
            AppConfig::RTK_STATUS_MAX_AGE_US, 
            matchedStatus
        );

        if(statusMatched && matchedStatus.positionInfo == 50) {
            referencePosition = rtkSample;
            referenceFound = true;
            break;
        }
    }

    if(!referenceFound) {
        return false;
    }

    generatedPoints.reserve(lidarSamples.size());

    for(const TimedLidarSample& lidarSample : lidarSamples) {
        if(!lidarSample.measurementValid || !lidarSample.timeEstimateReady || lidarSample.timeDiscontinuityDetected) {
            continue;
        }

        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched = interpolator.findLatestRtkStatus(
            lidarSample.timestampUs,
            statusSamples,
            AppConfig::RTK_STATUS_MAX_AGE_US,
            matchedStatus
        );

        if(!statusMatched || matchedStatus.positionInfo != 50) {
            continue;
        }

        InterpolatedPose pose{};

        const bool poseInterpolated = interpolator.interpolatePose(
            lidarSample.timestampUs,
            rtkSamples,
            attitudeSamples,
            AppConfig::POSITION_INTERPOLATION_MAX_GAP_US,
            AppConfig::ATTITUDE_INTERPOLATION_MAX_GAP_US,
            pose
        );

        if(!poseInterpolated) {
            continue;
        }

        CartesianPoint localNedPoint{};

        const bool pointGenerated = transformer.createLocalNedLidarPoint(
            lidarSample,
            pose,
            referencePosition,
            localNedPoint
        );

        if(!pointGenerated) {
            continue;
        }

        GeneratedLidarPoint generatedPoint{};

        generatedPoint.timestampUs = lidarSample.timestampUs;
        generatedPoint.sequenceNumber = lidarSample.sequenceNumber;

        generatedPoint.northM = localNedPoint.xM;
        generatedPoint.eastM = localNedPoint.yM;
        generatedPoint.downM = localNedPoint.zM;

        generatedPoint.angleDeg = lidarSample.angleDeg;
        generatedPoint.distanceM = lidarSample.distanceM;
        generatedPoint.signalStrength = lidarSample.signalStrength;

        generatedPoint.rtkPositionInfo = matchedStatus.positionInfo;

        generatedPoints.push_back(generatedPoint);
    }

    return !generatedPoints.empty();
}