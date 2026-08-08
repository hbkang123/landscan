#include "app_config.hpp"
#include "postprocess_manager.hpp"
#include "postprocess_data_loader.hpp"
#include "postprocess_types.hpp"
#include "postprocess_interpolator.hpp"
#include "postprocess_coordinate_transformer.hpp"
#include "postprocess_point_cloud_generator.hpp"
#include "postprocess_point_cloud_writer.hpp"
#include "postprocess_point_cloud_projector.hpp"
#include "postprocess_crs_transformer.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>
#include <string>

bool PostprocessManager::validateAttitudeData(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path& attitudePath = sessionDirectory / "attitude.csv";

    PostprocessDataLoader loader;
    std::vector<TimedAttitudeSample> samples;

    const CsvLoadResult result = loader.loadAttitudeSamples(attitudePath.string(), samples);

    if(!result.success) {
        std::cerr << "[Postprocess] Attitude load failed: " << result.errorMessage << std::endl;

        return false;
    }

     std::cout
        << "[Postprocess] Attitude samples loaded"
        << ", loaded=" << result.loadedRowCount
        << ", skipped=" << result.skippedRowCount
        << ", firstTimeUs=" << samples.front().timestampUs
        << ", lastTimeUs=" << samples.back().timestampUs
        << std::endl;

    return true;
}

bool PostprocessManager::validateRtkPositionData(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path rtkPositionPath = sessionDirectory / "position_rtk.csv";

    PostprocessDataLoader loader;
    std::vector<TimedPositionSample> samples;

    const CsvLoadResult result = loader.loadRtkPositionSamples(rtkPositionPath.string(), samples);

    if(!result.success) {
        std::cerr << "[Postprocess] RTK position load failed: " << result.errorMessage << std::endl;

        return false;
    }

    std::cout
        << "[Postprocess] RTK position samples loaded"
        << ", loaded=" << result.loadedRowCount
        << ", skipped=" << result.skippedRowCount
        << ", firstTimeUs=" << samples.front().timestampUs
        << ", lastTimeUs=" << samples.back().timestampUs
        << ", firstLatitude=" << samples.front().latitudeDeg
        << ", firstLongitude=" << samples.front().longitudeDeg
        << ", firstAltitude=" << samples.front().altitudeM
        << std::endl;

    return true;
}

bool PostprocessManager::validateRtkStatusData(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path rtkStatusPath = sessionDirectory / "rtk_status.csv";

    PostprocessDataLoader loader;
    std::vector<TimedRtkStatusSample> samples;

    const CsvLoadResult result = loader.loadRtkStatusSamples(rtkStatusPath.string(), samples);

    if(!result.success) {
        std::cerr << "[Postprocess] RTK status load failed: " << result.errorMessage << std::endl;

        return false;
    }

    size_t unavailableCount = 0;
    size_t floatCount = 0;
    size_t fixedCount = 0;
    size_t otherCount = 0;

    for(const TimedRtkStatusSample& sample : samples) {
        switch(sample.positionInfo) {
            case 0:
                ++unavailableCount;
                break;

            case 34:
                ++floatCount;
                break;

            case 50:
                ++fixedCount;
                break;

            default:
                ++otherCount;
                break;
        }
    }

    std::cout
        << "[Postprocess] RTK status samples loaded"
        << ", loaded=" << result.loadedRowCount
        << ", skipped=" << result.skippedRowCount
        << ", unavailable=" << unavailableCount
        << ", float=" << floatCount
        << ", fixed=" << fixedCount
        << ", other=" << otherCount
        << ", firstTimeUs=" << samples.front().timestampUs
        << ", lastTimeUs=" << samples.back().timestampUs
        << std::endl;

    return true;
}

bool PostprocessManager::validateRtkStatusMatching(
    const std::filesystem::path& sessionDirectory
) const {
    constexpr uint64_t maximumTimeDifferenceUs =
        250000ULL;

    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedPositionSample> positionSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const std::filesystem::path positionPath =
        sessionDirectory / "position_rtk.csv";

    const std::filesystem::path statusPath =
        sessionDirectory / "rtk_status.csv";

    const CsvLoadResult positionResult =
        loader.loadRtkPositionSamples(
            positionPath.string(),
            positionSamples
        );

    if(!positionResult.success) {
        std::cerr
            << "[Postprocess] RTK matching position load failed: "
            << positionResult.errorMessage
            << std::endl;

        return false;
    }

    const CsvLoadResult statusResult =
        loader.loadRtkStatusSamples(
            statusPath.string(),
            statusSamples
        );

    if(!statusResult.success) {
        std::cerr
            << "[Postprocess] RTK matching status load failed: "
            << statusResult.errorMessage
            << std::endl;

        return false;
    }

    size_t matchedCount = 0;
    size_t unmatchedCount = 0;

    size_t unavailableCount = 0;
    size_t floatCount = 0;
    size_t fixedCount = 0;
    size_t otherCount = 0;

    size_t previousCount = 0;
    size_t exactCount = 0;
    size_t nextCount = 0;

    uint64_t maximumObservedDifferenceUs = 0;

    for(const TimedPositionSample& position : positionSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool matched =
            interpolator.findLatestRtkStatus(
                position.timestampUs,
                statusSamples,
                maximumTimeDifferenceUs,
                matchedStatus
            );

        if(!matched) {
            ++unmatchedCount;
            continue;
        }

        ++matchedCount;

        uint64_t timeDifferenceUs = 0;

        if(matchedStatus.timestampUs < position.timestampUs) {
            ++previousCount;

            timeDifferenceUs =
                position.timestampUs
                - matchedStatus.timestampUs;
        }
        else if(matchedStatus.timestampUs
            > position.timestampUs) {

            ++nextCount;

            timeDifferenceUs =
                matchedStatus.timestampUs
                - position.timestampUs;
        }
        else {
            ++exactCount;
        }

        if(timeDifferenceUs > maximumObservedDifferenceUs) {
            maximumObservedDifferenceUs =
                timeDifferenceUs;
        }

        switch(matchedStatus.positionInfo) {
            case 0:
                ++unavailableCount;
                break;

            case 34:
                ++floatCount;
                break;

            case 50:
                ++fixedCount;
                break;

            default:
                ++otherCount;
                break;
        }
    }

    std::cout
        << "[Postprocess] RTK status matching"
        << ", positions=" << positionSamples.size()
        << ", matched=" << matchedCount
        << ", unmatched=" << unmatchedCount
        << ", fixed=" << fixedCount
        << ", float=" << floatCount
        << ", unavailable=" << unavailableCount
        << ", other=" << otherCount
        << ", previous=" << previousCount
        << ", exact=" << exactCount
        << ", next=" << nextCount
        << ", maxDifferenceUs="
        << maximumObservedDifferenceUs
        << std::endl;

    return matchedCount > 0;
}

bool PostprocessManager::validateGpsPositionData(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path gpsPositionPath = sessionDirectory / "position_gps.csv";
    
    PostprocessDataLoader loader;
    std::vector<TimedPositionSample> samples;

    const CsvLoadResult result = loader.loadGpsPositionSamples(gpsPositionPath.string(), samples);

    if(!result.success) {
        std::cerr << "[Postprocess] GPS position load failed: " << result.errorMessage << std::endl;

        return false;
    }

    std::cout
        << "[Postprocess] GPS position samples loaded"
        << ", loaded=" << result.loadedRowCount
        << ", skipped=" << result.skippedRowCount
        << ", firstTimeUs=" << samples.front().timestampUs
        << ", lastTimeUs=" << samples.back().timestampUs
        << ", firstLatitude=" << samples.front().latitudeDeg
        << ", firstLongitude=" << samples.front().longitudeDeg
        << ", firstAltitude=" << samples.front().altitudeM
        << std::endl;

    return true;
}

bool PostprocessManager::validatePositionInterpolation(const std::filesystem::path& sessionDirectory) const {
    constexpr double earthRadiusM = 6378137.0;
    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;

    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedPositionSample> gpsSamples;
    std::vector<TimedPositionSample> rtkSamples;

    const CsvLoadResult gpsResult = loader.loadGpsPositionSamples((sessionDirectory / "position_gps.csv").string(), gpsSamples);

    if(!gpsResult.success) {
        std::cerr << "[Postprocess] Position interpolation GPS load failed: " << gpsResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] Position interpolation RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    size_t interpolatedCount = 0;
    size_t failedCount = 0;

    double horizontalDifferenceSumM = 0.0;
    double maximumHorizontalDifferenceM = 0.0;
    
    double altitudeDifferenceSumM = 0.0;
    double maximumAltitudeDifferenceM = 0.0;

    for(const TimedPositionSample& gpsSample : gpsSamples) {
        TimedPositionSample interpolatedRtkSample{};

        const bool interpolated = interpolator.interpolatePosition(
            gpsSample.timestampUs,
            rtkSamples,
            AppConfig::POSITION_INTERPOLATION_MAX_GAP_US,
            interpolatedRtkSample
        );

        if(!interpolated) {
            ++failedCount;
            continue;
        }

        ++interpolatedCount;

        const double latitudeDifferenceRad = (interpolatedRtkSample.latitudeDeg - gpsSample.latitudeDeg) * degreesToRadians;
        const double longitudeDifferenceRad = (interpolatedRtkSample.longitudeDeg - gpsSample.longitudeDeg) * degreesToRadians;
        const double meanLatitudeRad = ((interpolatedRtkSample.latitudeDeg + gpsSample.latitudeDeg) * 0.5) * degreesToRadians;
        const double northDifferenceM = latitudeDifferenceRad * earthRadiusM;
        const double eastDifferenceM = longitudeDifferenceRad * earthRadiusM * std::cos(meanLatitudeRad);
        const double horizontalDifferenceM = std::hypot(eastDifferenceM, northDifferenceM);
        const double altitudeDifferenceM = std::abs(interpolatedRtkSample.altitudeM - gpsSample.altitudeM);

        horizontalDifferenceSumM += horizontalDifferenceM;
        altitudeDifferenceSumM += altitudeDifferenceM;

        if(horizontalDifferenceM > maximumHorizontalDifferenceM) {
            maximumHorizontalDifferenceM = horizontalDifferenceM;
        }

        if(altitudeDifferenceM > maximumAltitudeDifferenceM) {
            maximumAltitudeDifferenceM = altitudeDifferenceM;
        }
    }

    if(interpolatedCount == 0) {
        std::cerr << "[Postprocess] Position interpolation failed: " << "no GPS timestamps could be interpolated" << std::endl;

        return false;
    }

    const double averageHorizontalDifferenceM = horizontalDifferenceSumM / static_cast<double>(interpolatedCount);
    const double averageAltitudeDifferenceM = altitudeDifferenceSumM / static_cast<double>(interpolatedCount);

    std::cout
        << "[Postprocess] Position interpolation"
        << ", gpsSamples=" << gpsSamples.size()
        << ", rtkSamples=" << rtkSamples.size()
        << ", interpolated=" << interpolatedCount
        << ", failed=" << failedCount
        << ", averageHorizontalDifferenceM="
        << averageHorizontalDifferenceM
        << ", maximumHorizontalDifferenceM="
        << maximumHorizontalDifferenceM
        << ", averageAltitudeDifferenceM="
        << averageAltitudeDifferenceM
        << ", maximumAltitudeDifferenceM="
        << maximumAltitudeDifferenceM
        << std::endl;

    return true;
}

bool PostprocessManager::validateAttitudeInterpolation(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path attitudePath = sessionDirectory / "attitude.csv";

    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedAttitudeSample> attitudeSamples;

    const CsvLoadResult loadResult = loader.loadAttitudeSamples(attitudePath.string(), attitudeSamples);

    if(!loadResult.success) {
        std::cerr << "[Postprocess] Attitude interpolation load failed: " << loadResult.errorMessage << std::endl;

        return false;
    }


    if(attitudeSamples.size() < 2) {
        std::cerr << "[Postprocess] Attitude interpolation failed: " << "at least two attitude samples are required" << std::endl;

        return false;
    }

    size_t interpolatedCount = 0;
    size_t failedCount = 0;
    size_t excessiveGapCount = 0;

    double quaternionNormErrorSum = 0.0;
    double maximumQuaternionNormError = 0.0;
    uint64_t maximumObservedSampleGapUs = 0;

    for(size_t index = 1; index < attitudeSamples.size(); ++index) {
        const TimedAttitudeSample& previousSample = attitudeSamples[index -1];
        const TimedAttitudeSample& nextSample = attitudeSamples[index];

        if(nextSample.timestampUs <= previousSample.timestampUs) {
            ++failedCount;
            continue;
        }

        const uint64_t sampleGapUs = nextSample.timestampUs - previousSample.timestampUs;

        if(sampleGapUs > maximumObservedSampleGapUs) {
            maximumObservedSampleGapUs = sampleGapUs;
        }

        if(sampleGapUs > AppConfig::ATTITUDE_INTERPOLATION_MAX_GAP_US) {
            ++excessiveGapCount;
            continue;
        }

        const uint64_t middleTimestampUs = previousSample.timestampUs + sampleGapUs / 2ULL;

        TimedAttitudeSample interpolatedSample{};

        const bool interpolated = interpolator.interpolateAttitude(middleTimestampUs, attitudeSamples, AppConfig::ATTITUDE_INTERPOLATION_MAX_GAP_US, interpolatedSample);

        if(!interpolated) {
            ++failedCount;
            continue;
        }

        ++interpolatedCount;

        const double quaternionNorm = std::sqrt(
            interpolatedSample.q0W * interpolatedSample.q0W + 
            interpolatedSample.q1X * interpolatedSample.q1X + 
            interpolatedSample.q2Y * interpolatedSample.q2Y + 
            interpolatedSample.q3Z * interpolatedSample.q3Z
        );

        const double quaternionNormError = std::abs(quaternionNorm - 1.0);

        quaternionNormErrorSum += quaternionNormError;

        if(quaternionNormError > maximumQuaternionNormError) {
            maximumQuaternionNormError = quaternionNormError;
        }
    }

    if(interpolatedCount == 0) {
        std::cerr << "[Postprocess] Attitude interpolation failed: " << "no attitude intervals could be interpolated" << std::endl;

        return false;
    }

    const double averageQuaternionNormError = quaternionNormErrorSum / static_cast<double>(interpolatedCount);

    std::cout
        << "[Postprocess] Attitude interpolation"
        << ", samples=" << attitudeSamples.size()
        << ", interpolated=" << interpolatedCount
        << ", failed=" << failedCount
        << ", excessiveGap=" << excessiveGapCount
        << ", maximumObservedSampleGapUs="
        << maximumObservedSampleGapUs
        << ", averageQuaternionNormError="
        << averageQuaternionNormError
        << ", maximumQuaternionNormError="
        << maximumQuaternionNormError
        << std::endl;

    return failedCount == 0;
}

bool PostprocessManager::validatePoseInterpolation(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedAttitudeSample> attitudeSamples;

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] Pose interpolation RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult attitudeResult = loader.loadAttitudeSamples((sessionDirectory / "attitude.csv").string(), attitudeSamples);

    if(!attitudeResult.success) {
        std::cerr << "[Postprocess] Pose interpolation attitude load failed: " << attitudeResult.errorMessage << std::endl;

        return false;
    }

    size_t poseValidCount = 0;
    size_t positionInvalidCount = 0;
    size_t attitudeInvalidCount = 0;
    size_t bothInvalidCount = 0;

    double maximumQuaternionNormError = 0.0;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        InterpolatedPose interpolatedPose{};

        const bool poseValid = interpolator.interpolatePose(
            rtkSample.timestampUs,
            rtkSamples,
            attitudeSamples,
            AppConfig::POSITION_INTERPOLATION_MAX_GAP_US,
            AppConfig::ATTITUDE_INTERPOLATION_MAX_GAP_US,
            interpolatedPose
        );

        if(!interpolatedPose.positionValid && !interpolatedPose.attitudeValid) {
            ++bothInvalidCount;
            continue;
        }

        if(!interpolatedPose.positionValid) {
            ++positionInvalidCount;
            continue;
        }

        if(!interpolatedPose.attitudeValid) {
            ++attitudeInvalidCount;
            continue;
        }

        if(!poseValid) {
            ++bothInvalidCount;
            continue;
        }

        ++poseValidCount;

        const double quaternionNorm = std::sqrt(
            interpolatedPose.q0W * interpolatedPose.q0W
            + interpolatedPose.q1X * interpolatedPose.q1X
            + interpolatedPose.q2Y * interpolatedPose.q2Y
            + interpolatedPose.q3Z * interpolatedPose.q3Z
        );
        
        const double quaternionNormError = std::abs(quaternionNorm - 1.0);

        if(quaternionNormError > maximumQuaternionNormError) {
            maximumQuaternionNormError = quaternionNormError;
        }
    }

    std::cout
        << "[Postprocess] Pose interpolation"
        << ", targetTimes=" << rtkSamples.size()
        << ", poseValid=" << poseValidCount
        << ", positionInvalid=" << positionInvalidCount
        << ", attitudeInvalid=" << attitudeInvalidCount
        << ", bothInvalid=" << bothInvalidCount
        << ", maximumQuaternionNormError="
        << maximumQuaternionNormError
        << std::endl;

    return poseValidCount > 0;
}

bool PostprocessManager::validateLidarData(const std::filesystem::path& sessionDirectory) const {
    const std::filesystem::path lidarPath = sessionDirectory / "lidar.csv";

    PostprocessDataLoader loader;
    std::vector<TimedLidarSample> samples;

    const CsvLoadResult loadResult = loader.loadLidarSamples(lidarPath.string(), samples);

    if(!loadResult.success) {
        std::cerr << "[Postprocess] LiDAR load failed: " << loadResult.errorMessage << std::endl;

        return false;
    }

    size_t validMeasurementCount = 0;
    size_t invalidMeasurementCount = 0;

    size_t timeEstimateReadyCount = 0;
    size_t timeDiscontinuityCount = 0;

    size_t timeBackwardsCount = 0;
    size_t duplicateTimestampCount = 0;
    size_t angleOutsideConfiguredRangeCount = 0;

    size_t duplicateSequenceCount = 0;
    size_t backwardsSequenceCount = 0;
    uint64_t missingSequenceCount = 0;

    uint64_t positiveIntervalSumUs = 0;
    size_t positiveIntervalCount = 0;

    constexpr double angleToleranceDeg = 1.0;

    for(size_t index = 0; index < samples.size(); ++index) {
        const TimedLidarSample& sample = samples[index];

        if(sample.angleDeg < runtimeConfig_.lidarScanLowAngleDeg - angleToleranceDeg
            || sample.angleDeg > runtimeConfig_.lidarScanHighAngleDeg + angleToleranceDeg) {
            ++angleOutsideConfiguredRangeCount;
        }

        if(sample.measurementValid) {
            ++validMeasurementCount;
        }
        else {
            ++invalidMeasurementCount;
        }

        if(sample.timeEstimateReady) {
            ++timeEstimateReadyCount;
        }

        if(sample.timeDiscontinuityDetected) {
            ++timeDiscontinuityCount;
        }

        if(index == 0) {
            continue;
        }

        const TimedLidarSample& previousSample = samples[index - 1];

        if(sample.timestampUs < previousSample.timestampUs) {
            ++timeBackwardsCount;
        }
        else if(sample.timestampUs == previousSample.timestampUs) {
            ++duplicateTimestampCount;
        }
        else {
            positiveIntervalSumUs += sample.timestampUs - previousSample.timestampUs;

            ++positiveIntervalCount;
        }

        if(sample.sequenceNumber == previousSample.sequenceNumber) {
            ++duplicateSequenceCount;
        }
        else if(sample.sequenceNumber > previousSample.sequenceNumber) {
            missingSequenceCount += sample.sequenceNumber - previousSample.sequenceNumber - 1ULL;
        }
        else {
            ++backwardsSequenceCount;
        }
    }

    double averageIntervalUs = 0.0;
    double averageRateHz = 0.0;

    if(positiveIntervalCount > 0) {
        averageIntervalUs = static_cast<double>(positiveIntervalSumUs) / static_cast<double>(positiveIntervalCount);

        if(averageIntervalUs > 0.0) {
            averageRateHz = 1000000.0 / averageIntervalUs;
        }
    }

    const double expectedRateHz = runtimeConfig_.lidarUpdateRateHz;
    const double expectedIntervalUs = 1000000.0 / expectedRateHz;

    double rateErrorPercent = 0.0;

    if(expectedRateHz > 0.0) {
        rateErrorPercent = std::abs(averageRateHz - expectedRateHz) / expectedRateHz * 100.0;
    }

    uint64_t durationUs = 0;

    if(samples.back().timestampUs >= samples.front().timestampUs) {
        durationUs = samples.back().timestampUs - samples.front().timestampUs;
    }

     std::cout
        << "[Postprocess] LiDAR samples loaded"
        << ", loaded=" << loadResult.loadedRowCount
        << ", skipped=" << loadResult.skippedRowCount
        << ", valid=" << validMeasurementCount
        << ", invalid=" << invalidMeasurementCount
        << ", timeEstimateReady="
        << timeEstimateReadyCount
        << ", timeDiscontinuities="
        << timeDiscontinuityCount
        << ", timeBackwards="
        << timeBackwardsCount
        << ", duplicateTimestamps="
        << duplicateTimestampCount
        << ", missingSequences="
        << missingSequenceCount
        << ", duplicateSequences="
        << duplicateSequenceCount
        << ", backwardsSequences="
        << backwardsSequenceCount
        << ", firstTimeUs="
        << samples.front().timestampUs
        << ", lastTimeUs="
        << samples.back().timestampUs
        << ", durationUs="
        << durationUs
        << ", averageIntervalUs="
        << averageIntervalUs
        << ", averageRateHz="
        << averageRateHz
        << ", expectedRateHz="
        << expectedRateHz
        << ", expectedIntervalUs="
        << expectedIntervalUs
        << ", rateErrorPercent="
        << rateErrorPercent
        << ", configuredAngleRangeDeg=("
        << runtimeConfig_.lidarScanLowAngleDeg
        << ","
        << runtimeConfig_.lidarScanHighAngleDeg
        << ")"
        << ", angleOutsideConfiguredRange="
        << angleOutsideConfiguredRangeCount
        << std::endl;

    return true;
}

bool PostprocessManager::validateLidarPoseMatching(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedLidarSample> lidarSamples;
    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedAttitudeSample> attitudeSamples;

    const CsvLoadResult lidarResult = loader.loadLidarSamples((sessionDirectory / "lidar.csv").string(), lidarSamples);

    if(!lidarResult.success) {
        std::cerr << "[Postprocess] LiDAR pose matching " << "LiDAR load failed: " << lidarResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] LiDAR pose matching " << "RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult attitudeResult = loader.loadAttitudeSamples((sessionDirectory / "attitude.csv").string(), attitudeSamples);

    if(!attitudeResult.success) {
        std::cerr << "[Postprocess] LiDAR pose matching " << "attitude load failed: " << attitudeResult.errorMessage << std::endl;

        return false;
    }

    size_t poseMatchedCount = 0;

    size_t positionInvalidCount = 0;
    size_t attitudeInvalidCount = 0;
    size_t bothInvalidCount = 0;

    size_t validMeasurementCount = 0;
    size_t invalidMeasurementCount = 0;
    size_t usablePointCount = 0;

    size_t timeEstimateNotReadyCount = 0;
    size_t timeDiscontinuityCount = 0;

    double maximumQuaternionNormError = 0.0;

    for(const TimedLidarSample& lidarSample : lidarSamples) {
        if(lidarSample.measurementValid) {
            ++validMeasurementCount;
        }
        else {
            ++invalidMeasurementCount;
        }

        if(!lidarSample.timeEstimateReady) {
            ++timeEstimateNotReadyCount;
        }

        if(lidarSample.timeDiscontinuityDetected) {
            ++timeDiscontinuityCount;
        }

        InterpolatedPose interpolatedPose{};

        const bool poseMatched = interpolator.interpolatePose(
            lidarSample.timestampUs, 
            rtkSamples, 
            attitudeSamples, 
            AppConfig::POSITION_INTERPOLATION_MAX_GAP_US, 
            AppConfig::ATTITUDE_INTERPOLATION_MAX_GAP_US, 
            interpolatedPose
        );

        if(!interpolatedPose.positionValid && !interpolatedPose.attitudeValid) {
            ++bothInvalidCount;
            continue;
        }

        if(!interpolatedPose.positionValid) {
            ++positionInvalidCount;
            continue;
        }

        if(!interpolatedPose.attitudeValid) {
            ++attitudeInvalidCount;
            continue;
        }

        if(!poseMatched) {
            ++bothInvalidCount;
            continue;
        }

        ++poseMatchedCount;

        const double quaternionNorm = std::sqrt(
            interpolatedPose.q0W * interpolatedPose.q0W
            + interpolatedPose.q1X * interpolatedPose.q1X
            + interpolatedPose.q2Y * interpolatedPose.q2Y
            + interpolatedPose.q3Z * interpolatedPose.q3Z
        );

        const double quaternionNormError = std::abs(quaternionNorm - 1.0);

        if(quaternionNormError > maximumQuaternionNormError) {
            maximumQuaternionNormError = quaternionNormError;
        }

        if(lidarSample.measurementValid) {
            ++usablePointCount;
        }
    }

    std::cout
        << "[Postprocess] LiDAR pose matching"
        << ", lidarSamples=" << lidarSamples.size()
        << ", poseMatched=" << poseMatchedCount
        << ", positionInvalid=" << positionInvalidCount
        << ", attitudeInvalid=" << attitudeInvalidCount
        << ", bothInvalid=" << bothInvalidCount
        << ", validMeasurements="
        << validMeasurementCount
        << ", invalidMeasurements="
        << invalidMeasurementCount
        << ", usablePoints=" << usablePointCount
        << ", timeEstimateNotReady="
        << timeEstimateNotReadyCount
        << ", timeDiscontinuities="
        << timeDiscontinuityCount
        << ", maximumQuaternionNormError="
        << maximumQuaternionNormError
        << std::endl;

    return poseMatchedCount > 0;
}

bool PostprocessManager::validateLidarRtkStatusMatching(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedLidarSample> lidarSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult lidarResult = loader.loadLidarSamples((sessionDirectory / "lidar.csv").string(), lidarSamples);

    if(!lidarResult.success) {
        std::cerr << "[Postprocess] LiDAR RTK status matching " << "Lidar load failed: " << lidarResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult statusResult = loader.loadRtkStatusSamples((sessionDirectory / "rtk_status.csv").string(), statusSamples);

    if(!statusResult.success) {
        std::cerr << "[Postprocess] LiDAR RTK status matching " << "RTK status load failed: " << statusResult.errorMessage << std::endl;

        return false;
    }

    size_t matchedCount = 0;
    size_t unmatchedCount = 0;

    size_t unavailableCount = 0;
    size_t floatCount = 0;
    size_t fixedCount = 0;
    size_t otherCount = 0;

    size_t fixedValidMeasurementCount = 0;

    uint64_t maximumObservedAgeUs = 0;

    for(const TimedLidarSample& lidarSample : lidarSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool matched = interpolator.findLatestRtkStatus(lidarSample.timestampUs, statusSamples, AppConfig::RTK_STATUS_MAX_AGE_US, matchedStatus);
    
        if(!matched) {
            ++unmatchedCount;
            continue;
        }

        ++matchedCount;

        const uint64_t statusAgeUs = lidarSample.timestampUs - matchedStatus.timestampUs;

        if(statusAgeUs > maximumObservedAgeUs) {
            maximumObservedAgeUs = statusAgeUs;
        }

        switch(matchedStatus.positionInfo) {
            case 0:
                ++unavailableCount;
                break;

            case 34:
                ++floatCount;
                break;
            
            case 50:
                ++fixedCount;

                if(lidarSample.measurementValid) {
                    ++fixedValidMeasurementCount;
                }

                break;

            default:
                ++otherCount;
                break;
        }
    }

    std::cout
        << "[Postprocess] LiDAR RTK status matching"
        << ", lidarSamples=" << lidarSamples.size()
        << ", matched=" << matchedCount
        << ", unmatched=" << unmatchedCount
        << ", unavailable=" << unavailableCount
        << ", float=" << floatCount
        << ", fixed=" << fixedCount
        << ", other=" << otherCount
        << ", fixedValidMeasurements="
        << fixedValidMeasurementCount
        << ", maximumObservedAgeUs="
        << maximumObservedAgeUs
        << std::endl;

    return matchedCount > 0;
}

bool PostprocessManager::validateLidarBodyCoordinateTransform() const {
    constexpr double testDistanceM = 10.0;
    constexpr double toleranceM = 1.0e-9;

    PostprocessCoordinateTransformer transformer;

    CartesianPoint zeroDegreePoint{};
    CartesianPoint negativeNinetyDegreePoint{};
    CartesianPoint positiveNinetyDegreePoint{};

    const bool zeroDegreeConverted = transformer.lidarMeasurementToBodyPoint(0.0, testDistanceM, zeroDegreePoint);
    const bool negativeNinetyDegreeConverted = transformer.lidarMeasurementToBodyPoint(-90.0, testDistanceM, negativeNinetyDegreePoint);
    const bool positiveNinetyDegreeConverted = transformer.lidarMeasurementToBodyPoint(90.0, testDistanceM, positiveNinetyDegreePoint);
    
    const auto approximatelyEqual = [toleranceM](double left, double right) {
        return std::abs(left - right) <= toleranceM;
    };

    const bool zeroDegreeCorrect = zeroDegreeConverted 
        && approximatelyEqual(zeroDegreePoint.xM, 0.0)
        && approximatelyEqual(zeroDegreePoint.yM, 0.0)
        && approximatelyEqual(zeroDegreePoint.zM, testDistanceM);

    const bool negativeNinetyDegreeCorrect = negativeNinetyDegreeConverted
        && approximatelyEqual(negativeNinetyDegreePoint.xM, 0.0)
        && approximatelyEqual(negativeNinetyDegreePoint.yM, testDistanceM)
        && approximatelyEqual(negativeNinetyDegreePoint.zM, 0.0);

    const bool positiveNinetyDegreeCorrect = positiveNinetyDegreeConverted
        && approximatelyEqual(positiveNinetyDegreePoint.xM, 0.0)
        && approximatelyEqual(positiveNinetyDegreePoint.yM, -testDistanceM)
        && approximatelyEqual(positiveNinetyDegreePoint.zM, 0.0);
        
    const bool validationSucceeded = zeroDegreeCorrect
        && negativeNinetyDegreeCorrect
        && positiveNinetyDegreeCorrect;

    std::cout
        << "[Postprocess] LiDAR body coordinate transform"
        << ", zeroDegree=("
        << zeroDegreePoint.xM << ","
        << zeroDegreePoint.yM << ","
        << zeroDegreePoint.zM << ")"
        << ", negativeNinetyDegree=("
        << negativeNinetyDegreePoint.xM << ","
        << negativeNinetyDegreePoint.yM << ","
        << negativeNinetyDegreePoint.zM << ")"
        << ", positiveNinetyDegree=("
        << positiveNinetyDegreePoint.xM << ","
        << positiveNinetyDegreePoint.yM << ","
        << positiveNinetyDegreePoint.zM << ")"
        << ", success="
        << (validationSucceeded ? 1 : 0)
        << std::endl;

    return validationSucceeded;
}

bool PostprocessManager::validateBodyToNedRotation() const {
    constexpr double toleranceM = 1.0e-9;
    constexpr double squareRootHalf = 0.70710678118654752440;

    PostprocessCoordinateTransformer transformer;

    const auto approximatelyEqual = [toleranceM](double left, double right) {
        return std::abs(left - right) <= toleranceM;
    };

    TimedAttitudeSample identityAttitude{};

    identityAttitude.q0W = 1.0;
    identityAttitude.q1X = 0.0;
    identityAttitude.q2Y = 0.0;
    identityAttitude.q3Z = 0.0;

    CartesianPoint identityBodyPoint{};

    identityBodyPoint.xM = 1.0;
    identityBodyPoint.yM = 2.0;
    identityBodyPoint.zM = 3.0;

    CartesianPoint identityNedPoint{};

    const bool identityConverted = transformer.rotateBodyPointToNed(identityBodyPoint, identityAttitude, identityNedPoint);
    const bool identityCorrect = identityConverted
        && approximatelyEqual(identityNedPoint.xM, identityBodyPoint.xM)
        && approximatelyEqual(identityNedPoint.yM, identityBodyPoint.yM)
        && approximatelyEqual(identityNedPoint.zM, identityBodyPoint.zM);

    TimedAttitudeSample yawNinetyAttitude{};

    yawNinetyAttitude.q0W = squareRootHalf;
    yawNinetyAttitude.q1X = 0.0;
    yawNinetyAttitude.q2Y = 0.0;
    yawNinetyAttitude.q3Z = squareRootHalf;

    CartesianPoint forwardBodyPoint{};

    forwardBodyPoint.xM = 10.0;
    forwardBodyPoint.yM = 0.0;
    forwardBodyPoint.zM = 0.0;

    CartesianPoint yawNinetyNedPoint{};

    const bool yawNinetyConverted = transformer.rotateBodyPointToNed(forwardBodyPoint, yawNinetyAttitude, yawNinetyNedPoint);

    const bool yawNinetyCorrect = yawNinetyConverted 
        && approximatelyEqual(yawNinetyNedPoint.xM, 0.0)
        && approximatelyEqual(yawNinetyNedPoint.yM, 10.0)
        && approximatelyEqual(yawNinetyNedPoint.zM, 0.0);

    const double bodyLengthM = std::sqrt(
        forwardBodyPoint.xM * forwardBodyPoint.xM
        + forwardBodyPoint.yM * forwardBodyPoint.yM
        + forwardBodyPoint.zM * forwardBodyPoint.zM
    );

    const double nedLengthM = std::sqrt(
        yawNinetyNedPoint.xM * yawNinetyNedPoint.xM
        + yawNinetyNedPoint.yM * yawNinetyNedPoint.yM
        + yawNinetyNedPoint.zM * yawNinetyNedPoint.zM
    );

    const bool lengthPreserved = approximatelyEqual(bodyLengthM, nedLengthM);
    const bool validationSucceeded = identityCorrect && yawNinetyCorrect && lengthPreserved;

    std::cout
        << "[Postprocess] Body to NED rotation"
        << ", identity=("
        << identityNedPoint.xM << ","
        << identityNedPoint.yM << ","
        << identityNedPoint.zM << ")"
        << ", yawNinety=("
        << yawNinetyNedPoint.xM << ","
        << yawNinetyNedPoint.yM << ","
        << yawNinetyNedPoint.zM << ")"
        << ", bodyLengthM=" << bodyLengthM
        << ", nedLengthM=" << nedLengthM
        << ", success="
        << (validationSucceeded ? 1 : 0)
        << std::endl;

    return validationSucceeded;
}

bool PostprocessManager::validateGeodeticToLocalNed(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;
    PostprocessCoordinateTransformer transformer;

    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] Local NED RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult statusResult = loader.loadRtkStatusSamples((sessionDirectory / "rtk_status.csv").string(), statusSamples);

    if(!statusResult.success) {
        std::cerr << "[Postprocess] Local NED RTK status load failed: " << statusResult.errorMessage << std::endl;

        return false;
    }

    TimedPositionSample referencePosition{};
    bool referenceFound = false;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched = interpolator.findLatestRtkStatus(rtkSample.timestampUs, statusSamples, AppConfig::RTK_STATUS_MAX_AGE_US, matchedStatus);

        if(statusMatched && matchedStatus.positionInfo == 50) {
            referencePosition = rtkSample;
            referenceFound = true;
            break;
        }
    }

    if(!referenceFound) {
        std::cerr << "[Postprocess] Local NED reference " << "position was not found" << std::endl;

        return false;
    }

    CartesianPoint referenceNed{};

    const bool referenceConverted = transformer.geodeticPositionToLocalNed(referencePosition, referencePosition, referenceNed);

    if(!referenceConverted) {
        std::cerr << "[Postprocess] Local NED reference " << "conversion failed" << std::endl;

        return false;
    }

    const double referenceOriginErrorM = std::sqrt(referenceNed.xM * referenceNed.xM + referenceNed.yM * referenceNed.yM + referenceNed.zM * referenceNed.zM);

    size_t convertedCount = 0;
    size_t failedCount = 0;

    bool boundsInitialized = false;

    double minimumNorthM = 0.0;
    double maximumNorthM = 0.0;

    double minimumEastM = 0.0;
    double maximumEastM = 0.0;

    double minimumDownM = 0.0;
    double maximumDownM = 0.0;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        CartesianPoint localNed{};

        const bool converted = transformer.geodeticPositionToLocalNed(referencePosition, rtkSample, localNed);

        if(!converted) {
            ++failedCount;
            continue;
        }

        ++convertedCount;

        if(!boundsInitialized) {
            minimumNorthM = localNed.xM;
            maximumNorthM = localNed.xM;

            minimumEastM = localNed.yM;
            maximumEastM = localNed.yM;

            minimumDownM = localNed.zM;
            maximumDownM = localNed.zM;

            boundsInitialized = true;
            continue;
        }

        if(localNed.xM < minimumNorthM) {
            minimumNorthM = localNed.xM;
        }

        if(localNed.xM > maximumNorthM) {
            maximumNorthM = localNed.xM;
        }

        if(localNed.yM < minimumEastM) {
            minimumEastM = localNed.yM;
        }

        if(localNed.yM > maximumEastM) {
            maximumEastM = localNed.yM;
        }

        if(localNed.zM < minimumDownM) {
            minimumDownM = localNed.zM;
        }

        if(localNed.zM > maximumDownM) {
            maximumDownM = localNed.zM;
        }
    }

    const bool referenceAtOrigin = referenceOriginErrorM <= 1.0e-6;

    std::cout
        << "[Postprocess] Geodetic to local NED"
        << ", referenceTimeUs="
        << referencePosition.timestampUs
        << ", referenceLatitude="
        << referencePosition.latitudeDeg
        << ", referenceLongitude="
        << referencePosition.longitudeDeg
        << ", referenceAltitudeM="
        << referencePosition.altitudeM
        << ", referenceOriginErrorM="
        << referenceOriginErrorM
        << ", converted=" << convertedCount
        << ", failed=" << failedCount
        << ", northRangeM=("
        << minimumNorthM << ","
        << maximumNorthM << ")"
        << ", eastRangeM=("
        << minimumEastM << ","
        << maximumEastM << ")"
        << ", downRangeM=("
        << minimumDownM << ","
        << maximumDownM << ")"
        << ", success="
        << (
            referenceAtOrigin
            && convertedCount > 0
            ? 1
            : 0
        )
        << std::endl;

    return referenceAtOrigin && convertedCount > 0;
}

bool PostprocessManager::validateLocalNedLidarPointGeneration(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;
    PostprocessCoordinateTransformer transformer;

    if(!transformer.configureLidarOffset(runtimeConfig_.lidarOffsetForwardM, runtimeConfig_.lidarOffsetRightM,runtimeConfig_.lidarOffsetDownM)) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "offset configuration failed" << std::endl;

        return false;
    }

    std::vector<TimedLidarSample> lidarSamples;
    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedAttitudeSample> attitudeSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult lidarResult = loader.loadLidarSamples((sessionDirectory / "lidar.csv").string(), lidarSamples);

    if(!lidarResult.success) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "LiDAR load failed: " << lidarResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult attitudeResult = loader.loadAttitudeSamples((sessionDirectory / "attitude.csv").string(), attitudeSamples);

    if(!attitudeResult.success) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "attitude load failed: " << attitudeResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult statusResult = loader.loadRtkStatusSamples((sessionDirectory / "rtk_status.csv").string(), statusSamples);

    if(!statusResult.success) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "RTK status load failed: " << statusResult.errorMessage << std::endl;

        return false;
    }

    TimedPositionSample referencePosition{};
    bool referenceFound = false;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched = interpolator.findLatestRtkStatus(rtkSample.timestampUs, statusSamples, AppConfig::RTK_STATUS_MAX_AGE_US, matchedStatus);

        if(statusMatched && matchedStatus.positionInfo == 50) {
            referencePosition = rtkSample;
            referenceFound = true;
            break;
        }
    }

    if(!referenceFound) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation " << "fixed RTK reference position was not found" << std::endl;

        return false;
    }

    size_t generatedPointCount = 0;
    size_t invalidMeasurementCount = 0;
    size_t timeEstimateNotReadyCount = 0;
    size_t timeDiscontinuityCount = 0;
    size_t statusUnmatchedCount = 0;
    size_t statusNotFixedCount = 0;
    size_t poseInterpolationFailedCount = 0;
    size_t coordinateTransformFailedCount = 0;

    bool boundsInitialized = false;

    double minimumNorthM = 0.0;
    double maximumNorthM = 0.0;
    double minimumEastM = 0.0;
    double maximumEastM = 0.0;
    double minimumDownM = 0.0;
    double maximumDownM = 0.0;

    for(const TimedLidarSample& lidarSample : lidarSamples) {
        if(!lidarSample.measurementValid) {
            ++invalidMeasurementCount;
            continue;
        }

        if(!lidarSample.timeEstimateReady) {
            ++timeEstimateNotReadyCount;
            continue;
        }

        if(lidarSample.timeDiscontinuityDetected) {
            ++timeDiscontinuityCount;
            continue;
        }

        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched = interpolator.findLatestRtkStatus(lidarSample.timestampUs, statusSamples, AppConfig::RTK_STATUS_MAX_AGE_US, matchedStatus);

        if(!statusMatched) {
            ++statusUnmatchedCount;
            continue;
        }

        if(matchedStatus.positionInfo != 50) {
            ++statusNotFixedCount;
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
            ++poseInterpolationFailedCount;
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
            ++coordinateTransformFailedCount;
            continue;
        }

        ++generatedPointCount;

        if(!boundsInitialized) {
            minimumNorthM = localNedPoint.xM;
            maximumNorthM = localNedPoint.xM;
            minimumEastM = localNedPoint.yM;
            maximumEastM = localNedPoint.yM;
            minimumDownM = localNedPoint.zM;
            maximumDownM = localNedPoint.zM;

            boundsInitialized = true;
            continue;
        }

        if(localNedPoint.xM < minimumNorthM) {
            minimumNorthM = localNedPoint.xM;
        }

        if(localNedPoint.xM > maximumNorthM) {
            maximumNorthM = localNedPoint.xM;
        }

        if(localNedPoint.yM < minimumEastM) {
            minimumEastM = localNedPoint.yM;
        }

        if(localNedPoint.yM > maximumEastM) {
            maximumEastM = localNedPoint.yM;
        }

        if(localNedPoint.zM < minimumDownM) {
            minimumDownM = localNedPoint.zM;
        }

        if(localNedPoint.zM > maximumDownM) {
            maximumDownM = localNedPoint.zM;
        }
    }

    std::cout
        << "[Postprocess] Local NED LiDAR point generation"
        << ", lidarSamples=" << lidarSamples.size()
        << ", generated=" << generatedPointCount
        << ", invalidMeasurement=" << invalidMeasurementCount
        << ", timeEstimateNotReady=" << timeEstimateNotReadyCount
        << ", timeDiscontinuity=" << timeDiscontinuityCount
        << ", statusUnmatched=" << statusUnmatchedCount
        << ", statusNotFixed=" << statusNotFixedCount
        << ", poseInterpolationFailed="
        << poseInterpolationFailedCount
        << ", coordinateTransformFailed="
        << coordinateTransformFailedCount
        << ", referenceTimeUs=" << referencePosition.timestampUs
        << ", northRangeM=("
        << minimumNorthM << "," << maximumNorthM << ")"
        << ", eastRangeM=("
        << minimumEastM << "," << maximumEastM << ")"
        << ", downRangeM=("
        << minimumDownM << "," << maximumDownM << ")"
        << std::endl;

    return generatedPointCount > 0;
}

bool PostprocessManager::generateLocalNedPointCloudCsv(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessPointCloudGenerator generator;
    PostprocessPointCloudProjector projector;
    PostprocessPointCloudWriter writer;

    std::vector<TimedLidarSample> lidarSamples;
    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedAttitudeSample> attitudeSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult lidarResult = loader.loadLidarSamples((sessionDirectory / "lidar.csv").string(), lidarSamples);

    if(!lidarResult.success) {
        std::cerr << "[Postprocess] Point cloud generation LiDAR load failed: " << lidarResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] Point cloud generation RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult attitudeResult = loader.loadAttitudeSamples((sessionDirectory / "attitude.csv").string(), attitudeSamples);

    if(!attitudeResult.success) {
        std::cerr << "[Postprocess] Point cloud generation attitude load failed: " << attitudeResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult statusResult = loader.loadRtkStatusSamples((sessionDirectory / "rtk_status.csv").string(), statusSamples);

    if(!statusResult.success) {
        std::cerr << "[Postprocess] Point cloud generation RTK status load failed: " << statusResult.errorMessage << std::endl;

        return false;
    }

    std::vector<GeneratedLidarPoint> generatedPoints;
    TimedPositionSample referencePosition{};

    const bool generated = generator.generateLocalNedPoints(
        lidarSamples,
        rtkSamples,
        attitudeSamples,
        statusSamples,
        runtimeConfig_,
        generatedPoints,
        referencePosition
    );

    if(!generated) {
        std::cerr << "[Postprocess] Local NED point cloud generation failed" << std::endl;

        return false;
    }

    const bool written = writer.writeLocalNedCsv(sessionDirectory, generatedPoints, referencePosition);

    if(!written) {
        std::cerr << "[Postprocess] Local NED point cloud CSV write failed" << std::endl;

        return false;
    }

    const bool lasWritten = writer.writeLocalLas(sessionDirectory, generatedPoints);
    
    if(!lasWritten) {
        std::cerr << "[Postprocess] Local LAS write failed" << std::endl;

        return false;
    }

    std::vector<ProjectedLidarPoint> projectedPoints;

    const bool projected = projector.projectPoints(
        generatedPoints,
        referencePosition,
        runtimeConfig_.sourceGeographicEpsgCode,
        runtimeConfig_.outputProjectedEpsgCode,
        projectedPoints
    );

    if(!projected) {
        std::cerr
            << "[Postprocess] Projected point cloud generation failed"
            << std::endl;

        return false;
    }

    const bool projectedCsvWritten = writer.writeProjectedCsv(
        sessionDirectory,
        projectedPoints,
        runtimeConfig_.outputProjectedEpsgCode
    );

    if(!projectedCsvWritten) {
        std::cerr
            << "[Postprocess] Projected point cloud CSV write failed"
            << std::endl;

        return false;
    }

    const bool projectedLasWritten = writer.writeProjectedLas(sessionDirectory, projectedPoints, runtimeConfig_.outputProjectedEpsgCode);

    if(!projectedLasWritten) {
        std::cerr
            << "[Postprocess] Projected LAS write failed"
            << std::endl;

        return false;
    }

    std::cout
        << "[Postprocess] Local NED point cloud CSV written"
        << ", points=" << generatedPoints.size()
        << ", pointCloudPath="
        << (sessionDirectory / "point_cloud.csv")
        << ", referencePath="
        << (sessionDirectory / "point_cloud_reference.csv")
        << ", localLasPath="
        << (sessionDirectory / "point_cloud_local.las")
        << ", projectedPoints="
        << projectedPoints.size()
        << ", projectedCsvPath="
        << (
            sessionDirectory
            / (
                "point_cloud_epsg"
                + std::to_string(
                    runtimeConfig_.outputProjectedEpsgCode
                )
                + ".csv"
            )
        )
        << ", projectedLasPath="
        << (
            sessionDirectory
            / (
                "point_cloud_epsg"
                + std::to_string(
                    runtimeConfig_.outputProjectedEpsgCode
                )
                + ".las"
            )
        )
        << std::endl;

    return true;
}

bool PostprocessManager::validateCrsTransformation(const std::filesystem::path& sessionDirectory) const {
    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;

    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult rtkResult = loader.loadRtkPositionSamples((sessionDirectory / "position_rtk.csv").string(), rtkSamples);

    if(!rtkResult.success) {
        std::cerr << "[Postprocess] CRS transformation RTK load failed: " << rtkResult.errorMessage << std::endl;

        return false;
    }

    const CsvLoadResult statusResult = loader.loadRtkStatusSamples((sessionDirectory / "rtk_status.csv").string(), statusSamples);

    if(!statusResult.success) {
        std::cerr << "[Postprocess] CRS transformation RTK status load failed: " << statusResult.errorMessage << std::endl;

        return false;
    }

    TimedPositionSample referencePosition{};
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
        std::cerr << "[Postprocess] CRS transformation fixed RTK " << "reference position was not found" << std::endl;

        return false;
    }

    PostprocessCrsTransformer transformer(runtimeConfig_.sourceGeographicEpsgCode, runtimeConfig_.outputProjectedEpsgCode);

    if(!transformer.isValid()) {
        std::cerr << "[Postprocess] CRS transformer initialization failed" << std::endl;

        return false;
    }

    double projectedXM = 0.0;
    double projectedYM = 0.0;

    const bool transformed = transformer.transform(
        referencePosition.longitudeDeg,
        referencePosition.latitudeDeg,
        projectedXM,
        projectedYM
    );

    if(!transformed) {
        std::cerr << "[Postprocess] CRS reference coordinate " << "transformation failed" << std::endl;

        return false;
    }

    std::cout
        << "[Postprocess] CRS transformation"
        << ", sourceEpsg="
        << runtimeConfig_.sourceGeographicEpsgCode
        << ", targetEpsg="
        << runtimeConfig_.outputProjectedEpsgCode
        << ", referenceTimeUs="
        << referencePosition.timestampUs
        << ", longitude="
        << referencePosition.longitudeDeg
        << ", latitude="
        << referencePosition.latitudeDeg
        << ", projectedX="
        << projectedXM
        << ", projectedY="
        << projectedYM
        << std::endl;

    return true;
}

bool PostprocessManager::validateLocalNedGeodeticRoundTrip(
    const std::filesystem::path& sessionDirectory
) const {
    constexpr double pi = 3.14159265358979323846;
    constexpr double degreesToRadians = pi / 180.0;
    constexpr double earthRadiusM = 6378137.0;
    constexpr double maximumAllowedErrorM = 0.001;

    PostprocessDataLoader loader;
    PostprocessInterpolator interpolator;
    PostprocessCoordinateTransformer transformer;

    std::vector<TimedPositionSample> rtkSamples;
    std::vector<TimedRtkStatusSample> statusSamples;

    const CsvLoadResult rtkResult =
        loader.loadRtkPositionSamples(
            (sessionDirectory / "position_rtk.csv").string(),
            rtkSamples
        );

    if(!rtkResult.success) {
        std::cerr
            << "[Postprocess] Local NED round-trip RTK load failed: "
            << rtkResult.errorMessage
            << std::endl;

        return false;
    }

    const CsvLoadResult statusResult =
        loader.loadRtkStatusSamples(
            (sessionDirectory / "rtk_status.csv").string(),
            statusSamples
        );

    if(!statusResult.success) {
        std::cerr
            << "[Postprocess] Local NED round-trip RTK status load failed: "
            << statusResult.errorMessage
            << std::endl;

        return false;
    }

    TimedPositionSample referencePosition{};
    bool referenceFound = false;

    for(const TimedPositionSample& rtkSample : rtkSamples) {
        TimedRtkStatusSample matchedStatus{};

        const bool statusMatched =
            interpolator.findLatestRtkStatus(
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
        std::cerr
            << "[Postprocess] Local NED round-trip fixed RTK "
            << "reference position was not found"
            << std::endl;

        return false;
    }

    size_t validatedCount = 0;
    size_t failedCount = 0;

    double horizontalErrorSumM = 0.0;
    double maximumHorizontalErrorM = 0.0;

    double altitudeErrorSumM = 0.0;
    double maximumAltitudeErrorM = 0.0;

    for(const TimedPositionSample& originalPosition : rtkSamples) {
        CartesianPoint localNedPosition{};

        const bool convertedToNed =
            transformer.geodeticPositionToLocalNed(
                referencePosition,
                originalPosition,
                localNedPosition
            );

        if(!convertedToNed) {
            ++failedCount;
            continue;
        }

        TimedPositionSample restoredPosition{};

        const bool convertedToGeodetic =
            transformer.localNedToGeodeticPosition(
                referencePosition,
                localNedPosition,
                restoredPosition
            );

        if(!convertedToGeodetic) {
            ++failedCount;
            continue;
        }

        const double latitudeDifferenceRad =
            (
                restoredPosition.latitudeDeg
                - originalPosition.latitudeDeg
            )
            * degreesToRadians;

        const double longitudeDifferenceRad =
            (
                restoredPosition.longitudeDeg
                - originalPosition.longitudeDeg
            )
            * degreesToRadians;

        const double meanLatitudeRad =
            (
                restoredPosition.latitudeDeg
                + originalPosition.latitudeDeg
            )
            * 0.5
            * degreesToRadians;

        const double northErrorM =
            latitudeDifferenceRad * earthRadiusM;

        const double eastErrorM =
            longitudeDifferenceRad
            * earthRadiusM
            * std::cos(meanLatitudeRad);

        const double horizontalErrorM =
            std::hypot(northErrorM, eastErrorM);

        const double altitudeErrorM =
            std::abs(
                restoredPosition.altitudeM
                - originalPosition.altitudeM
            );

        horizontalErrorSumM += horizontalErrorM;
        altitudeErrorSumM += altitudeErrorM;

        if(horizontalErrorM > maximumHorizontalErrorM) {
            maximumHorizontalErrorM = horizontalErrorM;
        }

        if(altitudeErrorM > maximumAltitudeErrorM) {
            maximumAltitudeErrorM = altitudeErrorM;
        }

        ++validatedCount;
    }

    if(validatedCount == 0) {
        std::cerr
            << "[Postprocess] Local NED round-trip validation failed: "
            << "no positions were validated"
            << std::endl;

        return false;
    }

    const double averageHorizontalErrorM =
        horizontalErrorSumM
        / static_cast<double>(validatedCount);

    const double averageAltitudeErrorM =
        altitudeErrorSumM
        / static_cast<double>(validatedCount);

    const bool validationSucceeded =
        failedCount == 0
        && maximumHorizontalErrorM
            <= maximumAllowedErrorM
        && maximumAltitudeErrorM
            <= maximumAllowedErrorM;

    std::cout
        << "[Postprocess] Local NED geodetic round-trip"
        << ", samples=" << rtkSamples.size()
        << ", validated=" << validatedCount
        << ", failed=" << failedCount
        << ", averageHorizontalErrorM="
        << averageHorizontalErrorM
        << ", maximumHorizontalErrorM="
        << maximumHorizontalErrorM
        << ", averageAltitudeErrorM="
        << averageAltitudeErrorM
        << ", maximumAltitudeErrorM="
        << maximumAltitudeErrorM
        << ", toleranceM="
        << maximumAllowedErrorM
        << ", success="
        << (validationSucceeded ? 1 : 0)
        << std::endl;

    return validationSucceeded;
}

bool PostprocessManager::configure(const RuntimeConfig& runtimeConfig) {
    std::string validationError;

    if(!runtimeConfig.validate(validationError)) {
        std::cerr
            << "[Postprocess] Runtime configuration rejected: "
            << validationError
            << std::endl;

        return false;
    }

    runtimeConfig_ = runtimeConfig;

    std::cout
        << "[Postprocess] Runtime configuration accepted"
        << ", lidarRateHz="
        << runtimeConfig_.lidarUpdateRateHz
        << ", scanAngleDeg=("
        << runtimeConfig_.lidarScanLowAngleDeg
        << ","
        << runtimeConfig_.lidarScanHighAngleDeg
        << ")"
        << ", lidarOffsetFrdM=("
        << runtimeConfig_.lidarOffsetForwardM
        << ","
        << runtimeConfig_.lidarOffsetRightM
        << ","
        << runtimeConfig_.lidarOffsetDownM
        << ")"
        << ", sourceEpsg="
        << runtimeConfig_.sourceGeographicEpsgCode
        << ", outputEpsg="
        << runtimeConfig_.outputProjectedEpsgCode
        << std::endl;

    return true;
}