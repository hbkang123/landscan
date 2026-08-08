#pragma once

#include "runtime_config.hpp"

#include <filesystem>

class PostprocessManager
{

public:
    bool validateAttitudeData(const std::filesystem::path& sessionDirectory) const;
    bool validateRtkPositionData(const std::filesystem::path& sessionDirectory) const;
    bool validateRtkStatusData(const std::filesystem::path& sessionDirectory) const;
    bool validateRtkStatusMatching(const std::filesystem::path& sessionDirectory) const;
    bool validateGpsPositionData(const std::filesystem::path& sessionDirectory) const;
    bool validatePositionInterpolation(const std::filesystem::path& sessionDirectory) const;
    bool validateAttitudeInterpolation(const std::filesystem::path& sessionDirectory) const;
    bool validatePoseInterpolation(const std::filesystem::path& sessionDirectory) const;
    bool validateLidarData(const std::filesystem::path& sessionDirectory) const;
    bool validateLidarPoseMatching(const std::filesystem::path& sessionDirectory) const;
    bool validateLidarRtkStatusMatching(const std::filesystem::path& sessionDirectory) const;
    bool validateLidarBodyCoordinateTransform() const;
    bool validateBodyToNedRotation() const;
    bool validateGeodeticToLocalNed(const std::filesystem::path& sessionDirectory) const;
    bool validateLocalNedLidarPointGeneration(const std::filesystem::path& sessionDirectory) const;
    bool generateLocalNedPointCloudCsv(const std::filesystem::path& sessionDirectory) const;
    bool validateCrsTransformation(const std::filesystem::path& sessionDirectory) const;
    bool validateLocalNedGeodeticRoundTrip(const std::filesystem::path& sessionDirectory) const;
    bool configure(const RuntimeConfig& runtimeConfig);
private:
    RuntimeConfig runtimeConfig_{};
};