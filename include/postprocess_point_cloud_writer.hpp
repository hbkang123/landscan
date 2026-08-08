#pragma once

#include "postprocess_types.hpp"

#include <filesystem>
#include <vector>

class PostprocessPointCloudWriter
{
public:
    bool writeLocalNedCsv(
        const std::filesystem::path& sessionDirectory,
        const std::vector<GeneratedLidarPoint>& points,
        const TimedPositionSample& referencePosition
    ) const;

    bool writeLocalLas(const std::filesystem::path& sessionDriectroy, const std::vector<GeneratedLidarPoint>& points) const;

    bool writeProjectedCsv(
        const std::filesystem::path& sessionDirectory,
        const std::vector<ProjectedLidarPoint>& points,
        int targetEpsgCode
    ) const;

    bool writeProjectedLas(
        const std::filesystem::path& sessionDirectory,
        const std::vector<ProjectedLidarPoint>& points,
        int targetEpsgCode
    ) const;
};