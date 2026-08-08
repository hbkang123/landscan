#pragma once

#include "postprocess_types.hpp"

#include <stddef.h>
#include <string>
#include <vector>

struct CsvLoadResult
{
    bool success = false;

    size_t loadedRowCount = 0;
    size_t skippedRowCount = 0;

    std::string errorMessage;
};

class PostprocessDataLoader
{
public:
    CsvLoadResult loadAttitudeSamples(const std::string& csvPath, std::vector<TimedAttitudeSample>& sample) const;
    CsvLoadResult loadGpsPositionSamples(const std::string& csvPath, std::vector<TimedPositionSample>& samples) const;
    CsvLoadResult loadRtkPositionSamples(const std::string& csvPath, std::vector<TimedPositionSample>& samples) const;
    CsvLoadResult loadRtkStatusSamples(const std::string& csvPath, std::vector<TimedRtkStatusSample>& samples) const;
    CsvLoadResult loadLidarSamples(const std::string& csvPath, std::vector<TimedLidarSample>& sample) const;
};