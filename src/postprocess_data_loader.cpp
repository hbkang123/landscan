#include "app_config.hpp"
#include "postprocess_data_loader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iterator>

namespace {

std::vector<std::string> splitCsvLine(
    const std::string& line
) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;

    while(std::getline(stream, field, ',')) {
        if(!field.empty() && field.back() == '\r') {
            field.pop_back();
        }

        fields.push_back(field);
    }

    return fields;
}

bool findColumn(
    const std::unordered_map<std::string, size_t>& columns,
    const std::string& name,
    size_t& index
) {
    const auto iterator = columns.find(name);

    if(iterator == columns.end()) {
        return false;
    }

    index = iterator->second;
    return true;
}

}

CsvLoadResult PostprocessDataLoader::loadAttitudeSamples(
    const std::string& csvPath,
    std::vector<TimedAttitudeSample>& samples
) const {
    CsvLoadResult result{};
    samples.clear();

    std::ifstream file(csvPath);

    if(!file.is_open()) {
        result.errorMessage =
            "Failed to open attitude CSV: " + csvPath;

        return result;
    }

    std::string headerLine;

    if(!std::getline(file, headerLine)) {
        result.errorMessage =
            "Attitude CSV header is missing: " + csvPath;

        return result;
    }

    const std::vector<std::string> headerFields =
        splitCsvLine(headerLine);

    std::unordered_map<std::string, size_t> columns;

    for(size_t index = 0; index < headerFields.size(); ++index) {
        columns[headerFields[index]] = index;
    }

    size_t timestampIndex = 0;
    size_t successIndex = 0;
    size_t q0Index = 0;
    size_t q1Index = 0;
    size_t q2Index = 0;
    size_t q3Index = 0;

    const bool requiredColumnsExist =
        findColumn(columns, "topic_time_us", timestampIndex)
        && findColumn(columns, "success", successIndex)
        && findColumn(columns, "q0_w", q0Index)
        && findColumn(columns, "q1_x", q1Index)
        && findColumn(columns, "q2_y", q2Index)
        && findColumn(columns, "q3_z", q3Index);

    if(!requiredColumnsExist) {
        result.errorMessage =
            "Attitude CSV required columns are missing: "
            + csvPath;

        return result;
    }

    const size_t maximumRequiredIndex =
        std::max({
            timestampIndex,
            successIndex,
            q0Index,
            q1Index,
            q2Index,
            q3Index
        });

    std::string line;

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        const std::vector<std::string> fields =
            splitCsvLine(line);

        if(fields.size() <= maximumRequiredIndex) {
            ++result.skippedRowCount;
            continue;
        }

        try {
            const int success =
                std::stoi(fields[successIndex]);

            if(success != 1) {
                ++result.skippedRowCount;
                continue;
            }

            TimedAttitudeSample sample{};

            sample.timestampUs =
                static_cast<uint64_t>(
                    std::stoull(fields[timestampIndex])
                );

            sample.q0W = std::stod(fields[q0Index]);
            sample.q1X = std::stod(fields[q1Index]);
            sample.q2Y = std::stod(fields[q2Index]);
            sample.q3Z = std::stod(fields[q3Index]);

            if(sample.timestampUs == 0
                || !std::isfinite(sample.q0W)
                || !std::isfinite(sample.q1X)
                || !std::isfinite(sample.q2Y)
                || !std::isfinite(sample.q3Z)) {

                ++result.skippedRowCount;
                continue;
            }

            const double quaternionNorm =
                std::sqrt(
                    sample.q0W * sample.q0W
                    + sample.q1X * sample.q1X
                    + sample.q2Y * sample.q2Y
                    + sample.q3Z * sample.q3Z
                );

            if(quaternionNorm <= 1.0e-12) {
                ++result.skippedRowCount;
                continue;
            }

            sample.q0W /= quaternionNorm;
            sample.q1X /= quaternionNorm;
            sample.q2Y /= quaternionNorm;
            sample.q3Z /= quaternionNorm;

            samples.push_back(sample);
            ++result.loadedRowCount;
        }
        catch(...) {
            ++result.skippedRowCount;
        }
    }

    std::sort(
        samples.begin(),
        samples.end(),
        [](
            const TimedAttitudeSample& left,
            const TimedAttitudeSample& right
        ) {
            return left.timestampUs < right.timestampUs;
        }
    );

    const auto uniqueEnd =
        std::unique(
            samples.begin(),
            samples.end(),
            [](
                const TimedAttitudeSample& left,
                const TimedAttitudeSample& right
            ) {
                return left.timestampUs == right.timestampUs;
            }
        );

    const size_t duplicateCount =
        static_cast<size_t>(
            std::distance(uniqueEnd, samples.end())
        );

    samples.erase(uniqueEnd, samples.end());

    result.skippedRowCount += duplicateCount;
    result.loadedRowCount = samples.size();

    if(samples.empty()) {
        result.errorMessage =
            "No valid attitude samples were loaded: "
            + csvPath;

        return result;
    }

    result.success = true;
    return result;
}

CsvLoadResult PostprocessDataLoader::loadRtkPositionSamples(const std::string& csvPath, std::vector<TimedPositionSample>& samples) const {
    CsvLoadResult result{};
    samples.clear();

    std::ifstream file(csvPath);

    if(!file.is_open()) {
        result.errorMessage = "Failed to open RTK position CSV: " + csvPath;

        return result;
    }

    std::string headerLine;

    if(!std::getline(file, headerLine)) {
        result.errorMessage = "RTK position CSV header is missing: " + csvPath;

        return result;
    }

    const std::vector<std::string> headerFields = splitCsvLine(headerLine);
    std::unordered_map<std::string, size_t> columns;

    for(size_t index = 0; index < headerFields.size(); ++index) {
        columns[headerFields[index]] = index;
    }

    size_t timestampIndex = 0;
    size_t successIndex = 0;
    size_t longitudeIndex = 0;
    size_t latitudeIndex = 0;
    size_t altitudeIndex = 0;

    const bool requiredColumnsExist = 
        findColumn(columns, "topic_time_us", timestampIndex)
        && findColumn(columns, "success", successIndex)
        && findColumn(columns, "longitude_deg", longitudeIndex)
        && findColumn(columns, "latitude_deg", latitudeIndex)
        && findColumn(columns, "hfsl_m", altitudeIndex);

    if(!requiredColumnsExist) {
        result.errorMessage = "RTK position CSV required columns are missing: " + csvPath;

        return result;
    }

    const size_t maximumRequiredIndex = std::max({timestampIndex, successIndex, longitudeIndex, latitudeIndex, altitudeIndex});

    std::string line;

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = splitCsvLine(line);

        if(fields.size() <= maximumRequiredIndex) {
            ++result.skippedRowCount;
            continue;
        }

        try{
            const int success = std::stoi(fields[successIndex]);

            if(success != 1) {
                ++result.skippedRowCount;
                continue;
            }

            TimedPositionSample sample{};

            sample.timestampUs = static_cast<uint64_t>(std::stoull(fields[timestampIndex]));
            sample.longitudeDeg = std::stod(fields[longitudeIndex]);
            sample.latitudeDeg = std::stod(fields[latitudeIndex]);
            sample.altitudeM = std::stod(fields[altitudeIndex]);
            sample.source = PositionSource::RTK;

            const bool coordinateFinite = std::isfinite(sample.longitudeDeg) && std::isfinite(sample.latitudeDeg) && std::isfinite(sample.altitudeM);
            const bool coordinateRangeValid = sample.longitudeDeg >= -180.0 && sample.longitudeDeg <= 180.0 && sample.latitudeDeg >= -90.0 && sample.latitudeDeg <= 90.0;
            const bool coordinateNotZero = std::abs(sample.longitudeDeg) > 1.0e-12 || std::abs(sample.latitudeDeg) > 1.0e-12;

            if(sample.timestampUs == 0 || !coordinateFinite || !coordinateRangeValid || !coordinateNotZero) {
                ++result.skippedRowCount;
                continue;
            }

            samples.push_back(sample);
            ++result.loadedRowCount;
        }
        catch(...) {
            ++result.skippedRowCount;
        }
    }

    std::sort(samples.begin(), samples.end(), [](const TimedPositionSample& left, const TimedPositionSample& right) {
        return left.timestampUs < right.timestampUs;
    });

    const auto uniqueEnd = std::unique(samples.begin(), samples.end(), [](const TimedPositionSample& left, const TimedPositionSample& right) {
        return left.timestampUs == right.timestampUs;
    });

    const size_t duplicateCount = static_cast<size_t>(std::distance(uniqueEnd, samples.end()));
    samples.erase(uniqueEnd, samples.end());

    result.skippedRowCount += duplicateCount;
    result.loadedRowCount = samples.size();

    if(samples.empty()) {
        result.errorMessage = "No valid RTK position samples were loaded: " + csvPath;

        return result;
    }

    result.success = true;
    return result;
}

CsvLoadResult PostprocessDataLoader::loadRtkStatusSamples(const std::string& csvPath, std::vector<TimedRtkStatusSample>& samples) const {
    CsvLoadResult result{};
    samples.clear();

    std::ifstream file(csvPath);

    if(!file.is_open()) {
        result.errorMessage = "Failed to open RTK status CSV: " + csvPath;

        return result;
    }

    std::string headerLine;

    if(!std::getline(file, headerLine)) {
        result.errorMessage = "RTK status CSV header is missing: " + csvPath;

        return result;
    }

    const std::vector<std::string> headerFields = splitCsvLine(headerLine);

    std::unordered_map<std::string, size_t> columns;

    for(size_t index = 0; index < headerFields.size(); ++index) {
        columns[headerFields[index]] = index;
    }

    size_t timestampIndex = 0;
    size_t successIndex = 0;
    size_t positionInfoIndex = 0;

    const bool requiredColumnsExist = 
        findColumn(columns, "topic_time_us", timestampIndex)
        && findColumn(columns, "success", successIndex)
        && findColumn(columns, "position_info", positionInfoIndex);

    if(!requiredColumnsExist) {
        result.errorMessage = "RTK status CSV required columns are missing: " + csvPath;

        return result;
    }

    const size_t maximumRequiredIndex = std::max({timestampIndex, successIndex, positionInfoIndex});

    std::string line;

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = splitCsvLine(line);

        if(fields.size() <= maximumRequiredIndex) {
            ++result.skippedRowCount;
            continue;
        }

        try{
            const int success = std::stoi(fields[successIndex]);

            if(success != 1) {
                ++result.skippedRowCount;
                continue;
            }

            const unsigned long rawPositionInfo = std::stoul(fields[positionInfoIndex]);

            if(rawPositionInfo > 255UL) {
                ++result.skippedRowCount;
                continue;
            }

            TimedRtkStatusSample sample{};

            sample.timestampUs = static_cast<uint64_t>(std::stoull(fields[timestampIndex]));
            sample.positionInfo = static_cast<uint8_t>(rawPositionInfo);

            if(sample.timestampUs == 0) {
                ++result.skippedRowCount;
                continue;
            }

            samples.push_back(sample);
            ++result.loadedRowCount;
        }
        catch(...) {
            ++result.skippedRowCount;
        }
    }

    std::sort(samples.begin(), samples.end(), [](const TimedRtkStatusSample& left, const TimedRtkStatusSample& right){
        return left.timestampUs < right.timestampUs;
    });

    const auto uniqueEnd = std::unique(samples.begin(), samples.end(), [](const TimedRtkStatusSample& left, const TimedRtkStatusSample& right){
        return left.timestampUs == right.timestampUs;
    });

    const size_t duplicateCount = static_cast<size_t>(std::distance(uniqueEnd, samples.end()));

    samples.erase(uniqueEnd, samples.end());

    result.skippedRowCount += duplicateCount;
    result.loadedRowCount = samples.size();

    if(samples.empty()) {
        result.errorMessage = "No valid RTK status samples were loaded: " + csvPath;

        return result;
    }

    result.success = true;
    return result;
}

CsvLoadResult PostprocessDataLoader::loadGpsPositionSamples(const std::string& csvPath, std::vector<TimedPositionSample>& samples) const {
    CsvLoadResult result{};
    samples.clear();

    std::ifstream file(csvPath);

    if(!file.is_open()) {
        result.errorMessage = "Failed to open GPS position CSV: " + csvPath;

        return result;
    }

    std::string headerLine;

    if(!std::getline(file, headerLine)) {
        result.errorMessage = "GPS position CSV header is missing: " + csvPath;

        return result;
    }

    const std::vector<std::string> headerFields = splitCsvLine(headerLine);

    std::unordered_map<std::string, size_t> columns;

    for(size_t index = 0; index < headerFields.size(); ++index) {
        columns[headerFields[index]] = index;
    }

    size_t timestampIndex = 0;
    size_t successIndex = 0;
    size_t longitudeIndex = 0;
    size_t latitudeIndex = 0;
    size_t altitudeIndex = 0;

    const bool requiredColumnExist = 
        findColumn(columns, "topic_time_us", timestampIndex)
        && findColumn(columns, "success", successIndex)
        && findColumn(columns, "longitude_deg_e7", longitudeIndex)
        && findColumn(columns, "latitude_deg_e7", latitudeIndex)
        && findColumn(columns, "altitude_mm", altitudeIndex);

    if(!requiredColumnExist) {
        result.errorMessage = "GPS position CSV required columns are missing: " + csvPath;

        return result;
    }

    const size_t maximumRequiredIndex = std::max({timestampIndex, successIndex, longitudeIndex, latitudeIndex, altitudeIndex});

    std::string line;

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = splitCsvLine(line);

        if(fields.size() <= maximumRequiredIndex) {
            ++result.skippedRowCount;
            continue;
        }

        try {
            const int success = std::stoi(fields[successIndex]);

            if(success != 1) { 
                ++result.skippedRowCount;
                continue;
            }

            const int64_t longitudeE7 = static_cast<int64_t>(std::stoll(fields[longitudeIndex]));
            const int64_t latitudeE7  = static_cast<int64_t>(std::stoll(fields[latitudeIndex]));
            const int64_t altitudeMm  = static_cast<int64_t>(std::stoll(fields[altitudeIndex]));

            TimedPositionSample sample{};

            sample.timestampUs = static_cast<uint64_t>(std::stoull(fields[timestampIndex]));
            sample.longitudeDeg = static_cast<double>(longitudeE7) * 0.0000001;
            sample.latitudeDeg = static_cast<double>(latitudeE7) * 0.0000001;
            sample.altitudeM = static_cast<double>(altitudeMm) / 1000.0;

            sample.source = PositionSource::GNSS;

            const bool coordinateFinite = std::isfinite(sample.longitudeDeg) && std::isfinite(sample.latitudeDeg) && std::isfinite(sample.altitudeM);
            const bool coordinateRangeValid = sample.longitudeDeg >= -180.0 && sample.longitudeDeg <= 180.0 && sample.latitudeDeg >= -90.0 && sample.latitudeDeg <= 90.0;
            const bool coordinateNotZero = std::abs(sample.longitudeDeg) > 1.0e-12 || std::abs(sample.latitudeDeg) > 1.0e-12;

            if(sample.timestampUs == 0 || !coordinateFinite || !coordinateRangeValid || !coordinateNotZero) {
                ++result.skippedRowCount;
                continue;
            }

            samples.push_back(sample);
            ++result.loadedRowCount;
        }
        catch(...) {
            ++result.skippedRowCount;
        }
    }

    std::sort(
        samples.begin(), 
        samples.end(), 
        [](const TimedPositionSample& left, const TimedPositionSample& right){
            return left.timestampUs < right.timestampUs;
        }
    );

    const auto uniqueEnd = std::unique(
        samples.begin(),
        samples.end(),
        [](const TimedPositionSample& left, const TimedPositionSample& right){
            return left.timestampUs == right.timestampUs;
        }
    );

    const size_t duplicateCount = static_cast<size_t>(std::distance(uniqueEnd, samples.end()));

    samples.erase(uniqueEnd, samples.end());

    result.skippedRowCount += duplicateCount;
    result.loadedRowCount = samples.size();

    if(samples.empty()) {
        result.errorMessage = "No valid GPS position samples were loaded: " + csvPath;

        return result;
    }

    result.success = true;
    return result;
}

CsvLoadResult PostprocessDataLoader::loadLidarSamples(const std::string& csvPath, std::vector<TimedLidarSample>& samples) const {
    CsvLoadResult result{};
    samples.clear();

    std::ifstream file(csvPath);

    if(!file.is_open()) {
        result.errorMessage = "Failed to open LiDAR CSV: " + csvPath;

        return result;
    }

    std::string headerLine;

    if(!std::getline(file, headerLine)) {
        result.errorMessage = "LiDAR CSV header is missing: " + csvPath;

        return result;
    }

    const std::vector<std::string> headerFields = splitCsvLine(headerLine);

    std::unordered_map<std::string, size_t> columns;

    for(size_t index = 0; index < headerFields.size(); ++index) {
        columns[headerFields[index]] = index;
    }

    size_t receiveTimestampIndex = 0;
    size_t estimatedTimestampIndex = 0;
    size_t sequenceNumberIndex = 0;
    size_t estimatedPeriodIndex = 0;
    size_t timeEstimateReadyIndex = 0;
    size_t timeDiscontinuityIndex = 0;
    size_t angleIndex = 0;
    size_t distanceIndex = 0;
    size_t signalStrengthIndex = 0;
    size_t measurementValidIndex = 0;

    const bool requiredColumnsExist = 
        findColumn(columns, "receive_osal_time_us", receiveTimestampIndex)
        && findColumn(columns, "sequence_number", sequenceNumberIndex)
        && findColumn(columns, "angle_deg", angleIndex)
        && findColumn(columns, "distance_m", distanceIndex)
        && findColumn(columns, "signal_strength", signalStrengthIndex);

    if(!requiredColumnsExist) {
        result.errorMessage = "LiDAR CSV required columns are missing: " + csvPath;

        return result;
    }

    const bool hasEstimatedTimestamp = findColumn(columns, "estimated_measurement_osal_time_us", estimatedTimestampIndex);
    const bool hasEstimatedPeriod = findColumn(columns, "estimated_period_us", estimatedPeriodIndex);
    const bool hasTimeEstimateReady = findColumn(columns, "time_estimate_ready", timeEstimateReadyIndex);
    const bool hasTimeDiscontinuity = findColumn(columns, "time_discontinuity_detected", timeDiscontinuityIndex);
    const bool hasMeasurementValid = findColumn(columns, "measurement_valid", measurementValidIndex);
 
    size_t maximumRequiredIndex = std::max({
        receiveTimestampIndex,
        sequenceNumberIndex,
        angleIndex,
        distanceIndex,
        signalStrengthIndex
    });

    if(hasEstimatedTimestamp) {
        maximumRequiredIndex = std::max(maximumRequiredIndex, estimatedTimestampIndex);
    }

    if(hasEstimatedPeriod) {
        maximumRequiredIndex = std::max(maximumRequiredIndex, estimatedPeriodIndex);
    }

    if(hasTimeEstimateReady) {
        maximumRequiredIndex = std::max(maximumRequiredIndex, timeEstimateReadyIndex);
    }

    if(hasTimeDiscontinuity) {
        maximumRequiredIndex = std::max(maximumRequiredIndex, timeDiscontinuityIndex);
    }

    if(hasMeasurementValid) {
        maximumRequiredIndex = std::max(maximumRequiredIndex, measurementValidIndex);
    }

    std::string line;

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = splitCsvLine(line);

        if(fields.size() <= maximumRequiredIndex) {
            ++result.skippedRowCount;
            continue;
        }

        try{
            TimedLidarSample sample{};

            sample.receiveTimestampUs = static_cast<uint64_t>(std::stoull(fields[receiveTimestampIndex]));
            sample.timestampUs = sample.receiveTimestampUs;

            if(hasEstimatedTimestamp) {
                const uint64_t estimatedTimestampUs = static_cast<uint64_t>(std::stoull(fields[estimatedTimestampIndex]));

                if(estimatedTimestampUs != 0) {
                    sample.timestampUs = estimatedTimestampUs;
                }
            }

            sample.sequenceNumber = static_cast<uint64_t>(std::stoull(fields[sequenceNumberIndex]));

            if(hasEstimatedPeriod) {
                sample.estimatedPeriodUs = std::stod(fields[estimatedPeriodIndex]);
            }

            if(hasTimeEstimateReady) {
                sample.timeEstimateReady = std::stoi(fields[timeEstimateReadyIndex]) == 1;
            }

            if(hasTimeDiscontinuity) {
                sample.timeDiscontinuityDetected = std::stoi(fields[timeDiscontinuityIndex]) == 1;
            }

            sample.angleDeg = std::stod(fields[angleIndex]);
            sample.distanceM = std::stod(fields[distanceIndex]);
            sample.signalStrength = std::stod(fields[signalStrengthIndex]);

            if(hasMeasurementValid) {
                sample.measurementValid = std::stoi(fields[measurementValidIndex]) == 1;
            }
            else{
                sample.measurementValid = sample.distanceM >= AppConfig::LIDAR_MIN_VALID_DISTANCE_M && sample.distanceM <= AppConfig::LIDAR_MAX_VALID_DISTANCE_M;
            }

            const bool valuesFinite = 
                std::isfinite(sample.angleDeg)
                && std::isfinite(sample.distanceM)
                && std::isfinite(sample.signalStrength)
                && std::isfinite(sample.estimatedPeriodUs);
            
            if(sample.timestampUs == 0 || sample.receiveTimestampUs == 0 || !valuesFinite) {
                ++result.skippedRowCount;
                continue;
            }

            samples.push_back(sample);
            ++result.loadedRowCount;
        }
        catch(...) {
            ++result.skippedRowCount;
        }
    }

    if(samples.empty()) {
        result.errorMessage = "No valid LiDAR samples were loaded: " + csvPath;

        return result;
    }

    result.loadedRowCount = samples.size();
    result.success = true;

    return result;
}