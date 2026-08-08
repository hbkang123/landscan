#include "runtime_config.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <iomanip>

namespace {

std::string trim(const std::string& value)
{
    const size_t first = value.find_first_not_of(" \t\r\n");

    if(first == std::string::npos) {
        return "";
    }

    const size_t last = value.find_last_not_of(" \t\r\n");

    return value.substr(first, last - first + 1);
}

bool parseInteger(
    const std::string& value,
    int& parsedValue
) {
    try {
        size_t parsedLength = 0;

        const long long result = std::stoll(
            value,
            &parsedLength,
            10
        );

        if(parsedLength != value.size()) {
            return false;
        }

        if(
            result < std::numeric_limits<int>::min()
            || result > std::numeric_limits<int>::max()
        ) {
            return false;
        }

        parsedValue = static_cast<int>(result);

        return true;
    }
    catch(...) {
        return false;
    }
}

bool parseDouble(
    const std::string& value,
    double& parsedValue
) {
    try {
        size_t parsedLength = 0;

        const double result = std::stod(
            value,
            &parsedLength
        );

        if(
            parsedLength != value.size()
            || !std::isfinite(result)
        ) {
            return false;
        }

        parsedValue = result;

        return true;
    }
    catch(...) {
        return false;
    }
}

bool parseFloat(
    const std::string& value,
    float& parsedValue
) {
    try {
        size_t parsedLength = 0;

        const float result = std::stof(
            value,
            &parsedLength
        );

        if(
            parsedLength != value.size()
            || !std::isfinite(result)
        ) {
            return false;
        }

        parsedValue = result;

        return true;
    }
    catch(...) {
        return false;
    }
}

}

bool RuntimeConfig::loadFromFile(
    const std::string& filePath,
    std::string& errorMessage
) {
    errorMessage.clear();

    std::ifstream file(filePath);

    if(!file.is_open()) {
        errorMessage =
            "Failed to open configuration file: "
            + filePath;

        return false;
    }

    RuntimeConfig loadedConfig = *this;

    std::unordered_set<std::string> loadedKeys;

    std::string line;
    size_t lineNumber = 0;

    while(std::getline(file, line)) {
        ++lineNumber;

        const size_t commentPosition = line.find('#');

        if(commentPosition != std::string::npos) {
            line = line.substr(0, commentPosition);
        }

        line = trim(line);

        if(line.empty()) {
            continue;
        }

        const size_t separatorPosition = line.find('=');

        if(separatorPosition == std::string::npos) {
            errorMessage =
                "Configuration separator '=' is missing at line "
                + std::to_string(lineNumber);

            return false;
        }

        const std::string key = trim(
            line.substr(0, separatorPosition)
        );

        const std::string value = trim(
            line.substr(separatorPosition + 1)
        );

        if(key.empty()) {
            errorMessage =
                "Configuration key is empty at line "
                + std::to_string(lineNumber);

            return false;
        }

        if(value.empty()) {
            errorMessage =
                "Configuration value is empty for key '"
                + key
                + "' at line "
                + std::to_string(lineNumber);

            return false;
        }

        if(loadedKeys.find(key) != loadedKeys.end()) {
            errorMessage =
                "Duplicate configuration key '"
                + key
                + "' at line "
                + std::to_string(lineNumber);

            return false;
        }

        loadedKeys.insert(key);

        if(key == "lidar_port") {
            loadedConfig.lidarPort = value;
        }
        else if(key == "lidar_baudrate") {
            if(!parseInteger(
                value,
                loadedConfig.lidarBaudrate
            )) {
                errorMessage =
                    "Invalid lidar_baudrate at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_update_rate_hz") {
            if(!parseDouble(
                value,
                loadedConfig.lidarUpdateRateHz
            )) {
                errorMessage =
                    "Invalid lidar_update_rate_hz at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_scan_low_angle_deg") {
            if(!parseFloat(
                value,
                loadedConfig.lidarScanLowAngleDeg
            )) {
                errorMessage =
                    "Invalid lidar_scan_low_angle_deg at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_scan_high_angle_deg") {
            if(!parseFloat(
                value,
                loadedConfig.lidarScanHighAngleDeg
            )) {
                errorMessage =
                    "Invalid lidar_scan_high_angle_deg at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_offset_forward_m") {
            if(!parseDouble(
                value,
                loadedConfig.lidarOffsetForwardM
            )) {
                errorMessage =
                    "Invalid lidar_offset_forward_m at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_offset_right_m") {
            if(!parseDouble(
                value,
                loadedConfig.lidarOffsetRightM
            )) {
                errorMessage =
                    "Invalid lidar_offset_right_m at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "lidar_offset_down_m") {
            if(!parseDouble(
                value,
                loadedConfig.lidarOffsetDownM
            )) {
                errorMessage =
                    "Invalid lidar_offset_down_m at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "source_geographic_epsg_code") {
            if(!parseInteger(
                value,
                loadedConfig.sourceGeographicEpsgCode
            )) {
                errorMessage =
                    "Invalid source_geographic_epsg_code at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else if(key == "output_projected_epsg_code") {
            if(!parseInteger(
                value,
                loadedConfig.outputProjectedEpsgCode
            )) {
                errorMessage =
                    "Invalid output_projected_epsg_code at line "
                    + std::to_string(lineNumber);

                return false;
            }
        }
        else {
            errorMessage =
                "Unknown configuration key '"
                + key
                + "' at line "
                + std::to_string(lineNumber);

            return false;
        }
    }

    if(!loadedConfig.validate(errorMessage)) {
        return false;
    }

    *this = loadedConfig;

    return true;
}

bool RuntimeConfig::saveToFile(
    const std::string& filePath,
    std::string& errorMessage
) const {
    errorMessage.clear();

    std::string validationError;

    if(!validate(validationError)) {
        errorMessage =
            "Cannot save invalid configuration: "
            + validationError;

        return false;
    }

    uint8_t updateRateCommand = 0;

    if(!getLidarUpdateRateCommand(
        updateRateCommand
    )) {
        errorMessage =
            "Cannot determine SF45 update rate command";

        return false;
    }

    std::ofstream file(
        filePath,
        std::ios::out | std::ios::trunc
    );

    if(!file.is_open()) {
        errorMessage =
            "Failed to create configuration file: "
            + filePath;

        return false;
    }

    file << std::setprecision(15);

    file
        << "# LandScan session acquisition configuration"
        << '\n'
        << '\n'
        << "# LiDAR connection"
        << '\n'
        << "lidar_port="
        << lidarPort
        << '\n'
        << "lidar_baudrate="
        << lidarBaudrate
        << '\n'
        << '\n'
        << "# SF45/B measurement settings"
        << '\n'
        << "lidar_update_rate_hz="
        << lidarUpdateRateHz
        << '\n'
        //<< "lidar_update_rate_command="
        << "# calculated_lidar_update_rate_command="
        << static_cast<unsigned int>(
            updateRateCommand
        )
        << '\n'
        << "lidar_scan_low_angle_deg="
        << lidarScanLowAngleDeg
        << '\n'
        << "lidar_scan_high_angle_deg="
        << lidarScanHighAngleDeg
        << '\n'
        << '\n'
        << "# LiDAR mounting offset in DJI body FRD coordinates"
        << '\n'
        << "lidar_offset_forward_m="
        << lidarOffsetForwardM
        << '\n'
        << "lidar_offset_right_m="
        << lidarOffsetRightM
        << '\n'
        << "lidar_offset_down_m="
        << lidarOffsetDownM
        << '\n'
        << '\n'
        << "# Coordinate reference systems"
        << '\n'
        << "source_geographic_epsg_code="
        << sourceGeographicEpsgCode
        << '\n'
        << "output_projected_epsg_code="
        << outputProjectedEpsgCode
        << '\n';

    file.close();

    if(!file) {
        errorMessage =
            "Failed while writing configuration file: "
            + filePath;

        return false;
    }

    return true;
}

bool RuntimeConfig::validate(
    std::string& errorMessage
) const {
    errorMessage.clear();

    if(lidarPort.empty()) {
        errorMessage = "lidar_port must not be empty";

        return false;
    }

    if(
        lidarBaudrate <= 0
        || lidarBaudrate > 10000000
    ) {
        errorMessage = "lidar_baudrate is out of range";

        return false;
    }

    uint8_t updateRateCommand = 0;

    if(!getLidarUpdateRateCommand(updateRateCommand)) {
        errorMessage =
            "lidar_update_rate_hz is not supported by SF45/B";

        return false;
    }

    if(
        !std::isfinite(lidarScanLowAngleDeg)
        || !std::isfinite(lidarScanHighAngleDeg)
    ) {
        errorMessage = "LiDAR scan angles must be finite";

        return false;
    }

    if(
        lidarScanLowAngleDeg < -160.0f
        || lidarScanHighAngleDeg > 160.0f
        || lidarScanLowAngleDeg >= lidarScanHighAngleDeg
    ) {
        errorMessage =
            "LiDAR scan angle range is invalid";

        return false;
    }

    const bool offsetsFinite =
        std::isfinite(lidarOffsetForwardM)
        && std::isfinite(lidarOffsetRightM)
        && std::isfinite(lidarOffsetDownM);

    if(!offsetsFinite) {
        errorMessage =
            "LiDAR offsets must be finite";

        return false;
    }

    if(
        std::abs(lidarOffsetForwardM) > 10.0
        || std::abs(lidarOffsetRightM) > 10.0
        || std::abs(lidarOffsetDownM) > 10.0
    ) {
        errorMessage =
            "LiDAR offsets are outside the allowed range";

        return false;
    }

    if(sourceGeographicEpsgCode <= 0) {
        errorMessage =
            "source_geographic_epsg_code must be positive";

        return false;
    }

    if(outputProjectedEpsgCode <= 0) {
        errorMessage =
            "output_projected_epsg_code must be positive";

        return false;
    }

    return true;
}

bool RuntimeConfig::getLidarUpdateRateCommand(
    uint8_t& commandValue
) const {
    struct UpdateRateEntry
    {
        double rateHz;
        uint8_t commandValue;
    };

    static constexpr UpdateRateEntry entries[] = {
        {50.0, 1},
        {100.0, 2},
        {200.0, 3},
        {400.0, 4},
        {500.0, 5},
        {625.0, 6},
        {1000.0, 7},
        {1250.0, 8},
        {1538.0, 9},
        {2000.0, 10},
        {2500.0, 11},
        {5000.0, 12}
    };

    for(const UpdateRateEntry& entry : entries) {
        if(
            std::abs(lidarUpdateRateHz - entry.rateHz)
            <= 0.001
        ) {
            commandValue = entry.commandValue;

            return true;
        }
    }

    commandValue = 0;

    return false;
}