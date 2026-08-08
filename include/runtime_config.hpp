#pragma once

#include "app_config.hpp"

#include <stdint.h>
#include <string>

class RuntimeConfig
{
public:
    bool loadFromFile(
        const std::string& filePath,
        std::string& errorMessage
    );

    bool saveToFile(
        const std::string& filePath,
        std::string& errorMessage
    ) const;

    bool validate(
        std::string& errorMessage
    ) const;

    bool getLidarUpdateRateCommand(
        uint8_t& commandValue
    ) const;

    std::string lidarPort =
        AppConfig::LIDAR_PORT;

    int lidarBaudrate =
        AppConfig::LIDAR_BAUDRATE;

    double lidarUpdateRateHz =
        AppConfig::LIDAR_UPDATE_RATE_HZ;

    float lidarScanLowAngleDeg =
        AppConfig::LIDAR_SCAN_LOW_ANGLE_DEG;

    float lidarScanHighAngleDeg =
        AppConfig::LIDAR_SCAN_HIGH_ANGLE_DEG;

    double lidarOffsetForwardM =
        AppConfig::LIDAR_OFFSET_FORWARD_M;

    double lidarOffsetRightM =
        AppConfig::LIDAR_OFFSET_RIGHT_M;

    double lidarOffsetDownM =
        AppConfig::LIDAR_OFFSET_DOWN_M;

    int sourceGeographicEpsgCode =
        AppConfig::SOURCE_GEOGRAPHIC_EPSG_CODE;

    int outputProjectedEpsgCode =
        AppConfig::OUTPUT_PROJECTED_EPSG_CODE;
};