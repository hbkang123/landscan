/******************************************************************************
 * LidarManager
 *
 * Responsibility
 *  - Receive LiDAR points
 *  - Timestamp each point
 *  - Estimate the measurement time
 ******************************************************************************/
#pragma once

#include "app_config.hpp"
#include "runtime_config.hpp"
#include "lidar_point.hpp"
#include "lidar_time_estimator.hpp"

#include <stdint.h>
#include <string>

class lwSerialPort;

class LidarManager
{
public:
    LidarManager();
    ~LidarManager();

    bool configure(
        const RuntimeConfig& runtimeConfig
    );

    bool init(
        const std::string& portName,
        int baudrate
    );

    bool start();
    void stop();

    bool readPoint(LidarPoint& point);

private:
    uint16_t readUInt16(
        uint8_t* buffer,
        uint32_t offset
    );

private:
    bool running_ = false;

    uint64_t nextSequenceNumber_ = 0;

    RuntimeConfig runtimeConfig_{};

    uint8_t updateRateCommand_ =
        AppConfig::LIDAR_UPDATE_RATE_COMMAND;

    LidarTimeEstimator timeEstimator_{
        AppConfig::LIDAR_UPDATE_RATE_HZ
    };

    lwSerialPort* serial_ = nullptr;
};