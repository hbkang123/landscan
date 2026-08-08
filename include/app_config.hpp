#pragma once

#include <stdint.h>

class AppConfig
{
    public:
        static constexpr const char* APP_NAME = "LandScan";
        static constexpr const char* LIDAR_PORT = "/dev/ttyACM0";
        static constexpr int LIDAR_BAUDRATE = 921600;
        static constexpr uint8_t LIDAR_UPDATE_RATE_COMMAND = 5;
        static constexpr double LIDAR_UPDATE_RATE_HZ = 500.0;

        static constexpr uint32_t LIDAR_OUTPUT_FORMAT = 0x185U;

        static constexpr float LIDAR_MIN_VALID_DISTANCE_M = 0.2f;
        static constexpr float LIDAR_MAX_VALID_DISTANCE_M = 50.0f;

        //static constexpr float LIDAR_SCAN_LOW_ANGLE_DEG = -90.0f;
        //static constexpr float LIDAR_SCAN_HIGH_ANGLE_DEG = 90.0f;

        static constexpr float LIDAR_SCAN_LOW_ANGLE_DEG = -45.0f;
        static constexpr float LIDAR_SCAN_HIGH_ANGLE_DEG = 45.0f;

        static constexpr uint32_t LIDAR_STREAM_STOP_VALUE = 0U;
        static constexpr uint32_t LIDAR_STREAM_START_VALUE = 5U;

        static constexpr uint32_t LIDAR_RECEIVE_TIMEOUT_MS = 1000U;
        
        static constexpr uint64_t POSITION_INTERPOLATION_MAX_GAP_US = 250000ULL;

        static constexpr uint64_t ATTITUDE_INTERPOLATION_MAX_GAP_US = 250000ULL;

        static constexpr uint64_t RTK_STATUS_MAX_AGE_US = 250000ULL;

        static constexpr double LIDAR_OFFSET_FORWARD_M = 0.0;
        static constexpr double LIDAR_OFFSET_RIGHT_M = 0.0;
        static constexpr double LIDAR_OFFSET_DOWN_M = 0.0;

        static constexpr int SOURCE_GEOGRAPHIC_EPSG_CODE = 4326;
        static constexpr int OUTPUT_PROJECTED_EPSG_CODE = 5186;
};