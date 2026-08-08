#pragma once

#include "payload_manager.hpp"
#include "positioning_manager.hpp"
#include "navigation_manager.hpp"
#include "timesync_manager.hpp"
#include "pps_manager.hpp"
#include "lidar_manager.hpp"
#include "postprocess_manager.hpp"
#include "runtime_config.hpp"

#include <atomic>
#include <thread>

class LandScanApplication
{
public:
    bool initialize(
        const RuntimeConfig& runtimeConfig
    );
    void shutdown();

private:
    PayloadManager payloadManager;
    PositioningManager positioningManager;
    PpsManager ppsManager;
    NavigationManager navigationManager;
    TimeSyncManager timeSyncManager;
    LidarManager lidarManager;
    PostprocessManager postprocessManager;

    std::atomic<bool> lidarRunning{false};
    std::thread lidarThread;
};