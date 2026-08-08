/******************************************************************************
 * PpsManager
 *
 * Responsibility
 *  - Detect PPS rising edge on GPIO16
 *  - Store the latest PPS trigger time using DJI OSAL time
 *  - Provide the latest PPS trigger time to TimeSyncManager
 ******************************************************************************/
#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

class PpsManager {
public:
    bool initialize();
    void shutdown();
    
    bool hasPps() const;
    uint64_t getLastPpsTimeUs() const;

private:
    void ppsLoop();
    uint64_t getNowUs() const;

private:
    std::atomic<bool> running{false};
    std::atomic<bool> ppsDetected{false};
    std::atomic<uint64_t> lastPpsTimeUs{0};

    std::thread workerThread;
};