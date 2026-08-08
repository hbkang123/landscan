/******************************************************************************
 * TimeSyncManager
 *
 * Responsibility
 *  - Initialize DJI Time Sync
 *  - Register the latest PPS time callback
 *  - Convert local OSAL time to aircraft time
 ******************************************************************************/
#pragma once

#include <atomic>
#include <cstdint>

#include <dji_typedef.h>

class PpsManager;

class TimeSyncManager {
public:
    void setPpsManager(PpsManager* manager);

    bool initialize();
    void shutdown();

    bool isSyncronized() const;
    uint64_t getCurrentAircraftTimeUs();

private:
    static T_DjiReturnCode getNewestPpsTriggerTimeUs(uint64_t* localTimeUs);

    uint64_t getCurrentLocalTimeUs() const;

private:
    static TimeSyncManager* instance;

    PpsManager* ppsManager{nullptr};
    
    std::atomic<bool> initialized{false};
    std::atomic<bool> synchronized{false};
    mutable std::atomic<uint32_t> lastSynchAttemptTimeMs{0};
};