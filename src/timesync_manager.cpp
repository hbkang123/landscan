#include "timesync_manager.hpp"
#include "pps_manager.hpp"

#include <dji_error.h>
#include <dji_platform.h>
#include <dji_time_sync.h>

#include <iostream>

TimeSyncManager* TimeSyncManager::instance = nullptr;

void TimeSyncManager::setPpsManager(PpsManager* manager) {
    ppsManager = manager;
}

bool TimeSyncManager::initialize() {
    if(initialized.load()) {
        return true;
    }

    if(ppsManager == nullptr) {
        std::cerr << "[TimeSync] PpsManager is not configured" << std::endl;
        return false;
    }

    std::cout << "[TimeSync] Initialize" << std::endl;

    synchronized.store(false);
    lastSynchAttemptTimeMs.store(0);

    instance = this;

    T_DjiReturnCode returnCode = DjiTimeSync_Init();

    std::cout << "[TimeSync] DjiTimeSync_Init result: 0x" << std::hex << returnCode << std::dec << std::endl;

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        instance = nullptr;
        return false;
    }

    returnCode = DjiTimeSync_RegGetNewestPpsTriggerTimeCallback(TimeSyncManager::getNewestPpsTriggerTimeUs);

    std::cout << "[TimeSync] Reg PPS callback result: 0x" << std::hex << returnCode << std::dec << std::endl;

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        instance = nullptr;
        return false;
    }

    initialized.store(true);

    return true;
}

void TimeSyncManager::shutdown() {
    if(!initialized.load()) {
        return;
    }

    initialized.store(false);
    synchronized.store(false);
    lastSynchAttemptTimeMs.store(0);

    instance = nullptr;

    std::cout << "[TimeSync] Shutdown" <<std::endl;
}

bool TimeSyncManager::isSyncronized() const {
    return synchronized.load();
}

T_DjiReturnCode TimeSyncManager::getNewestPpsTriggerTimeUs(uint64_t* localTimeUs) {
    if(localTimeUs == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    if(instance == nullptr || instance->ppsManager == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }

    if(!instance->ppsManager->hasPps()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }

    const uint64_t newestPpsTimeUs = instance->ppsManager->getLastPpsTimeUs();

    if(newestPpsTimeUs == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }

    *localTimeUs = newestPpsTimeUs;

    std::cout << "[TimeSync] Callback PPS localTimeUs=" << *localTimeUs <<std::endl;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

uint64_t TimeSyncManager::getCurrentLocalTimeUs() const {
    T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();

    if(osalHandler == nullptr || osalHandler->GetTimeUs == nullptr) {
        std::cerr << "[TimeSync] OSAL GetTimeUs is unavailable" << std::endl;
        return 0;
    }

    uint64_t currentTimeUs = 0;

    const T_DjiReturnCode returnCode = osalHandler->GetTimeUs(&currentTimeUs);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[TimeSync] OSAL GetTimeUs failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return 0;
    }

    return currentTimeUs;
}

uint64_t TimeSyncManager::getCurrentAircraftTimeUs() {
    if(!initialized.load() || ppsManager == nullptr || !ppsManager->hasPps()) {
        return 0;
    }

    T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();

    if(osalHandler == nullptr || osalHandler->GetTimeMs == nullptr || osalHandler->GetTimeUs == nullptr) {
        return 0;
    }

    uint32_t currentTimeMs = 0;

    T_DjiReturnCode returnCode = osalHandler->GetTimeMs(&currentTimeMs);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 0;
    }

    if(!synchronized.load()) {
        uint32_t previousAttemptMs = lastSynchAttemptTimeMs.load();

        if(previousAttemptMs != 0 && currentTimeMs - previousAttemptMs < 500U) {
            return 0;
        }

        if(!lastSynchAttemptTimeMs.compare_exchange_strong(previousAttemptMs, currentTimeMs)) {
            return 0;
        }
    }

    //const uint64_t getCurrentLocalTimeUs = static_cast<uint64_t>(currentTimeMs) * 1000ULL;

    uint64_t currentLocalTimeUs = 0;

    returnCode = osalHandler->GetTimeUs(&currentLocalTimeUs);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 0;
    }

    T_DjiTimeSyncAircraftTime aircraftTime{};

    returnCode = DjiTimeSync_TransferToAircraftTime(currentLocalTimeUs, &aircraftTime);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        if(synchronized.load()) {
            std::cerr << "[TimeSync] TransferToAircraftTime failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        }

        return 0;
    }

    if(!synchronized.exchange(true)) {
        std::cout << "[TimeSync] Aircraft time synchronization ready" << std::endl;
    }

    return static_cast<uint64_t>(aircraftTime.hour) * 3600ULL * 1000000ULL
        + static_cast<uint64_t>(aircraftTime.minute) * 60ULL * 1000000ULL
        + static_cast<uint64_t>(aircraftTime.second) * 1000000ULL
        + static_cast<uint64_t>(aircraftTime.microsecond);
}