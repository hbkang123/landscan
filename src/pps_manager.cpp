#include "pps_manager.hpp"
#include <gpiod.h>
#include <dji_platform.h>
#include <iostream>

namespace {
constexpr const char* GPIO_CHIP_NAME = "gpiochip0";
constexpr unsigned int PPS_GPIO_OFFSET = 16;
constexpr uint64_t PPS_MIN_INTERVAL_US = 800000ULL;
}

bool PpsManager::initialize() {
    if(running.load()) {
        return true;
    }

    std::cout << "[PPS] Initialize GPIO" << PPS_GPIO_OFFSET << " PPS" << std::endl;

    ppsDetected.store(false);
    lastPpsTimeUs.store(0);
    running.store(true);

    workerThread = std::thread(&PpsManager::ppsLoop, this);

    return true;
}

void PpsManager::shutdown() {
    if(!running.load() && !workerThread.joinable()) {
        return;
    }

    running.store(false);

    if(workerThread.joinable()) {
        workerThread.join();
    }

    std::cout << "[PPS] Shutdown" << std::endl;
}

bool PpsManager::hasPps() const {
    return ppsDetected.load();
}

uint64_t PpsManager::getLastPpsTimeUs() const {
    return lastPpsTimeUs.load();
}

uint64_t PpsManager::getNowUs() const {
    T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();

    if(osalHandler == nullptr || osalHandler->GetTimeUs == nullptr) {
        std::cerr << "[PPS] OSAL GetTimeUs handler is unavailable" << std::endl;

        return 0;
    }

    uint64_t timeUs = 0;

    T_DjiReturnCode returnCode = osalHandler->GetTimeUs(&timeUs);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[PPS] OSAL GetTimeUs failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return 0;
    }

    return timeUs;
}

void PpsManager::ppsLoop() {
    gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if(chip == nullptr) {
        std::cerr << "[PPS] Failed to open " << GPIO_CHIP_NAME << std::endl;
        running.store(false);
        return;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, PPS_GPIO_OFFSET);
    if(line == nullptr) {
        std::cerr << "[PPS] Failed to get GPIO line " << PPS_GPIO_OFFSET << std::endl;

        gpiod_chip_close(chip);
        running.store(false);
        return;
    }

    int result = gpiod_line_request_rising_edge_events(line, "landscan-pps");

    if(result < 0) {
        std::cerr << "[PPS] Failed to request rising-edge events on GPIO" << PPS_GPIO_OFFSET << std::endl;

        gpiod_chip_close(chip);
        running.store(false);
        return;
    }

    std::cout << "[PPS] Wating for rising edge on GPIO" << PPS_GPIO_OFFSET << std::endl;

    while (running.load()) {
        timespec timeout{};
        timeout.tv_sec = 1;
        timeout.tv_nsec = 0;

        result = gpiod_line_event_wait(line, &timeout);

        if(result < 0) {
            std::cerr << "[PPS] GPIO event wait failed" << std::endl;
            break;
        }

        if(result == 0) {
            continue;
        }

        gpiod_line_event event{};
        result = gpiod_line_event_read(line, &event);

        if(result < 0) {
            std::cerr << "[PPS] GPIO event read failed" << std::endl;
            break;
        }

        if(event.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
            continue;
        }

        const uint64_t currentPpsTimeUs = getNowUs();
        if(currentPpsTimeUs == 0) {
            continue;
        }

        const uint64_t previousPpsTimeUs = lastPpsTimeUs.load();

        if(previousPpsTimeUs != 0 && 
            currentPpsTimeUs > previousPpsTimeUs &&
            currentPpsTimeUs - previousPpsTimeUs < PPS_MIN_INTERVAL_US) {
                
            continue;
        }

        lastPpsTimeUs.store(currentPpsTimeUs);
        ppsDetected.store(true);

        std::cout << "[PPS] Rising edge, localTimeUs=" << currentPpsTimeUs << std::endl;
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);

    running.store(false);
}