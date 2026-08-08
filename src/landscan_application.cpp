#include "landscan_application.hpp"

#include <iostream>
#include <filesystem>
#include <string>

bool LandScanApplication::initialize(const RuntimeConfig& runtimeConfig) {
    std::cout << "[INFO] LandScanApplication initialize" << std::endl;

    if(!lidarManager.configure(runtimeConfig)) {
        std::cerr
            << "[ERROR] LidarManager configuration failed"
            << std::endl;

        return false;
    }

    if(!payloadManager.initialize()) {
        std::cerr << "[ERROR] PayloadManager initialize failed" << std::endl;

        return false;
    }

    if(!positioningManager.initialize()) {
        std::cerr << "[ERROR] PositioningManager initialize failed" << std::endl;

        payloadManager.shutdown();
        return false;
    }

    timeSyncManager.setPpsManager(&ppsManager);

    if(!timeSyncManager.initialize()) {
        std::cerr << "[ERROR] TimeSyncManager initialize failed" << std::endl;

        positioningManager.shutdown();
        payloadManager.shutdown();
        return false;
    }

    if(!ppsManager.initialize()) {
        std::cerr << "[ERROR] PpsManager initialize failed" << std::endl;

        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();
        return false;
    }

    if(!navigationManager.initialize()) {
        std::cerr << "[ERROR] NavigationManager initialize failed" << std::endl;

        ppsManager.shutdown();
        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();
        return false;
    }

    navigationManager.setTimeSynchronizer(&timeSyncManager);

    if(!navigationManager.start()) {
        std::cerr << "[ERROR] NavigationManager start failed" << std::endl;

        ppsManager.shutdown();
        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();
        return false;
    }

    const std::filesystem::path sessionDirectory = navigationManager.getSessionDirectory();
    const std::filesystem::path sessionConfigurationPath = sessionDirectory / "session_config.conf";

    std::string configurationSaveError;

    if(!runtimeConfig.saveToFile(sessionConfigurationPath.string(), configurationSaveError)) {
        std::cerr << "[ERROR] Session configuration save failed: " << configurationSaveError << std::endl;

        navigationManager.stop();
        ppsManager.shutdown();
        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();

        return false;
    }

    std::cout << "[Config] Session configuration saved: " << sessionConfigurationPath << std::endl;

    if(!lidarManager.init(runtimeConfig.lidarPort, runtimeConfig.lidarBaudrate)) {
        std::cerr << "[ERROR] LidarManager initialize failed" << std::endl;

        navigationManager.stop();
        ppsManager.shutdown();
        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();

        return false;
    }

    if(!lidarManager.start()) {
        std::cerr << "[ERROR] LidarManager start failed" << std::endl;

        lidarManager.stop();
        navigationManager.stop();
        ppsManager.shutdown();
        timeSyncManager.shutdown();
        positioningManager.shutdown();
        payloadManager.shutdown();

        return false;
    }

    std::cout << "[INFO] LidarManager started" << std::endl;

    lidarRunning.store(true);

    lidarThread = std::thread([this]() {
        while(lidarRunning.load()) {
            LidarPoint point{};

            if(!lidarManager.readPoint(point)) {
                continue;
            }

            navigationManager.recordLidarPoint(point);

            if(point.sequenceNumber % 500ULL == 0ULL) {
                std::cout
                << "[Lidar] sequence=" << point.sequenceNumber
                << ", osalTimeUs=" << point.receiveOsalTimeUs
                << ", angleDeg=" << point.angleDeg
                << ", distanceM=" << point.distanceM
                << ", strength=" << point.signalStrength
                << std::endl;
            }
        }

        std::cout << "[Lidar] Receive loop stopped" << std::endl;
    });

    std::cout << "[INFO] LandscanApplication initialized" << std::endl;

    return true;
}

void LandScanApplication::shutdown() {
    std::cout << "[INFO] LandScanApplication shutdown" << std::endl;

    const std::filesystem::path sessionDirectory = navigationManager.getSessionDirectory();

    lidarRunning.store(false);

    if(lidarThread.joinable()) {
        lidarThread.join();
    }

    lidarManager.stop();
    navigationManager.stop();
    ppsManager.shutdown();
    timeSyncManager.shutdown();
    positioningManager.shutdown();
    payloadManager.shutdown();

    const bool attitudeDataValid = postprocessManager.validateAttitudeData(sessionDirectory);

    if(!attitudeDataValid) {
        std::cerr << "[Postprocess] Attitude validation failed" << std::endl;
    }

    std::cout << "[INFO] LandScanApplication shutdown complete22" << std::endl;
}

