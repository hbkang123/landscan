#pragma once

#include "drone_telemetry.hpp"
#include "timesync_manager.hpp"
#include "navigation_data_recorder.hpp"
#include "lidar_point.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>
#include <filesystem>

class NavigationManager {
    public:
        bool initialize();
        bool start();
        void stop();

        std::filesystem::path getSessionDirectory() const;

        void setTimeSynchronizer(TimeSyncManager* timeSynchronizer);

        DroneTelemetry getNavigationSnapshot();
        void recordLidarPoint(const LidarPoint& point);

        bool getCurrentPosition(DronePosition& position);
        bool getCurrentAttitude(DroneAttitude& attitude);
        bool getCurrentVelocity(DroneVelocity& velocity);
        bool getCurrentAngularRate(DroneAngularRate& angularRate);
        bool getCurrentAcceleration(DroneAcceleration& acceleration);

    private:
        void navigationLoop();

        bool updateFlightStatus();
        bool updateDisplayMode();
        bool updatePosition();
        bool updateGpsDetails();
        bool updateQuaternion();
        bool updateVelocity();
        bool updateAngularRate();
        bool updateAcceleration();
        bool updateRtkPosition();
        bool updateRtkStatus();

        void recordAircraftTimeDiagnostic();

        bool isRtkAvailable(const DronePosition& rtkPosition, const DroneRtkStatus& rtkStatus) const;

        void printTelemetry(const DroneTelemetry& snapshot);

        uint64_t getNowMs();
        uint64_t getNowUs();
        uint64_t getAircraftTimeUs() const;

        TimeSyncManager* timeSyncManager = nullptr;

        bool isStarted = false;
        std::atomic<bool> running{false};
        std::thread workerThread;

        DroneTelemetry telemetry;
        std::mutex navigationMutex;

        NavigationDataRecorder recorder;

        std::uint8_t lastRtkPositionInfo = 0;
        bool hasLastRtkPositionInfo = false;
};