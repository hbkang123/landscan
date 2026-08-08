#include "landscan_application.hpp"
#include "postprocess_manager.hpp"
#include "runtime_config.hpp"

#include <filesystem>
#include <string>
#include <iostream>
#include <atomic>
#include <csignal>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};

void signalHandler(int) {
    running.store(false);
}

int runPostProcessMode(const std::filesystem::path& sessionDirectory) {
    std::cout << "[Postprocess] Session directory: " << sessionDirectory << std::endl;

    if(!std::filesystem::exists(sessionDirectory)) {
        std::cerr << "[Postprocess] Session directory does not exist" << std::endl;

        return 1;
    }

    if(!std::filesystem::is_directory(sessionDirectory)) {
        std::cerr << "[Postprocess] Path is not a directory" << std::endl;

        return 1;
    }

        const std::filesystem::path sessionConfigurationPath =
        sessionDirectory / "session_config.conf";

    RuntimeConfig runtimeConfig;
    std::string configurationError;

    if(std::filesystem::exists(sessionConfigurationPath)) {
        if(!runtimeConfig.loadFromFile(
            sessionConfigurationPath.string(),
            configurationError
        )) {
            std::cerr
                << "[Postprocess] Session configuration load failed: "
                << configurationError
                << std::endl;

            return 1;
        }

        std::cout
            << "[Postprocess] Session configuration loaded: "
            << sessionConfigurationPath
            << std::endl;
    }
    else {
        std::cout
            << "[Postprocess] Session configuration not found; "
            << "using built-in default values"
            << std::endl;
    }

    uint8_t updateRateCommand = 0;

    if(!runtimeConfig.getLidarUpdateRateCommand(
        updateRateCommand
    )) {
        std::cerr
            << "[Postprocess] Invalid LiDAR update rate configuration"
            << std::endl;

        return 1;
    }

    std::cout
        << "[Postprocess] Acquisition configuration"
        << ", lidarRateHz="
        << runtimeConfig.lidarUpdateRateHz
        << ", lidarRateCommand="
        << static_cast<unsigned int>(updateRateCommand)
        << ", scanAngleDeg=("
        << runtimeConfig.lidarScanLowAngleDeg
        << ","
        << runtimeConfig.lidarScanHighAngleDeg
        << ")"
        << ", lidarOffsetFrdM=("
        << runtimeConfig.lidarOffsetForwardM
        << ","
        << runtimeConfig.lidarOffsetRightM
        << ","
        << runtimeConfig.lidarOffsetDownM
        << ")"
        << ", sourceEpsg="
        << runtimeConfig.sourceGeographicEpsgCode
        << ", outputEpsg="
        << runtimeConfig.outputProjectedEpsgCode
        << std::endl;

    PostprocessManager postprocessManager;

    if(!postprocessManager.configure(runtimeConfig)) {
        std::cerr
            << "[Postprocess] Configuration failed"
            << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLidarData(sessionDirectory)) {
        std::cerr << "[Postprocess] LiDAR validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateAttitudeData(sessionDirectory)) {
        std::cerr << "[Postprocess] Attitude validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateGpsPositionData(sessionDirectory)) {
        std::cerr << "[Postprocess] GPS position validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateRtkPositionData(sessionDirectory)) {
        std::cerr << "[Postprocess] RTK position validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateRtkStatusData(sessionDirectory)) {
        std::cerr << "[Postprocess] RTK status validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateRtkStatusMatching(sessionDirectory)) {
        std::cerr << "[Postprocess] RTK status matching validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validatePositionInterpolation(sessionDirectory)) {
        std::cerr << "[Postprocess] Position interpolation validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateAttitudeInterpolation(sessionDirectory)) {
        std::cerr << "[Postprocess] Attitude interpolation validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validatePoseInterpolation(sessionDirectory)) {
        std::cerr << "[Postprocess] Pose interpolation validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLidarPoseMatching(sessionDirectory)) {
        std::cerr << "[Postprocess] LiDAR pose matching validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLidarRtkStatusMatching(sessionDirectory)) {
        std::cerr << "[Postprocess] LiDAR RTK status matching validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLidarBodyCoordinateTransform()) {
        std::cerr << "[Postprocess] LiDAR body coordinate " << "transform validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateBodyToNedRotation()) {
        std::cerr << "[Postprocess] Body to NED rotation " << "validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateGeodeticToLocalNed(sessionDirectory)) {
        std::cerr << "[Postprocess] Geodetic to local NED " << "validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLocalNedGeodeticRoundTrip(sessionDirectory)) {
        std::cerr << "[Postprocess] Local NED geodetic round-trip validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateCrsTransformation(sessionDirectory)) {
        std::cerr << "[Postprocess] CRS transformation validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.validateLocalNedLidarPointGeneration(sessionDirectory)) {
        std::cerr << "[Postprocess] Local NED LiDAR point generation validation failed" << std::endl;

        return 1;
    }

    if(!postprocessManager.generateLocalNedPointCloudCsv(sessionDirectory)) {
        std::cerr << "[Postprocess] Local NED point cloud CSV generation failed" << std::endl;

        return 1;
    }

    std::cout << "[Postprocess] Validation completed" << std::endl;

    return 0;
}
}

int main(int argc, char* argv[])
{
    if(
        argc == 3
        && std::string(argv[1]) == "--postprocess"
    ) {
        return runPostProcessMode(
            std::filesystem::path(argv[2])
        );
    }

    std::filesystem::path configurationPath;

    if(argc == 1) {
        const std::filesystem::path executablePath =
            std::filesystem::absolute(
                std::filesystem::path(argv[0])
            ).lexically_normal();

        configurationPath =
            (
                executablePath.parent_path()
                .parent_path()
                / "config"
                / "landscan.conf"
            ).lexically_normal();
    }
    else if(
        argc == 3
        && std::string(argv[1]) == "--config"
    ) {
        configurationPath =
            std::filesystem::path(argv[2]);
    }
    else {
        std::cerr
            << "Usage:"
            << std::endl
            << " ./landscan"
            << std::endl
            << " ./landscan --config <configuration_file>"
            << std::endl
            << " ./landscan --postprocess <session_directory>"
            << std::endl;

        return 1;
    }

    RuntimeConfig runtimeConfig;
    std::string configurationError;

    if(!runtimeConfig.loadFromFile(
        configurationPath.string(),
        configurationError
    )) {
        std::cerr
            << "[Config] Load failed: "
            << configurationError
            << std::endl;

        return 1;
    }

    std::cout
        << "[Config] Loaded: "
        << configurationPath
        << std::endl;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout
        << "===================================="
        << std::endl;

    std::cout
        << "          LandScan Start"
        << std::endl;

    std::cout
        << "===================================="
        << std::endl;

    LandScanApplication app;

    if(!app.initialize(runtimeConfig)) {
        std::cerr
            << "LandScan init failed"
            << std::endl;

        return 1;
    }

    std::cout
        << "[INFO] LandScan running"
        << std::endl;

    while(running.load()) {
        sleep(1);
    }

    std::cout
        << "[INFO] LandScan stopping"
        << std::endl;

    app.shutdown();

    return 0;
}