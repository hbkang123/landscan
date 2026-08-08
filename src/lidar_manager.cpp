#include "lidar_manager.hpp"
#include "app_config.hpp"

#include "common.h"
#include "lwNx.h"

#include <dji_error.h>
#include <dji_platform.h>

#include <iostream>
#include <stdint.h>
#include <string>

namespace {

constexpr uint8_t COMMAND_DISTANCE_OUTPUT = 27;
constexpr uint8_t COMMAND_STREAM = 30;
constexpr uint8_t COMMAND_DISTANCE_DATA = 44;
constexpr uint8_t COMMAND_UPDATE_RATE = 66;
constexpr uint8_t COMMAND_SCAN_LOW_ANGLE = 98;
constexpr uint8_t COMMAND_SCAN_HIGH_ANGLE = 99;

bool getOsalTimeUs(uint64_t& timeUs)
{
    T_DjiOsalHandler* osalHandler =
        DjiPlatform_GetOsalHandler();

    if(
        osalHandler == nullptr
        || osalHandler->GetTimeUs == nullptr
    ) {
        std::cerr
            << "[Lidar] OSAL GetTimeUs is unavailable"
            << std::endl;

        return false;
    }

    const T_DjiReturnCode returnCode =
        osalHandler->GetTimeUs(&timeUs);

    if(
        returnCode
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
    ) {
        std::cerr
            << "[Lidar] OSAL GetTimeUs failed: 0x"
            << std::hex
            << returnCode
            << std::dec
            << std::endl;

        return false;
    }

    return true;
}

}

LidarManager::LidarManager()
{
}

LidarManager::~LidarManager()
{
    stop();
}

bool LidarManager::configure(
    const RuntimeConfig& runtimeConfig
) {
    std::string errorMessage;

    if(!runtimeConfig.validate(errorMessage)) {
        std::cerr
            << "[Lidar] Runtime configuration is invalid: "
            << errorMessage
            << std::endl;

        return false;
    }

    uint8_t updateRateCommand = 0;

    if(!runtimeConfig.getLidarUpdateRateCommand(
        updateRateCommand
    )) {
        std::cerr
            << "[Lidar] Failed to convert update rate "
            << runtimeConfig.lidarUpdateRateHz
            << " Hz to SF45 command"
            << std::endl;

        return false;
    }

    if(!timeEstimator_.configureRateHz(
        runtimeConfig.lidarUpdateRateHz
    )) {
        std::cerr
            << "[Lidar] Failed to configure time estimator rate: "
            << runtimeConfig.lidarUpdateRateHz
            << " Hz"
            << std::endl;

        return false;
    }

    runtimeConfig_ = runtimeConfig;
    updateRateCommand_ = updateRateCommand;

    std::cout
        << "[Lidar] Runtime configuration accepted"
        << ", port=" << runtimeConfig_.lidarPort
        << ", baudrate=" << runtimeConfig_.lidarBaudrate
        << ", rateHz=" << runtimeConfig_.lidarUpdateRateHz
        << ", rateCommand="
        << static_cast<unsigned int>(
            updateRateCommand_
        )
        << ", lowAngleDeg="
        << runtimeConfig_.lidarScanLowAngleDeg
        << ", highAngleDeg="
        << runtimeConfig_.lidarScanHighAngleDeg
        << std::endl;

    return true;
}

uint16_t LidarManager::readUInt16(
    uint8_t* buffer,
    uint32_t offset
) {
    const uint16_t result =
        static_cast<uint16_t>(
            static_cast<uint16_t>(
                buffer[offset]
            )
            |
            static_cast<uint16_t>(
                static_cast<uint16_t>(
                    buffer[offset + 1]
                )
                << 8
            )
        );

    return result;
}

bool LidarManager::init(
    const std::string& portName,
    int baudrate
) {
    std::cout
        << "[Lidar] init port="
        << portName
        << ", baudrate="
        << baudrate
        << std::endl;

    platformInit();

    serial_ = platformCreateSerialPort();

    if(serial_ == nullptr) {
        std::cerr
            << "[Lidar] serial create failed"
            << std::endl;

        return false;
    }

    if(!serial_->connect(
        portName.c_str(),
        baudrate
    )) {
        std::cerr
            << "[Lidar] serial connect failed"
            << std::endl;

        return false;
    }

    char modelName[16] = {};

    if(!lwnxCmdReadString(
        serial_,
        0,
        modelName
    )) {
        std::cerr
            << "[Lidar] read model name failed"
            << std::endl;

        return false;
    }

    uint32_t hardwareVersion = 0;

    if(!lwnxCmdReadUInt32(
        serial_,
        1,
        &hardwareVersion
    )) {
        std::cerr
            << "[Lidar] read hardware version failed"
            << std::endl;

        return false;
    }

    uint32_t firmwareVersion = 0;

    if(!lwnxCmdReadUInt32(
        serial_,
        2,
        &firmwareVersion
    )) {
        std::cerr
            << "[Lidar] read firmware version failed"
            << std::endl;

        return false;
    }

    char serialNumber[16] = {};

    if(!lwnxCmdReadString(
        serial_,
        3,
        serialNumber
    )) {
        std::cerr
            << "[Lidar] read serial number failed"
            << std::endl;

        return false;
    }

    std::cout
        << "[Lidar] Model: "
        << modelName
        << std::endl;

    std::cout
        << "[Lidar] Hardware: "
        << hardwareVersion
        << std::endl;

    std::cout
        << "[Lidar] Firmware: "
        << firmwareVersion
        << std::endl;

    std::cout
        << "[Lidar] Serial: "
        << serialNumber
        << std::endl;

    return true;
}

bool LidarManager::start()
{
    if(serial_ == nullptr) {
        std::cerr
            << "[Lidar] serial is not initialized"
            << std::endl;

        return false;
    }

    if(!lwnxCmdWriteInt32(
        serial_,
        COMMAND_STREAM,
        static_cast<int32_t>(
            AppConfig::LIDAR_STREAM_STOP_VALUE
        )
    )) {
        std::cerr
            << "[Lidar] stop previous stream failed"
            << std::endl;

        return false;
    }

    uint8_t discardBuffer[256] = {};

    while(
        serial_->readData(
            discardBuffer,
            static_cast<int32_t>(
                sizeof(discardBuffer)
            )
        ) > 0
    ) {
    }

    nextSequenceNumber_ = 0;

    timeEstimator_.reset();

    if(!lwnxCmdWriteUInt8(
        serial_,
        COMMAND_UPDATE_RATE,
        updateRateCommand_
    )) {
        std::cerr
            << "[Lidar] set update rate failed"
            << std::endl;

        return false;
    }

    if(!lwnxCmdWriteFloat(
        serial_,
        COMMAND_SCAN_LOW_ANGLE,
        runtimeConfig_.lidarScanLowAngleDeg
    )) {
        std::cerr
            << "[Lidar] set low angle failed"
            << std::endl;

        return false;
    }

    if(!lwnxCmdWriteFloat(
        serial_,
        COMMAND_SCAN_HIGH_ANGLE,
        runtimeConfig_.lidarScanHighAngleDeg
    )) {
        std::cerr
            << "[Lidar] set high angle failed"
            << std::endl;

        return false;
    }

    if(!lwnxCmdWriteUInt32(
        serial_,
        COMMAND_DISTANCE_OUTPUT,
        AppConfig::LIDAR_OUTPUT_FORMAT
    )) {
        std::cerr
            << "[Lidar] set output format failed"
            << std::endl;

        return false;
    }

    if(!lwnxCmdWriteUInt32(
        serial_,
        COMMAND_STREAM,
        AppConfig::LIDAR_STREAM_START_VALUE
    )) {
        std::cerr
            << "[Lidar] stream start failed"
            << std::endl;

        return false;
    }

    running_ = true;

    std::cout
        << "[Lidar] stream started"
        << ", rateHz="
        << runtimeConfig_.lidarUpdateRateHz
        << ", lowAngleDeg="
        << runtimeConfig_.lidarScanLowAngleDeg
        << ", highAngleDeg="
        << runtimeConfig_.lidarScanHighAngleDeg
        << std::endl;

    return true;
}

void LidarManager::stop()
{
    if(!running_) {
        return;
    }

    running_ = false;

    if(serial_ != nullptr) {
        if(!lwnxCmdWriteUInt32(
            serial_,
            COMMAND_STREAM,
            AppConfig::LIDAR_STREAM_STOP_VALUE
        )) {
            std::cerr
                << "[Lidar] stream stop command failed"
                << std::endl;
        }
    }

    std::cout
        << "[Lidar] stop"
        << std::endl;
}

bool LidarManager::readPoint(LidarPoint& point)
{
    if(!running_ || serial_ == nullptr) {
        return false;
    }

    lwResponsePacket response{};

    if(!lwnxRecvPacket(
        serial_,
        COMMAND_DISTANCE_DATA,
        &response,
        AppConfig::LIDAR_RECEIVE_TIMEOUT_MS
    )) {
        return false;
    }

    uint64_t receiveOsalTimeUs = 0;

    if(!getOsalTimeUs(receiveOsalTimeUs)) {
        return false;
    }

    const uint16_t distanceCm =
        readUInt16(response.data, 4);

    const uint16_t strength =
        readUInt16(response.data, 6);

    const float yawAngle =
        static_cast<int16_t>(
            readUInt16(response.data, 10)
        )
        / 100.0f;

    point.receiveOsalTimeUs =
        receiveOsalTimeUs;

    point.sequenceNumber =
        nextSequenceNumber_++;

    const LidarTimeEstimator::Result timeResult =
        timeEstimator_.update(
            point.sequenceNumber,
            point.receiveOsalTimeUs
        );

    point.estimatedMeasurementOsalTimeUs =
        timeResult.estimatedTimeUs;

    point.estimatedPeriodUs =
        timeResult.estimatedPeriodUs;

    point.timeEstimateReady =
        timeResult.ready;

    point.timeDiscontinuityDetected =
        timeResult.discontinuityDetected;

    point.angleDeg = yawAngle;

    point.distanceM =
        static_cast<float>(distanceCm)
        / 100.0f;

    point.signalStrength =
        static_cast<float>(strength);

    point.measurementValid =
        point.distanceM
            >= AppConfig::LIDAR_MIN_VALID_DISTANCE_M
        &&
        point.distanceM
            <= AppConfig::LIDAR_MAX_VALID_DISTANCE_M;

    return true;
}