#include "navigation_manager.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstdint>
#include <string>

#include <dji_fc_subscription.h>
#include <dji_error.h>
#include <dji_platform.h>

// static uint64_t hardSyncToDroneTimeUs(const T_DjiFcSubscriptionHardSync& hardSync) {
//     uint64_t baseUs = static_cast<uint64_t>(hardSync.ts.time2p5ms) * 2500ULL;
//     uint64_t nsUs = static_cast<uint64_t>(hardSync.ts.time1ns) / 1000ULL;

//     return baseUs + nsUs;
// }

bool NavigationManager::initialize() {
    std::cout << "[Telemetry] Initialize" << std::endl;

    T_DjiReturnCode returnCode = DjiFcSubscription_Init();
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] DjiFcSubscription_Init failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    return true;
}

bool NavigationManager::start() {
    std::cout << "[Telemetry] Start" <<std::endl;

    T_DjiReturnCode returnCode;

    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
        nullptr
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe FLIGHT_STATUS failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
        DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
        nullptr
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe DISPLAY_MODE failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    // // returnCode = DjiFcSubscription_SubscribeTopic(
    // //     DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
    // //     DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
    // //     nullptr
    // // );

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Subscribe POSITION_FUSED failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        nullptr
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe GPS_POSITION failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
        nullptr
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe GPS_DETAILS failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe QUATERNION failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr  
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe VELOCITY failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr  
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe ANGULAR_RATE_FUSIONED failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr  
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe ACCELERATION_GROUND failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr  
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe RTK_POSITION failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiFcSubscription_SubscribeTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        nullptr  
    );

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Subscribe RTK_POSITION_INFO failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }




    //logwriter
    const std::uint64_t sessionStartOsalMs = getNowMs();

    if(sessionStartOsalMs == 0) {
        std::cerr << "[ERROR] Failed to get session start OSAL time" << std::endl;

        return false;
    }

    if(!recorder.initialize("landscan_logs", sessionStartOsalMs)) {
        std::cerr << "[ERROR] Navigation data recorder initialize failed" << std::endl;

        return false;
    }

    recorder.writeEvent(sessionStartOsalMs, "navigation_recording_started");





    running = true;
    workerThread = std::thread(&NavigationManager::navigationLoop, this);

    isStarted = true;
    std::cout << "[Telemetry] Started" << std::endl;

    return true;
}

void NavigationManager::stop() {
    if(!isStarted) return;

    std::cout << "[Telemetry] Stop" << std::endl;

    running = false;

    if (workerThread.joinable()) {
        workerThread.join();
    }


    //logwriter
    const std::uint64_t stopOsalTimeMs = getNowMs();
    recorder.writeEvent(stopOsalTimeMs, "navigation_recording_stopped");
    recorder.shutdown();



    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);
    //DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);

    DjiFcSubscription_DeInit();

    isStarted = false;

    std::cout << "[Telemetry] Stopped" << std::endl;
}

bool NavigationManager::updateFlightStatus() {
    T_DjiFcSubscriptionFlightStatus flightStatus{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        reinterpret_cast<uint8_t*>(&flightStatus),
        sizeof(flightStatus),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "flight_state.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "flight_status_raw",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<uint32_t>(returnCode),
        static_cast<uint32_t>(flightStatus)
    );

    if (!success) {
        std::cerr << "[ERROR] Get flight status failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get flight status failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.state.flightStatus = static_cast<uint8_t>(flightStatus);
        telemetry.state.flightStatusValid = true;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateDisplayMode() {
    T_DjiFcSubscriptionDisplaymode displayMode{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic (
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
        reinterpret_cast<uint8_t*>(&displayMode),
        sizeof(displayMode),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "display_mode.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "display_mode_raw",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<uint32_t>(returnCode),
        static_cast<uint32_t>(displayMode)
    );

    if (!success) {
        std::cerr << "[ERROR] Get display mode failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get display mode failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.state.displayMode = static_cast<uint8_t>(displayMode);
        telemetry.state.displayModeValid = true;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updatePosition() {
    T_DjiFcSubscriptionGpsPosition gpsPosition{};
    T_DjiDataTimestamp timestamp{};

    const T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        reinterpret_cast<std::uint8_t*>(&gpsPosition),
        sizeof(gpsPosition),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "position_gps.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "longitude_deg_e7,"
        "latitude_deg_e7,"
        "altitude_mm",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<std::uint32_t>(returnCode),
        gpsPosition.x,
        gpsPosition.y,
        gpsPosition.z
    );

    if (!success) {
        std::cerr << "[ERROR] Get GPS position failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    const double longitudeDeg = static_cast<double>(gpsPosition.x) * 0.0000001;
    const double latitudeDeg = static_cast<double>(gpsPosition.y) * 0.0000001;
    const double altitudeM = static_cast<double>(gpsPosition.z) / 1000.0;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);

        telemetry.position.longitude = longitudeDeg;
        telemetry.position.latitude = latitudeDeg;
        telemetry.position.altitude = altitudeM;
        telemetry.position.valid = true;
        telemetry.position.timestampMs = timestamp.millisecond;
        telemetry.position.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.position.aircraftTimeUs = 0;
        telemetry.position.source = PositionSource::GNSS;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateGpsDetails() {
    T_DjiFcSubscriptionGpsDetails gpsDetails{};
    T_DjiDataTimestamp timestamp{};

    const T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        reinterpret_cast<std::uint8_t*>(&gpsDetails),
        sizeof(gpsDetails),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

     recorder.writeRow(
        "gps_details.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "hdop_raw,"
        "pdop_raw,"
        "fix_state_raw,"
        "vacc_mm,"
        "hacc_mm,"
        "sacc_cmps,"
        "gps_satellites,"
        "glonass_or_beidou_satellites,"
        "total_satellites,"
        "gps_counter",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<std::uint32_t>(returnCode),
        gpsDetails.hdop,
        gpsDetails.pdop,
        gpsDetails.fixState,
        gpsDetails.vacc,
        gpsDetails.hacc,
        gpsDetails.sacc,
        gpsDetails.gpsSatelliteNumberUsed,
        gpsDetails.glonassSatelliteNumberUsed,
        gpsDetails.totalSatelliteNumberUsed,
        gpsDetails.gpsCounter
    );

    if (!success) {
        std::cerr << "[ERROR] Get GPS details failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    return true;
}

bool NavigationManager::updateQuaternion() {
    T_DjiFcSubscriptionQuaternion quaternion{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
        reinterpret_cast<uint8_t*>(&quaternion),
        sizeof(quaternion),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "attitude.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "q0_w,"
        "q1_x,"
        "q2_y,"
        "q3_z",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<std::uint32_t>(returnCode),
        quaternion.q0,
        quaternion.q1,
        quaternion.q2,
        quaternion.q3
    );

    if (!success) {
        std::cerr << "[ERROR] Get quaternion failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get quaternion failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }
    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.attitude.q0 = quaternion.q0;
        telemetry.attitude.q1 = quaternion.q1;
        telemetry.attitude.q2 = quaternion.q2;
        telemetry.attitude.q3 = quaternion.q3;
        telemetry.attitude.valid = true;
        telemetry.attitude.timestampMs = timestamp.millisecond;
        telemetry.attitude.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.attitude.aircraftTimeUs = 0;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateVelocity() {
    T_DjiFcSubscriptionVelocity velocity{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        reinterpret_cast<uint8_t*>(&velocity),
        sizeof(velocity),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "velocity.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "velocity_x_mps,"
        "velocity_y_mps,"
        "velocity_z_mps,"
        "health",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<uint32_t>(returnCode),
        velocity.data.x,
        velocity.data.y,
        velocity.data.z,
        static_cast<uint32_t>(velocity.health)
    );

    if(!success) {
        std::cerr << "[ERROR] Get velocity failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get velocity failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.velocity.x = velocity.data.x;
        telemetry.velocity.y = velocity.data.y;
        telemetry.velocity.z = velocity.data.z;
        telemetry.velocity.valid = true;
        telemetry.velocity.timestampMs = timestamp.millisecond;
        telemetry.velocity.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.velocity.aircraftTimeUs = 0;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateAngularRate() {
    T_DjiFcSubscriptionAngularRateFusioned angularRate{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
        reinterpret_cast<uint8_t*>(&angularRate),
        sizeof(angularRate),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "angular_rate.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "angular_rate_x_radps,"
        "angular_rate_y_radps,"
        "angular_rate_z_radps",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<uint32_t>(returnCode),
        angularRate.x,
        angularRate.y,
        angularRate.z
    );

    if(!success) {
        std::cerr << "[ERROR] Get angular rate fusioned failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        
        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get angular rate fusioned failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.angularRate.x = angularRate.x;
        telemetry.angularRate.y = angularRate.y;
        telemetry.angularRate.z = angularRate.z;
        telemetry.angularRate.valid = true;
        telemetry.angularRate.timestampMs = timestamp.millisecond;
        telemetry.angularRate.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.angularRate.aircraftTimeUs = 0;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateAcceleration() {
    T_DjiFcSubscriptionAccelerationGround acceleration{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
        reinterpret_cast<uint8_t*>(&acceleration),
        sizeof(acceleration),
        &timestamp
    );

    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    
    recorder.writeRow(
        "acceleration.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "acceleration_x_mps2,"
        "acceleration_y_mps2,"
        "acceleration_z_mps2",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<uint32_t>(returnCode),
        acceleration.x,
        acceleration.y,
        acceleration.z
    );

    if (!success) {
        std::cerr << "[ERROR] Get acceleration ground failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        
        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get acceleration ground failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.acceleration.x = acceleration.x;
        telemetry.acceleration.y = acceleration.y;
        telemetry.acceleration.z = acceleration.z;
        telemetry.acceleration.valid = true;
        telemetry.acceleration.timestampMs = timestamp.millisecond;
        telemetry.acceleration.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.acceleration.aircraftTimeUs = 0;
        telemetry.localUpdatedMs = getNowMs();
    }

    return true;
}

bool NavigationManager::updateRtkPosition() {
    T_DjiFcSubscriptionRtkPosition rtkPosition{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        reinterpret_cast<uint8_t*>(&rtkPosition),
        sizeof(rtkPosition),
        &timestamp
    );

    //logwriter
    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    recorder.writeRow(
        "position_rtk.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "longitude_deg,"
        "latitude_deg,"
        "hfsl_m",

        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<std::uint32_t>(returnCode),
        rtkPosition.longitude,
        rtkPosition.latitude,
        rtkPosition.hfsl
    );

    if(!success) {
        std::cerr << "[ERROR] Get RTK position failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get rtk position failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    std::lock_guard<std::mutex> lock(navigationMutex);
    telemetry.rtkPosition.latitude = rtkPosition.latitude;
    telemetry.rtkPosition.longitude = rtkPosition.longitude;
    telemetry.rtkPosition.altitude = rtkPosition.hfsl;
    telemetry.rtkPosition.valid = true;
    telemetry.rtkPosition.timestampMs = timestamp.millisecond;
    telemetry.rtkPosition.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
    telemetry.rtkPosition.aircraftTimeUs = 0;
    telemetry.localUpdatedMs = getNowMs();
    telemetry.rtkPosition.source = PositionSource::RTK;

    return true;
}

bool NavigationManager::updateRtkStatus() {
    T_DjiFcSubscriptionRtkPositionInfo rtkInfo{};
    T_DjiDataTimestamp timestamp{};

    T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        reinterpret_cast<uint8_t*>(&rtkInfo),
        sizeof(rtkInfo),
        &timestamp
    );

    //logwriter
    const bool success = returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    recorder.writeRow(
        "rtk_status.csv",

        "topic_time_ms,"
        "topic_time_us,"
        "success,"
        "return_code,"
        "position_info",
    
        timestamp.millisecond,
        timestamp.microsecond,
        success ? 1 : 0,
        static_cast<std::uint32_t>(returnCode),
        static_cast<std::uint32_t>(rtkInfo)
    );

    if (!success) {
        std::cerr << "[ERROR] Get rtk position info failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return false;
    }

    // if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     std::cerr << "[ERROR] Get rtk position info failed: 0x" << std::hex << returnCode << std::dec << std::endl;
    //     return false;
    // }

    if(!hasLastRtkPositionInfo || lastRtkPositionInfo != static_cast<uint8_t>(rtkInfo)) {
        std::string previousValue = "none";

        if(hasLastRtkPositionInfo) {
            previousValue = std::to_string(static_cast<uint32_t>(lastRtkPositionInfo));
        }

        const std::string currentValue = std::to_string(static_cast<uint32_t>(rtkInfo));
        const std::string eventMessage = "rtk_position_info_changed previous=" + previousValue + " current=" + currentValue;

        recorder.writeEvent(getNowMs(), eventMessage);
        std::cout << "[RTK STATE] " << eventMessage << std::endl;

        lastRtkPositionInfo = static_cast<uint8_t>(rtkInfo);
        hasLastRtkPositionInfo = true;
    }

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        telemetry.rtkStatus.positionInfo = rtkInfo;
        telemetry.rtkStatus.valid = true;
        telemetry.rtkStatus.timestampMs = timestamp.millisecond;
        telemetry.rtkStatus.timestampUs = static_cast<uint64_t>(timestamp.microsecond);
        telemetry.rtkStatus.aircraftTimeUs = 0;
        telemetry.localUpdatedMs = getNowMs();
    }
    return true;
}

void NavigationManager::printTelemetry(const DroneTelemetry& snapshot) {
    std::cout << "[Telemetry] ";

    std::cout << "flightOk=" << snapshot.state.flightStatusValid
              << ", displayOk=" << snapshot.state.displayModeValid
              << ", positionOk=" << snapshot.position.valid
              << ", quaternionOk=" << snapshot.attitude.valid
              << ", velocityOk=" << snapshot.velocity.valid
              << ", angularRateOk=" << snapshot.angularRate.valid
              << ", accelerationOk=" << snapshot.acceleration.valid
              << ", rtkPositionOk=" << snapshot.rtkPosition.valid
              << ", rtkStatusOk=" << snapshot.rtkStatus.valid;

    if(snapshot.state.flightStatusValid) {
        std::cout << ", flightStatus=" << static_cast<int>(snapshot.state.flightStatus);
    }

    if(snapshot.state.displayModeValid) {
        std::cout << ", displayMode=" << static_cast<int>(snapshot.state.displayMode);
    }

    if(snapshot.position.valid) {
        std::cout << ", lat=" << snapshot.position.latitude
                  << ", lon=" << snapshot.position.longitude
                  << ", alt=" << snapshot.position.altitude;
    }

    if(snapshot.attitude.valid) {
        std::cout << ", q0=" << snapshot.attitude.q0
                  << ", q1=" << snapshot.attitude.q1
                  << ", q2=" << snapshot.attitude.q2
                  << ", q3=" << snapshot.attitude.q3;
    }

    if(snapshot.velocity.valid) {
        std::cout << ", vx=" << snapshot.velocity.x
                << ", vy=" << snapshot.velocity.y
                << ", vz=" << snapshot.velocity.z;
    }

    if(snapshot.angularRate.valid) {
        std::cout << ", wx=" << snapshot.angularRate.x
                << ", wy=" << snapshot.angularRate.y
                << ", wz=" << snapshot.angularRate.z;
    }

    if(snapshot.acceleration.valid) {
        std::cout << ", ax=" << snapshot.acceleration.x
                << ", ay=" << snapshot.acceleration.y
                << ", az=" << snapshot.acceleration.z;
    }

    if(snapshot.rtkPosition.valid) {
        std::cout << ", rtkLat=" << snapshot.rtkPosition.latitude
                << ", rtkLon=" << snapshot.rtkPosition.longitude
                << ", rtkAlt=" << snapshot.rtkPosition.altitude;
    }

    if(snapshot.rtkStatus.valid) {
        std::cout << ", rtkInfo=" << static_cast<int>(snapshot.rtkStatus.positionInfo);
    }

    std::cout << std::endl;
}

void NavigationManager::navigationLoop() {
    std::cout << "[Telemetry] Loop started" << std::endl;

    uint32_t cycleCount = 0;
    auto nextCycleTime = std::chrono::steady_clock::now();

    while(running) {
        nextCycleTime += std::chrono::milliseconds(100);

        //10Hz
        updateQuaternion();
        updateVelocity();
        updateAngularRate();
        updateAcceleration();
        updateRtkPosition();

        //5Hz
        if(cycleCount % 2 == 0) {
            updatePosition();
            updateGpsDetails();
            updateRtkStatus();
        }

        //1Hz
        if(cycleCount == 0) {
            updateFlightStatus();
            updateDisplayMode();

            recordAircraftTimeDiagnostic();

            const DroneTelemetry snapshot = getNavigationSnapshot();
            printTelemetry(snapshot);
            recorder.flush();
        }

        cycleCount = (cycleCount + 1) % 10;
        std::this_thread::sleep_until(nextCycleTime);
    }

    recorder.flush();

    std::cout << "[Telemetry] Loop stopped" << std::endl;
}

void NavigationManager::recordLidarPoint(const LidarPoint& point) {
     recorder.writeRow(
        "lidar.csv",
        "receive_osal_time_us,"
        "estimated_measurement_osal_time_us,"
        "sequence_number,"
        "estimated_period_us,"
        "time_estimate_ready,"
        "time_discontinuity_detected,"
        "angle_deg,"
        "distance_m,"
        "signal_strength,"
        "measurement_valid",

        point.receiveOsalTimeUs,
        point.estimatedMeasurementOsalTimeUs,
        point.sequenceNumber,
        point.estimatedPeriodUs,
        point.timeEstimateReady ? 1 : 0,
        point.timeDiscontinuityDetected ? 1 : 0,
        point.angleDeg,
        point.distanceM,
        point.signalStrength,
        point.measurementValid ? 1 : 0
    );
}

DroneTelemetry NavigationManager::getNavigationSnapshot() {
    std::lock_guard<std::mutex> lock(navigationMutex);
    return telemetry;
}

std::filesystem::path NavigationManager::getSessionDirectory() const {
    return recorder.getSessionDirectory();
}

uint64_t NavigationManager::getNowMs() {
    // auto now = std::chrono::steady_clock::now();
    // auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     now.time_since_epoch()
    // ).count();

    // return static_cast<uint64_t>(ms);

    T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();

    if(osalHandler == nullptr || osalHandler->GetTimeMs == nullptr) {
        std::cerr << "[Telemetry] OSAL GetTimeMs is unavailable" << std::endl;
        
        return 0;
    }

    std::uint32_t timeMs = 0;

    const T_DjiReturnCode returnCode = osalHandler->GetTimeMs(&timeMs);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[Telemetry] OSAL GetTimeMs failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return 0;
    }

    return static_cast<std::uint64_t>(timeMs);
}

void NavigationManager::setTimeSynchronizer(TimeSyncManager* timeSyncManager) {
    this->timeSyncManager = timeSyncManager;
}

uint64_t NavigationManager::getNowUs() {
    // // auto now = std::chrono::steady_clock::now();
    // // auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    // // return static_cast<uint64_t>(us);

    // const std::uint64_t timeMs = getNowMs();

    // if(timeMs == 0) {
    //     return 0;
    // }

    // return timeMs * 1000ULL;

    T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();

    if(osalHandler == nullptr || osalHandler->GetTimeUs == nullptr) {
        std::cerr << "[Telemetry] OSAL GetTimeUs is unavailable" << std::endl;

        return 0;
    }

    uint64_t timeUs = 0;

    const T_DjiReturnCode returnCode = osalHandler->GetTimeUs(&timeUs);

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[Telemetry] OSAL GetTimeUs failed: 0x" << std::hex << returnCode << std::dec << std::endl;

        return 0;
    }

    return timeUs;
}

uint64_t NavigationManager::getAircraftTimeUs() const {
    if(timeSyncManager == nullptr) {
        return 0;
    }

    return timeSyncManager->getCurrentAircraftTimeUs();
}

void NavigationManager::recordAircraftTimeDiagnostic() {
    const uint64_t osalTimeMs = getNowMs();
    const uint64_t osalTimeMsAsUs = osalTimeMs * 1000ULL;
    const uint64_t osalTimeUs = getNowUs();

    const int64_t osalDifferenceUs = static_cast<int64_t>(osalTimeUs) - static_cast<int64_t>(osalTimeMsAsUs);

    const uint64_t aircraftTimeUs = getAircraftTimeUs();

    const bool synchronized = timeSyncManager != nullptr && timeSyncManager->isSyncronized();
    const bool aircraftTimeValid = synchronized && aircraftTimeUs != 0;

    recorder.writeRow(
        "aircraft_time_diagnostic.csv",

        "osal_time_ms,"
        "osal_time_ms_as_us,"
        "osal_time_us,"
        "osal_us_minus_ms_us,"
        "aircraft_time_us,"
        "synchronized,"
        "aircraft_time_valid",

        osalTimeMs,
        osalTimeMsAsUs,
        osalTimeUs,
        osalDifferenceUs,
        aircraftTimeUs,
        synchronized ? 1 : 0,
        aircraftTimeValid ? 1 : 0
    );
}

bool NavigationManager::getCurrentPosition(DronePosition& position) {
    // if(telemetry.rtkPosition.valid) {
    //     position.valid = true;
    //     position.latitude = telemetry.rtkPosition.latitude;
    //     position.longitude = telemetry.rtkPosition.longitude;
    //     position.altitude = telemetry.rtkPosition.altitude;
    //     position.timestampMs = telemetry.rtkPosition.timestampMs;
    //     position.aircraftTimeUs = telemetry.rtkPosition.aircraftTimeUs;
    //     position.source = PositionSource::RTK;
    //     return true;
    // }

    // if(telemetry.position.valid) {
    //     position = telemetry.position;
    //     position.source = PositionSource::GNSS;
    //     return true;
    // }

    //std::lock_guard<std::mutex> lock(navigationMutex);

    DronePosition gnssPosition;
    DronePosition rtkPosition;
    DroneRtkStatus rtkStatus;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        gnssPosition = telemetry.position;
        rtkPosition = telemetry.rtkPosition;
        rtkStatus = telemetry.rtkStatus;
    }

    if(isRtkAvailable(rtkPosition, rtkStatus)) {
        position = rtkPosition;
        return true;
    }

    if(gnssPosition.valid) {
        position = gnssPosition;
        return true;
    }

    position = DronePosition{};
    return false;
}

bool NavigationManager::getCurrentAttitude(DroneAttitude& attitude) {
    DroneAttitude current;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        current = telemetry.attitude;
    }

    if (!current.valid) {
        attitude = DroneAttitude{};
        return false;
    }

    attitude = current;
    return true;
} 

bool NavigationManager::getCurrentVelocity(DroneVelocity& velocity) {
    DroneVelocity current;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        current = telemetry.velocity;
    }

    if (!current.valid) {
        velocity = DroneVelocity{};
        return false;
    }

    velocity = current;
    return true;
}

bool NavigationManager::getCurrentAngularRate(DroneAngularRate& angularRate) {
    DroneAngularRate current;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        current = telemetry.angularRate;
    }

    if (!current.valid) {
        angularRate = DroneAngularRate{};
        return false;
    }

    angularRate = current;
    return true;
}

bool NavigationManager::getCurrentAcceleration(DroneAcceleration& acceleration) {
    DroneAcceleration current;

    {
        std::lock_guard<std::mutex> lock(navigationMutex);
        current = telemetry.acceleration;
    }

    if (!current.valid) {
        acceleration = DroneAcceleration{};
        return false;
    }

    acceleration = current;
    return true;
}

bool NavigationManager::isRtkAvailable(const DronePosition& rtkPosition, const DroneRtkStatus& rtkStatus) const {
    if(!rtkPosition.valid || !rtkStatus.valid) {
        return false;
    }

    return rtkStatus.positionInfo ==
               DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_L1_AMBIGUITY_INT
        || rtkStatus.positionInfo ==
               DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_WIDE_LANE_AMBIGUITY_INT
        || rtkStatus.positionInfo ==
               DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT;
}
