#include "positioning_manager.hpp"

#include <iostream>

#include <dji_positioning.h>
#include <dji_error.h>

bool PositioningManager::initialize() {
    std::cout << "[Positioning] Initialize" << std::endl;

    T_DjiReturnCode returnCode = DjiPositioning_Init();

    std::cout << "[Positioning] DjiPositioning_Init result: 0x"
              << std::hex << returnCode << std::dec << std::endl;

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return false;
    }

    returnCode = DjiPositioning_RegReceiveRtcmDataCallback(
        DJI_POSITIONING_RTCM_DATA_TYPE_RTK_ON_AIRCRAFT,
        PositioningManager::receiveRtcmDataCallback
    );

     std::cout << "[Positioning] RegReceiveRtcmDataCallback result: 0x"
              << std::hex << returnCode << std::dec << std::endl;

    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return false;
    }

    return true;
}

void PositioningManager::shutdown() {
    std::cout << "[Positioning] Shutdown" << std::endl;
}

T_DjiReturnCode  PositioningManager::receiveRtcmDataCallback( 
    uint8_t index,
    const uint8_t* data,
    uint16_t dataLen) {
    (void)index;
    (void)data;
    (void)dataLen;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}