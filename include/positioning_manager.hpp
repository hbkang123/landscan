#pragma once

#include <cstdint>

#include <dji_typedef.h>

class PositioningManager {
    public:
        bool initialize();
        void shutdown();

    private:
        static T_DjiReturnCode receiveRtcmDataCallback(
            uint8_t index,
            const uint8_t *data,
            uint16_t dataLen);
};