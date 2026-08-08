#include <iostream>
#include <cstring>
#include <unistd.h>
#include <algorithm>

#include "application.hpp"

#include <dji_platform.h>
#include <dji_typedef.h>
#include <dji_core.h>
#include <dji_aircraft_info.h>
#include <dji_logger.h>

#include "dji_sdk_app_info.h"
#include "dji_sdk_config.h"
#include "dji_logger.h"
#include "dji_platform.h"


#include "../common/osal/osal.h"
#include "../hal/hal_uart.h"
#include "../hal/hal_i2c.h"
#include "../hal/hal_usb_bulk.h"
#include "../hal/hal_network.h"

//init function start
T_DjiReturnCode FillUserInfo(T_DjiUserInfo *userInfo) {
    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->appLicense, 0, sizeof(userInfo->appLicense));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));
    memset(userInfo->baudRate, 0, sizeof(userInfo->baudRate));

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, std::min(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, std::min(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    memcpy(userInfo->appLicense, USER_APP_LICENSE, std::min(sizeof(userInfo->appLicense), strlen(USER_APP_LICENSE)));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, std::min(sizeof(userInfo->baudRate), strlen(USER_BAUD_RATE)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiUser_PrintConsole(const uint8_t *data, uint16_t dataLen) {
    printf("%s", data);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
//init function end

//main functions start
bool ApplicationInit() {
   std::cout << "[PSDK] ApplicationInit()" << std::endl;

   if(!RegisterOsal()) return false;
   if(!RegisterHal()) return false;
   if(!RegisterSocket()) return false;
   if(!RegisterFileSystem()) return false;
   if(!RegisterLogger()) return false;

   return true;
}

bool ApplicationStart() {
    std::cout << "[PSDK] Application Start()" << std::endl;

    if(!InitializeCore()) return false;
    // if(!VerifyAircraft()) return false;
    // if(!ConfigureAlias()) return false;
    // if(!StartServices()) return false;

    return true;
}
//main functions end

//ApplicationInit Sub functions start
bool RegisterOsal() {
    std::cout << "[PSDK] Register OSAL" << std::endl;

    T_DjiOsalHandler osalHandler = {0};

    osalHandler.TaskCreate = Osal_TaskCreate;
    osalHandler.TaskDestroy = Osal_TaskDestroy;
    osalHandler.TaskSleepMs = Osal_TaskSleepMs;
    osalHandler.MutexCreate = Osal_MutexCreate;
    osalHandler.MutexDestroy = Osal_MutexDestroy;
    osalHandler.MutexLock = Osal_MutexLock;
    osalHandler.MutexUnlock = Osal_MutexUnlock;
    osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
    osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
    osalHandler.SemaphoreWait = Osal_SemaphoreWait;
    osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
    osalHandler.SemaphorePost = Osal_SemaphorePost;
    osalHandler.Malloc = Osal_Malloc;
    osalHandler.Free = Osal_Free;
    osalHandler.GetTimeMs = Osal_GetTimeMs;
    osalHandler.GetTimeUs = Osal_GetTimeUs;
    osalHandler.GetRandomNum = Osal_GetRandomNum;

    T_DjiReturnCode returnCode = DjiPlatform_RegOsalHandler(&osalHandler);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Register OSAL failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    return true;
}

bool RegisterHal() {
    std::cout << "[PSDK] Register HAL" << std::endl;

    T_DjiReturnCode returnCode;

    T_DjiHalUartHandler uartHandler = {0};
    T_DjiHalI2cHandler i2CHandler = {0};
    T_DjiHalUsbBulkHandler usbBulkHandler = {0};
    T_DjiHalNetworkHandler networkHandler = {0};

    uartHandler.UartInit = HalUart_Init;
    uartHandler.UartDeInit = HalUart_DeInit;
    uartHandler.UartWriteData = HalUart_WriteData;
    uartHandler.UartReadData = HalUart_ReadData;
    uartHandler.UartGetStatus = HalUart_GetStatus;
    uartHandler.UartGetDeviceInfo = HalUart_GetDeviceInfo;

    i2CHandler.I2cInit = HalI2c_Init;
    i2CHandler.I2cDeInit = HalI2c_DeInit;
    i2CHandler.I2cWriteData = HalI2c_WriteData;
    i2CHandler.I2cReadData = HalI2c_ReadData;

    usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
    usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
    usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
    usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
    usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

    networkHandler.NetworkInit = HalNetWork_Init;
    networkHandler.NetworkDeInit = HalNetWork_DeInit;
    networkHandler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

    returnCode = DjiPlatform_RegHalI2cHandler(&i2CHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalI2c failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

#if (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalUart failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalUsbBulk failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalUart failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalNetwork failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_USB_BULK_DEVICE)
    returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalUsbBulk failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_NETWORK_DEVICE)
    returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalNetwork failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] RegHalUart failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }
#endif

    return true;
}

bool RegisterSocket() {
    std::cout << "[PSDK] Register Socket" << std::endl;
    return true;
}

bool RegisterFileSystem() {
    std::cout << "[PSDK] Register FileSystem" << std::endl;
    return true;
}

bool RegisterLogger() {
    std::cout << "[PSDK] Register Logger" << std::endl;

    T_DjiReturnCode returnCode;

    T_DjiLoggerConsole printConsole = {
        .func = DjiUser_PrintConsole,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        .isSupportColor = true,
    };

    returnCode = DjiLogger_AddConsole(&printConsole);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] DjiLogger_AddConsole failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    return true;
}
//ApplicationInit Sub functions end

//ApplicationStart Sub functions start
bool InitializeCore() {
    std::cout << "[PSDK] Initialize Core" << std::endl;

    T_DjiUserInfo userInfo;
    T_DjiReturnCode returnCode;

    T_DjiFirmwareVersion firmwareVersion = {
        .majorVersion = 1,
        .minorVersion = 0,
        .modifyVersion = 0,
        .debugVersion = 0,
    };

    returnCode = FillUserInfo(&userInfo);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Fill user info failed" << std::endl;
        return false;
    }

    returnCode = DjiCore_SetFirmwareVersion(firmwareVersion);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Set firmware version failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiCore_SetSerialNumber("PSDK12345678XX");
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] Set serial number failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    returnCode = DjiCore_Init(&userInfo);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] DjiCore_Init failed: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;
    }

    return true;
}

bool VerifyAircraft() {
    std::cout << "[PSDK] Verify Aircraft" << std::endl;
    return true;
}

bool ConfigureAlias() {
    std::cout << "[PSDK] Configure Alias" << std::endl;
    return true;
}

bool StartServices() {
    std::cout << "[PSDK] Start Services" << std::endl;

    T_DjiReturnCode returnCode;

    returnCode = DjiCore_ApplicationStart();
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cerr << "[ERROR] DjiCore_ApplicationStart fail: 0x" << std::hex << returnCode << std::dec << std::endl;
        return false;        
    }

    std::cout << "[PSDK] Application Started" << std::endl;

    return true;
}
//ApplicationStart Sub functions end

void ApplicationStop() {
    std::cout << "[PSDK] ApplicationStop()" << std::endl;
}