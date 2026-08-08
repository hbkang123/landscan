#pragma once

#include "dji_typedef.h"
#include "dji_core.h"

bool ApplicationInit();
bool ApplicationStart();
void ApplicationStop();

bool RegisterOsal();

bool InitializeCore();
bool VerifyAircraft();
bool ConfigureAlias();
bool StartServices();

bool RegisterHal();
bool RegisterSocket();
bool RegisterFileSystem();
bool RegisterLogger();

T_DjiReturnCode FillUserInfo(T_DjiUserInfo *userInfo);
