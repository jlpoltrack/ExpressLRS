#pragma once

#include "targets.h"
#include "rxtx_intf.h"
#include "config.h"
#include "FHSS.h"
#include "helpers.h"
#include "hwTimer.h"
#include "logging.h"
#include "LBT.h"
#include "LQCALC.h"
#include "OTA.h"
#include "POWERMGNT.h"
#include "deferred.h"

// Set by the mains to hold off the WiFi auto-start; lives here so it exists
// even on targets built without the WiFi library
extern bool webserverPreventAutoStart;

void setupTargetCommon();
void rebootDevice();
void checkRebootTime(unsigned long now);