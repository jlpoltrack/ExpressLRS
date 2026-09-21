#pragma once

#include "targets.h"
#include "device.h"

extern device_t Serial0_device;
#if defined(HAS_SERIAL1)
extern device_t Serial1_device;
#endif
extern void sendImmediateRC();
extern void handleSerialIO();
extern void crsfRCFrameAvailable();
extern void crsfRCFrameMissed();
