#pragma once

#include <Arduino.h>
#include <hardware/regs/addressmap.h>

extern char __flash_binary_end;

// The subset of the ESP core's EspClass the shared code calls. Addresses are flash
// offsets as on ESP, and flash is memory mapped (XIP) so reads are a plain copy.
class EspClass
{
public:
    void restart() { rp2040.reboot(); }
    uint32_t getSketchSize() { return (uintptr_t)&__flash_binary_end - XIP_BASE; }
    bool flashRead(uint32_t address, uint32_t *data, size_t size)
    {
        memcpy(data, (const void *)(XIP_BASE + address), size);
        return true;
    }
};

inline EspClass ESP;
