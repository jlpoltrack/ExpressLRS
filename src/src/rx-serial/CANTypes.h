#pragma once

#include <stdint.h>

// Extended (29-bit) data frame, the only kind DroneCAN uses
struct CANFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

enum CANState : uint8_t {
    CAN_STOPPED,
    CAN_RUNNING,
    CAN_BUS_OFF,
    CAN_RECOVERING,
    CAN_RESETTING,      // ESP32 errata reset pending, done by poll()
};

struct CANStats {
    uint32_t rxOverrun;     // receive buffer overflows (TWAI hardware FIFO, can2040 internal), each drops frames
    uint32_t rxQueueFull;   // frames lost to the software RX ring being full
    uint32_t txFailed;      // frames the controller gave up on, or in flight when flushed
    uint32_t busOff;        // TWAI only
    uint32_t errataResets;  // ESP32 only
    uint32_t busErrors;     // can2040 only: parse errors, including unacknowledged transmits
};

// Two acceptance filters on the 29-bit extended ID, set mask bits must match
struct CANFilter {
    uint32_t code1, mask1;
    uint32_t code2, mask2;

    __attribute__((always_inline)) bool matches(uint32_t id) const
    {
        return ((id ^ code1) & mask1) == 0 || ((id ^ code2) & mask2) == 0;
    }

    // TWAI dual hardware filter: ID bits 28..13 of each, set mask bits are don't care
    uint32_t hwCode() const { return ((code1 >> 13) << 16) | ((code2 >> 13) & 0xFFFF); }
    uint32_t hwMask() const { return ~(((mask1 >> 13) << 16) | ((mask2 >> 13) & 0xFFFF)); }
};
