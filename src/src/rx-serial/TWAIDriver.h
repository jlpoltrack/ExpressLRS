#pragma once

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)

#include "CANTypes.h"

// Minimal TWAI (CAN) driver for 1Mbit/s extended frames with the ISR in IRAM, so frames keep
// being received while the flash cache is off (flash erase/write, OTA)
class TWAIDriver {
public:
    static bool begin(int8_t txPin, int8_t rxPin, const CANFilter &filter);
    static void end();

    // Call from the main loop, handles bus-off recovery and a lost TX interrupt
    static void poll();
    static bool receive(CANFrame &frame);
    // Returns false if the TX queue is full
    static bool transmit(const CANFrame &frame);
    // Drops queued frames and aborts the one being sent
    static void flushTx();
    // Frames acknowledged by another node, wraps
    static uint32_t txCompletedCount();
    static bool txQueued();

    static CANState state();
    static uint8_t txErrorCounter();
    static uint8_t rxErrorCounter();
    // Returns and resets the counters
    static CANStats takeStats();
};

#endif
