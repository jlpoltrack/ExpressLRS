#pragma once

#if defined(TARGET_RX) && defined(PLATFORM_RP2)

#include "CANTypes.h"

// CAN at 1Mbit/s, extended frames, on a PIO block via can2040, same interface as TWAIDriver.
// The PIO IRQ lands on the core that calls begin(), and every other call must come from that core
class CAN2040Driver {
public:
    static bool begin(int8_t txPin, int8_t rxPin, const CANFilter &filter);
    static void end();

    static void poll() {}
    static bool receive(CANFrame &frame);
    // Returns false if can2040's 4 frame TX queue is full
    static bool transmit(const CANFrame &frame);
    // Drops queued frames, restarting can2040 is the only way to abort a transmit
    static void flushTx();
    // Frames acknowledged by another node, wraps
    static uint32_t txCompletedCount();
    static bool txQueued();

    static CANState state();
    // can2040 does not keep error counters
    static uint8_t txErrorCounter() { return 0; }
    static uint8_t rxErrorCounter() { return 0; }
    // Returns and resets the counters
    static CANStats takeStats();
};

#endif
