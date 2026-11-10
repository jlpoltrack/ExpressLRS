#pragma once

#if defined(TARGET_RX) && defined(PLATFORM_RP2) && defined(CAN_MCP251XFD)

#include "CANTypes.h"

// CAN FD through an MCP251863/MCP2518FD on SPI, same interface as CAN2040Driver: 1 Mbit/s nominal,
// 4 Mbit/s data, extended frames. Every call must come from the core that calls begin()
class MCP251XFDDriver {
public:
    // The SPI and INT pins come from the hardware layout, txPin and rxPin are not used
    static bool begin(int8_t txPin, int8_t rxPin, const CANFilter &filter);
    static void end();

    static void poll();
    static bool receive(CANFrame &frame);
    // Returns false if the chip TX FIFO is full
    static bool transmit(const CANFrame &frame);
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
