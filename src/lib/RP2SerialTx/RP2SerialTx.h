#pragma once
#include "targets.h"

#if defined(PLATFORM_RP2)
#include <Arduino.h>
#include <hardware/uart.h>

/**
 * Non-blocking TX for a RP2 UART.
 *
 * arduino-pico's SerialUART::write() spins in uart_putc_raw() while the 32-byte FIFO
 * is full, ~24us per byte, and the callers are the hwTimer ISR (sendImmediateRC) and
 * sendQueuedData() with interrupts off. Writing into a ring instead costs a memcpy.
 *
 * TXIM stays off, so the ring drains opportunistically from write() and
 * handleSerialIO(); the FIFO holds ~760us at 420000 baud, longer than a loop().
 */
class RP2SerialTx final : public Stream
{
public:
    void begin(uart_inst_t *uart);
    void end();

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int availableForWrite() override;
    void flush() override;
    // Moves buffered bytes into the FIFO, safe to call when idle
    void service();

    // Output only, reads go through the separate input Stream
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }

private:
    // sendQueuedData() can present 128 bytes in one call (getMaxSerialWriteSize), and a
    // 26-byte RC frame may already be resident, so the ring has to hold ~154 worst case
    static constexpr uint16_t RING_SIZE = 256;
    static constexpr uint16_t RING_MASK = RING_SIZE - 1;
    // Caps on how long either side can run in one go, so neither becomes a long
    // interrupt-off window nor a long uninterruptible burst
    static constexpr uint16_t SERVICE_MAX_BYTES = 16;
    static constexpr uint16_t PRODUCER_CHUNK = 32;

    uart_inst_t *_uart = nullptr;
    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;
    volatile bool _servicing = false;
    uint8_t _buf[RING_SIZE];
};

extern RP2SerialTx SerialTxRing;
#endif
