#pragma once
#include "targets.h"

#if defined(PLATFORM_RP2)
#include <Arduino.h>
#include <hardware/uart.h>

// ESP32-style begin for a RP2 UART. A pin that is UNDEF_PIN or not routable to this UART is
// left unassigned instead of reaching setRX/setTX, which panic on it
void RP2UartBegin(SerialUART &port, unsigned long baud, uint16_t config, int8_t rxPin, int8_t txPin, bool invert);

// Non-blocking TX for a RP2 UART. SerialUART::write() spins ~24us per byte on a full FIFO and the callers
// are the hwTimer ISR and sendQueuedData() with IRQs off, so bytes go into a ring that drains opportunistically
class RP2SerialTx final : public Stream
{
public:
    RP2SerialTx(SerialUART &port, uart_inst_t *uart) : _port(port), _uartHw(uart) {}

    // Same arguments as the ESP32 HardwareSerial::begin, so callers need not care
    void begin(unsigned long baud, uint16_t config, int8_t rxPin, int8_t txPin, bool invert);
    // Drains the ring, then closes the UART
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

    SerialUART &_port;
    uart_inst_t *const _uartHw;
    uart_inst_t *_uart = nullptr;
    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;
    volatile bool _servicing = false;
    uint8_t _buf[RING_SIZE];
};

extern RP2SerialTx SerialTxRing;
#endif
