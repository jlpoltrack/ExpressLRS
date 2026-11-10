#include "RP2SerialTx.h"

#if defined(PLATFORM_RP2)
#include <hardware/sync.h>

// Serial is USB CDC on RP2, the CRSF UART is Serial1 (hardware UART0)
RP2SerialTx SerialTxRing(Serial1, uart0);

// Per-UART pin tables, mirroring the core's SerialUART::setTX. RX is always TX plus one.
#if defined(PICO_RP2350) && !PICO_RP2350A // RP2350B
static constexpr uint64_t UART_TX_PINS[2] = {
    __bitset({0, 2, 12, 14, 16, 18, 28, 30, 32, 34, 44, 46}),
    __bitset({4, 6, 8, 10, 20, 22, 24, 26, 36, 38, 40, 42})};
#else // RP2350A
static constexpr uint64_t UART_TX_PINS[2] = {
    __bitset({0, 2, 12, 14, 16, 18, 28}),
    __bitset({4, 6, 8, 10, 20, 22, 24, 26})};
#endif

static pin_size_t uartPin(int8_t pin, uint64_t validMask)
{
    return (pin >= 0 && pin < 64 && (validMask & (1ULL << pin))) ? (pin_size_t)pin : UART_PIN_NOT_DEFINED;
}

void RP2UartBegin(SerialUART &port, unsigned long baud, uint16_t config, int8_t rxPin, int8_t txPin, bool invert)
{
    const uint64_t txPins = UART_TX_PINS[&port == &Serial1 ? 0 : 1];
    port.setRX(uartPin(rxPin, txPins << 1));
    port.setTX(uartPin(txPin, txPins));
    port.setInvertTX(invert);
    port.setInvertRX(invert);
    port.setFIFOSize(256);
    port.begin(baud, config);
}

void RP2SerialTx::begin(unsigned long baud, uint16_t config, int8_t rxPin, int8_t txPin, bool invert)
{
    RP2UartBegin(_port, baud, config, rxPin, txPin, invert);
    const uint32_t save = save_and_disable_interrupts();
    _uart = _uartHw;
    _head = _tail = 0;
    restore_interrupts(save);
}

void RP2SerialTx::end()
{
    flush();
    const uint32_t save = save_and_disable_interrupts();
    _uart = nullptr;
    _head = _tail = 0;
    restore_interrupts(save);
    _port.end();
}

void ICACHE_RAM_ATTR RP2SerialTx::service()
{
    if (_uart == nullptr)
        return;

    // Claim the consumer role. Only the holder touches _tail, so the FIFO fill itself
    // runs with interrupts enabled; masking covers just this flag.
    const uint32_t save = save_and_disable_interrupts();
    if (_servicing)
    {
        restore_interrupts(save);
        return;
    }
    _servicing = true;
    restore_interrupts(save);

    uart_hw_t *const hw = uart_get_hw(_uart);
    uint16_t moved = 0;
    while (_head != _tail && (hw->fr & UART_UARTFR_TXFF_BITS) == 0 && moved < SERVICE_MAX_BYTES)
    {
        hw->dr = _buf[_tail];
        _tail = (_tail + 1) & RING_MASK;
        moved++;
    }
    _servicing = false;
}

size_t ICACHE_RAM_ATTR RP2SerialTx::write(const uint8_t *buffer, size_t size)
{
    if (_uart == nullptr)
        return 0;

    uart_hw_t *const hw = uart_get_hw(_uart);
    size_t remaining = size;

    while (remaining)
    {
        const uint32_t save = save_and_disable_interrupts();
        uint16_t head = _head;
        const uint16_t tail = _tail;
        size_t copied = 0;
        // With the ring empty the FIFO takes bytes directly. Same critical section as the ring fill,
        // or a preempting producer that also saw an empty ring would land inside this frame
        if (head == tail)
        {
            while (copied < remaining && (hw->fr & UART_UARTFR_TXFF_BITS) == 0)
                hw->dr = buffer[copied++];
        }
        const size_t chunk = remaining < copied + PRODUCER_CHUNK ? remaining : copied + PRODUCER_CHUNK;
        while (copied < chunk)
        {
            const uint16_t next = (head + 1) & RING_MASK;
            if (next == tail)
                break; // full, one slot is always left unused to separate the indices
            _buf[head] = buffer[copied++];
            head = next;
        }
        _head = head;
        restore_interrupts(save);

        buffer += copied;
        remaining -= copied;
        service();
        // Only spins if the ring stayed full, which needs a sustained overrun
    }
    return size;
}

size_t ICACHE_RAM_ATTR RP2SerialTx::write(uint8_t c)
{
    return write(&c, 1);
}

int RP2SerialTx::availableForWrite()
{
    const uint32_t save = save_and_disable_interrupts();
    const uint16_t used = (_head - _tail) & RING_MASK;
    restore_interrupts(save);
    return RING_MASK - used;
}

void RP2SerialTx::flush()
{
    if (_uart == nullptr)
        return;
    while (_head != _tail)
        service();
    while (uart_get_hw(_uart)->fr & UART_UARTFR_BUSY_BITS)
        tight_loop_contents();
}
#endif
