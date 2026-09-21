#include "RP2SerialTx.h"

#if defined(PLATFORM_RP2)
#include <hardware/sync.h>

RP2SerialTx SerialTxRing;


void RP2SerialTx::begin(uart_inst_t *uart)
{
    const uint32_t save = save_and_disable_interrupts();
    _uart = uart;
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

    size_t remaining = size;

    // Fast path: with the ring empty the FIFO can take bytes directly. The check and the
    // writes share one critical section, otherwise a preempting producer that also saw an
    // empty ring would interleave its bytes into the middle of this frame.
    {
        const uint32_t save = save_and_disable_interrupts();
        if (_head == _tail)
        {
            uart_hw_t *const hw = uart_get_hw(_uart);
            size_t direct = 0;
            while (direct < remaining && (hw->fr & UART_UARTFR_TXFF_BITS) == 0)
                hw->dr = buffer[direct++];
            buffer += direct;
            remaining -= direct;
        }
        restore_interrupts(save);
    }

    while (remaining)
    {
        const uint32_t save = save_and_disable_interrupts();
        uint16_t head = _head;
        const uint16_t tail = _tail;
        size_t copied = 0;
        const size_t chunk = remaining < PRODUCER_CHUNK ? remaining : PRODUCER_CHUNK;
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
