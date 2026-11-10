#if defined(TARGET_RX) && defined(PLATFORM_RP2)

#include "CAN2040Driver.h"

#include <atomic>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>

extern "C" {
#include "can2040.h"
}

// can2040 takes a whole PIO block: PIO2 on the RP2350 (NeoPixelBus uses PIO1), PIO0 on the RP2040
#if NUM_PIOS > 2
#define CAN_PIO_NUM 2
#else
#define CAN_PIO_NUM 0
#endif
#define CAN_PIO_SM_MASK 0x0F
#define CAN_BITRATE     1000000

// can2040's flags are an enum, so 1 << 31 is negative
static constexpr uint32_t ID_EFF = (uint32_t)CAN2040_ID_EFF;
static constexpr uint32_t ID_RTR = (uint32_t)CAN2040_ID_RTR;

// can2040 has no RX FIFO of its own, so this is all the buffering between the IRQ and the node
#define RX_RING_LEN 128
static_assert((RX_RING_LEN & (RX_RING_LEN - 1)) == 0, "Ring length must be a power of 2");

static can2040 cbus;
static CANFilter filter;
static int8_t canTxPin = -1, canRxPin = -1;
static volatile CANState busState = CAN_STOPPED;

// Everything below is shared with the IRQ, which runs on the same core as all the callers.
// rxHead and the TX/RX counters are advanced by the IRQ, rxTail and txSubmitted by the caller.
static CANFrame rxRing[RX_RING_LEN];
static std::atomic<uint32_t> rxHead, rxTail;
static volatile uint32_t txSubmitted, txCompleted;
static CANStats stats;
static uint32_t lastParseErrors;

static void canCallback(can2040 *cd, uint32_t notify, can2040_msg *msg)
{
    (void)cd;
    if (notify == CAN2040_NOTIFY_RX)
    {
        const uint32_t id = msg->id & ~(ID_EFF | ID_RTR);
        // Standard and remote frames are not DroneCAN; no hardware filter, so this is the only one
        if ((msg->id & (ID_EFF | ID_RTR)) != ID_EFF || !filter.matches(id))
            return;

        const uint32_t head = rxHead.load(std::memory_order_relaxed);
        if (head - rxTail.load(std::memory_order_acquire) >= RX_RING_LEN)
        {
            stats.rxQueueFull++;
            return;
        }
        CANFrame &f = rxRing[head & (RX_RING_LEN - 1)];
        f.id = id;
        f.dlc = msg->dlc > 8 ? 8 : msg->dlc;
        memcpy(f.data, msg->data, f.dlc);
        rxHead.store(head + 1, std::memory_order_release);
    }
    else if (notify == CAN2040_NOTIFY_TX)
    {
        txCompleted = txCompleted + 1;
    }
    else if (notify == CAN2040_NOTIFY_ERROR)
    {
        stats.rxOverrun++;
    }
}

static void pioIrqHandler()
{
    can2040_pio_irq_handler(&cbus);
}

static uint pioIrqNum()
{
    return pio_get_irq_num(pio_get_instance(CAN_PIO_NUM), 0);
}

// Also clears can2040's TX queue and statistics
static void startCan()
{
    can2040_setup(&cbus, CAN_PIO_NUM);
    can2040_callback_config(&cbus, canCallback);
    lastParseErrors = 0;
    can2040_start(&cbus, clock_get_hz(clk_sys), CAN_BITRATE, canRxPin, canTxPin);
}

bool CAN2040Driver::begin(int8_t txPin, int8_t rxPin, const CANFilter &acceptFilter)
{
    if (busState != CAN_STOPPED)
        return false;

    PIO pio = pio_get_instance(CAN_PIO_NUM);
    for (uint sm = 0; sm < 4; ++sm)
    {
        if (pio_sm_is_claimed(pio, sm))
            return false;
    }
    pio_claim_sm_mask(pio, CAN_PIO_SM_MASK);

    filter = acceptFilter;
    // On the RP2350B both pins must lie in GPIO 0-31 or both in 16-47, or can2040 silently goes receive-only
    canTxPin = txPin;
    canRxPin = rxPin;
    rxHead = rxTail = 0;
    txSubmitted = txCompleted = 0;
    stats = {};

    // can2040 needs low IRQ latency, nothing else on this core may preempt it
    const uint irq = pioIrqNum();
    irq_set_exclusive_handler(irq, pioIrqHandler);
    irq_set_priority(irq, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(irq, true);

    startCan();
    busState = CAN_RUNNING;
    return true;
}

void CAN2040Driver::end()
{
    if (busState == CAN_STOPPED)
        return;

    can2040_stop(&cbus);
    const uint irq = pioIrqNum();
    irq_set_enabled(irq, false);
    irq_remove_handler(irq, pioIrqHandler);

    // can2040_stop() leaves the state machines running and the pins on the PIO function
    PIO pio = pio_get_instance(CAN_PIO_NUM);
    pio_set_sm_mask_enabled(pio, CAN_PIO_SM_MASK, false);
    for (uint sm = 0; sm < 4; ++sm)
        pio_sm_unclaim(pio, sm);
    gpio_deinit(canTxPin);
    gpio_deinit(canRxPin);
    busState = CAN_STOPPED;
}

bool CAN2040Driver::receive(CANFrame &frame)
{
    const uint32_t tail = rxTail.load(std::memory_order_relaxed);
    if (rxHead.load(std::memory_order_acquire) == tail)
        return false;
    frame = rxRing[tail & (RX_RING_LEN - 1)];
    rxTail.store(tail + 1, std::memory_order_release);
    return true;
}

bool CAN2040Driver::transmit(const CANFrame &frame)
{
    if (busState != CAN_RUNNING || !can2040_check_transmit(&cbus))
        return false;

    can2040_msg msg;
    msg.id = frame.id | ID_EFF;
    msg.dlc = frame.dlc;
    memcpy(msg.data, frame.data, frame.dlc);
    if (can2040_transmit(&cbus, &msg) < 0)
        return false;
    txSubmitted = txSubmitted + 1;
    return true;
}

void CAN2040Driver::flushTx()
{
    if (busState != CAN_RUNNING)
        return;

    const uint irq = pioIrqNum();
    irq_set_enabled(irq, false);
    can2040_stop(&cbus);
    // startCan() zeroes can2040's counters, keep the errors seen so far
    can2040_stats canStats;
    can2040_get_statistics(&cbus, &canStats);
    stats.busErrors += canStats.parse_error - lastParseErrors;
    // As on TWAI the frame in flight counts as failed, even if it was acknowledged during the stop
    // (can2040 then sends no NOTIFY_TX); frames still queued behind it are dropped uncounted
    if (txSubmitted != txCompleted)
        stats.txFailed++;
    txSubmitted = txCompleted;
    startCan();
    irq_set_enabled(irq, true);
}

uint32_t CAN2040Driver::txCompletedCount()
{
    return txCompleted;
}

bool CAN2040Driver::txQueued()
{
    return txSubmitted != txCompleted;
}

CANState CAN2040Driver::state()
{
    return busState;
}

CANStats CAN2040Driver::takeStats()
{
    can2040_stats canStats;
    const uint32_t save = save_and_disable_interrupts();
    can2040_get_statistics(&cbus, &canStats);
    stats.busErrors += canStats.parse_error - lastParseErrors;
    lastParseErrors = canStats.parse_error;
    const CANStats s = stats;
    stats = {};
    restore_interrupts(save);
    return s;
}

#endif
