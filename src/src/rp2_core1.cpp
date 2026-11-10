#if defined(PLATFORM_RP2)

#include "rp2_core1.h"

#include <Arduino.h>
#include <atomic>
#include <hardware/structs/scb.h>
#include <hardware/timer.h>

// SysTick is taken by the core's cycle counter. The tick uses TIMER1 where there is one, else its own
// alarm (and alarm IRQ) on the RP2040's single timer, so it never shares an IRQ with core 0's alarm pool
#define TICK_INTERVAL_US 1000
#if NUM_GENERIC_TIMERS > 1
#define TICK_TIMER timer1_hw
#define SCR_SEVONPEND_BITS M33_SCR_SEVONPEND_BITS
#else
#define TICK_TIMER timer_hw
#define SCR_SEVONPEND_BITS M0PLUS_SCR_SEVONPEND_BITS
#endif

static std::atomic<RP2Core1Task *> requested {nullptr};
static std::atomic<RP2Core1Task *> running {nullptr};
static int tickAlarm = -1;
static volatile bool ticking = false;

static void armTick()
{
    // A target already past arms nothing, so retry rather than lose the tick for good
    while (timer_hardware_alarm_set_target(TICK_TIMER, tickAlarm, from_us_since_boot(timer_time_us_64(TICK_TIMER) + TICK_INTERVAL_US)))
        tight_loop_contents();
}

// Entering the IRQ is what wakes loop1(), there is nothing else to do
static void tickCallback(uint alarm)
{
    (void)alarm;
    if (ticking)
        armTick();
}

void RP2Core1::start(RP2Core1Task *task)
{
    requested.store(task, std::memory_order_release);
    wake();
    while (running.load(std::memory_order_acquire) != task)
        tight_loop_contents();
}

void RP2Core1::stop()
{
    start(nullptr);
}

// Defining these launches core 1 rather than leaving it in the bootrom's FIFO poll loop, which
// also cut the hwTimer alarm latency on core 0 from 6.03us to 2.29us
void setup1()
{
    // Any IRQ becoming pending sets the event register, so one taken while service() runs still wakes the WFE
    scb_hw->scr |= SCR_SEVONPEND_BITS;
    tickAlarm = timer_hardware_alarm_claim_unused(TICK_TIMER, true);
    timer_hardware_alarm_set_callback(TICK_TIMER, tickAlarm, tickCallback);
}

void loop1()
{
    RP2Core1Task *const want = requested.load(std::memory_order_acquire);
    RP2Core1Task *const current = running.load(std::memory_order_relaxed);
    if (want != current)
    {
        if (current)
        {
            ticking = false;
            timer_hardware_alarm_cancel(TICK_TIMER, tickAlarm);
            current->stop();
        }
        if (want)
        {
            want->start();
            ticking = true;
            armTick();
        }
        running.store(want, std::memory_order_release);
    }

    if (want)
        want->service();
    __wfe();
}

#endif
