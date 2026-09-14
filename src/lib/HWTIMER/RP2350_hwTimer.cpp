#if defined(PLATFORM_RP2350)
#include "hwTimer.h"
#include "logging.h"
#include <pico/time.h>

void (*hwTimer::callbackTick)() = nullptr;
void (*hwTimer::callbackTock)() = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

// Interval, PhaseShift and FreqOffset are in ticks, matching the ESP32 RX resolution of 0.2us
#if defined(TARGET_RX)
#define HWTIMER_TICKS_PER_US 5
#else
#define HWTIMER_TICKS_PER_US 1
#endif

static alarm_id_t alarmId = -1;
static void (*timerIsr)() = nullptr;
static volatile int64_t nextAlarmTicks = 0;
// Sub-microsecond part of the schedule carried between alarms, the hardware alarm is 1us resolution
static volatile int64_t tickRemainder = 0;

// Returning a negative value reschedules relative to the previous target time, so there is no drift
static int64_t ICACHE_RAM_ATTR alarmCallback(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;
    nextAlarmTicks = 0;
    timerIsr();
    if (nextAlarmTicks <= 0)
    {
        return 0;
    }
    const int64_t ticks = nextAlarmTicks + tickRemainder;
    tickRemainder = ticks % HWTIMER_TICKS_PER_US;
    return -(ticks / HWTIMER_TICKS_PER_US);
}

void hwTimer::init(void (*callbackTick)(), void (*callbackTock)())
{
    hwTimer::callbackTick = callbackTick;
    hwTimer::callbackTock = callbackTock;
    timerIsr = &hwTimer::callback;
    DBGLN("hwTimer Init");
}

void hwTimer::stop()
{
    if (running)
    {
        running = false;
        if (alarmId > 0)
            cancel_alarm(alarmId);
        alarmId = -1;
        DBGLN("hwTimer stop");
    }
}

void hwTimer::resume()
{
    if (!running)
    {
        // tock() should always be the first event to maintain consistency
        isTick = false;
        tickRemainder = 0;
        running = true;
        alarmId = add_alarm_in_us(1, alarmCallback, nullptr, true);
        DBGLN("hwTimer resume");
    }
}

void hwTimer::updateInterval(uint32_t time)
{
    // timer should not be running when updateInterval() is called
    HWtimerInterval = time * HWTIMER_TICKS_PER_US;
    DBGLN("hwTimer interval: %d", time);
}

void hwTimer::phaseShift(int32_t newPhaseShift)
{
    const int32_t minTicks = -(HWtimerInterval >> 2);
    const int32_t maxTicks = (HWtimerInterval >> 2);

    // Convert microseconds to ticks first, then clamp to the valid tick range
    const int32_t newPhaseShiftTicks = newPhaseShift * HWTIMER_TICKS_PER_US;
    PhaseShift = constrain(newPhaseShiftTicks, minTicks, maxTicks);
}

void ICACHE_RAM_ATTR hwTimer::callback()
{
    if (!running)
    {
        return;
    }
#if defined(TARGET_TX)
    callbackTock();
    nextAlarmTicks = HWtimerInterval;
#else
    int64_t nextInterval = (HWtimerInterval >> 1) + FreqOffset;
    if (isTick)
    {
        callbackTick();
    }
    else
    {
        nextInterval += PhaseShift;
        PhaseShift = 0;
        callbackTock();
    }
    isTick = !isTick;
    nextAlarmTicks = nextInterval;
#endif
}

#endif
