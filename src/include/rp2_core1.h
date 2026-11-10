#pragma once

#if defined(PLATFORM_RP2)

#include <hardware/sync.h>

// Work handed to core 1. start() and stop() run there, so IRQs they enable belong to core 1; service()
// runs on every wake (its IRQs, RP2Core1::wake(), the 1ms tick, core 0's alarm pool __sev() after each hwTimer IRQ)
class RP2Core1Task
{
public:
    virtual void start() = 0;
    virtual void service() = 0;
    virtual void stop() = 0;
};

namespace RP2Core1
{
    // Called from core 0, block until core 1 has run the task's start() / stop()
    void start(RP2Core1Task *task);
    void stop();
    // Called from core 0 after handing core 1 new work, so it is serviced without waiting for the tick
    inline void wake() { __sev(); }
}

#endif
