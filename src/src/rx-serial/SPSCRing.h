#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

/**
 * Lock-free byte ring for one producer and one consumer, which may run on different cores.
 * free() and push() are for the producer only, the rest for the consumer only.
 */
template <uint32_t N>
class SPSCRing
{
    static_assert((N & (N - 1)) == 0, "SPSCRing size must be a power of 2");

public:
    uint32_t free() const { return N - (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_acquire)); }
    uint32_t size() const { return head.load(std::memory_order_acquire) - tail.load(std::memory_order_relaxed); }

    // Caller checks free() first
    void push(const uint8_t *data, uint32_t len)
    {
        const uint32_t h = head.load(std::memory_order_relaxed);
        copyIn(h & (N - 1), data, len);
        head.store(h + len, std::memory_order_release);
    }

    int peek() const { return size() ? buf[tail.load(std::memory_order_relaxed) & (N - 1)] : -1; }

    // Caller checks size() first
    void pop(uint8_t *data, uint32_t len)
    {
        const uint32_t t = tail.load(std::memory_order_relaxed);
        copyOut(t & (N - 1), data, len);
        tail.store(t + len, std::memory_order_release);
    }

    void discard() { tail.store(head.load(std::memory_order_acquire), std::memory_order_release); }

private:
    void copyIn(uint32_t pos, const uint8_t *data, uint32_t len)
    {
        const uint32_t first = len < N - pos ? len : N - pos;
        memcpy(&buf[pos], data, first);
        memcpy(buf, data + first, len - first);
    }

    void copyOut(uint32_t pos, uint8_t *data, uint32_t len) const
    {
        const uint32_t first = len < N - pos ? len : N - pos;
        memcpy(data, &buf[pos], first);
        memcpy(data + first, buf, len - first);
    }

    uint8_t buf[N];
    std::atomic<uint32_t> head {0};
    std::atomic<uint32_t> tail {0};
};
