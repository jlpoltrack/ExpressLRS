// Interrupt driven receive: the INT pin interrupt only flags pending frames, the loop reads them
#include <Arduino.h>
#include <mcp251xfd.h>

static mcp251xfd_t can;
static volatile bool can_rx = false;

static void on_can_irq(void *ctx)
{
    (void)ctx;
    can_rx = true; // no driver calls in interrupt context
}

void setup()
{
    Serial.begin(115200);

    mcp251xfd_config_t cfg = {};
    cfg.spi = spi0;
    cfg.sck_pin = 2;
    cfg.mosi_pin = 3;
    cfg.miso_pin = 4;
    cfg.cs_pin = 5;
    cfg.int_pin = 1;
    cfg.spi_hz = 16000000;
    cfg.tx_attempts = MCP251XFD_TX_ATTEMPTS_THREE;
    cfg.irq_callback = on_can_irq;
    if (!mcp251xfd_begin(&can, &cfg))
        Serial.println("MCP251xFD not found");
}

void loop()
{
    mcp251xfd_service(&can); // still needed for recovery and tx tracking

    if (can_rx)
    {
        can_rx = false; // cleared first, so a frame arriving while draining flags it again
        mcp251xfd_frame_t frame;
        while (mcp251xfd_receive(&can, &frame)) // rearms the interrupt when it returns false
            Serial.printf("rx %08lx len %u\n", (unsigned long)frame.id, frame.len);
    }
}
