// Prints received frames and sends one every second, CAN 2.0 at 1 Mbit/s
#include <Arduino.h>
#include <mcp251xfd.h>

static mcp251xfd_t can;

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
    if (!mcp251xfd_begin(&can, &cfg))
        Serial.println("MCP251xFD not found");
}

void loop()
{
    mcp251xfd_service(&can);

    mcp251xfd_frame_t frame;
    while (mcp251xfd_receive(&can, &frame))
        Serial.printf("rx %08lx len %u\n", (unsigned long)frame.id, frame.len);

    static uint32_t tlast_ms = 0;
    if (millis() - tlast_ms >= 1000)
    {
        tlast_ms = millis();
        frame = {};
        frame.id = 0x1234567;
        frame.len = 8;
        frame.data[0] = (uint8_t)(tlast_ms / 1000);
        mcp251xfd_transmit(&can, &frame);
        const mcp251xfd_stats_t &s = can.stats;
        Serial.printf("state %d tx %lu done %lu tec %u rec %u\n", mcp251xfd_state(&can),
                      (unsigned long)s.tx_frames, (unsigned long)s.tx_completed, s.tec, s.rec);
    }
}
