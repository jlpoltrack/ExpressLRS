#include "MCP251XFDDriver.h"

#if defined(TARGET_RX) && defined(PLATFORM_RP2) && defined(CAN_MCP251XFD)

#include <string.h>

#include "common.h"
#include "logging.h"
#include "mcp251xfd.h"

#define CAN_SPI_HZ  16000000

static mcp251xfd_t dev;
static mcp251xfd_stats_t lastStats;

// Entering the INT pin IRQ is what wakes core 1 from its WFE, receive() rearms it
static void onCanIrq(void *ctx)
{
    (void)ctx;
}

static spi_inst_t *spiForSck(int8_t sck)
{
    // GPIO 0-7 and 16-23 route to spi0, 8-15 and 24-31 to spi1, as in SPIEx
    return ((sck / 8) % 2) ? spi1 : spi0;
}

bool MCP251XFDDriver::begin(int8_t txPin, int8_t rxPin, const CANFilter &filter)
{
    (void)txPin;
    (void)rxPin;
    if (dev.state != MCP251XFD_STOPPED)
        return false;

    const int pins[] = {GPIO_PIN_CAN_SCK, GPIO_PIN_CAN_MOSI, GPIO_PIN_CAN_MISO, GPIO_PIN_CAN_CS, GPIO_PIN_CAN_INT};
    for (const int pin : pins)
    {
        if (pin == UNDEF_PIN)
        {
            ERRLN("CAN: can_sck/mosi/miso/cs/int missing from the layout");
            return false;
        }
    }
    // The radio's SPI is driven from core 0, it cannot be shared
    if (GPIO_PIN_SCK != UNDEF_PIN && spiForSck(GPIO_PIN_SCK) == spiForSck(GPIO_PIN_CAN_SCK))
    {
        ERRLN("CAN: the MCP251863 must be on the other SPI from the radio");
        return false;
    }

    // Tunnel broadcasts and service frames get a FIFO each, so one cannot crowd out the other
    const mcp251xfd_filter_t filters[] = {
        {filter.code1, filter.mask1, 0},
        {filter.code2, filter.mask2, 1},
    };
    mcp251xfd_config_t cfg {};
    cfg.spi = spiForSck(GPIO_PIN_CAN_SCK);
    cfg.sck_pin = GPIO_PIN_CAN_SCK;
    cfg.mosi_pin = GPIO_PIN_CAN_MOSI;
    cfg.miso_pin = GPIO_PIN_CAN_MISO;
    cfg.cs_pin = GPIO_PIN_CAN_CS;
    cfg.int_pin = GPIO_PIN_CAN_INT;
    cfg.spi_hz = CAN_SPI_HZ;
    cfg.canfd = true;
    // As on TWAI, the node flushes frames nobody acknowledges
    cfg.tx_attempts = MCP251XFD_TX_ATTEMPTS_UNLIMITED;
    cfg.filters = filters;
    cfg.filter_num = 2;
    cfg.irq_callback = onCanIrq;

    lastStats = {};
    // Without a chip the driver stays NOT_FOUND and poll() retries every second
    mcp251xfd_begin(&dev, &cfg);
    return dev.state != MCP251XFD_STOPPED;
}

void MCP251XFDDriver::end()
{
    mcp251xfd_end(&dev);
}

void MCP251XFDDriver::poll()
{
    mcp251xfd_service(&dev);
}

bool MCP251XFDDriver::receive(CANFrame &frame)
{
    mcp251xfd_frame_t f;
    if (!mcp251xfd_receive(&dev, &f))
        return false;
    frame.id = f.id;
    frame.dlc = f.len;
    frame.fd = f.fdf;
    memcpy(frame.data, f.data, f.len);
    return true;
}

bool MCP251XFDDriver::transmit(const CANFrame &frame)
{
    mcp251xfd_frame_t f;
    f.id = frame.id;
    f.len = frame.dlc;
    f.fdf = frame.fd;
    memcpy(f.data, frame.data, frame.dlc);
    return mcp251xfd_transmit(&dev, &f) == 1;
}

void MCP251XFDDriver::flushTx()
{
    mcp251xfd_flush_tx(&dev);
}

uint32_t MCP251XFDDriver::txCompletedCount()
{
    return dev.stats.tx_completed;
}

bool MCP251XFDDriver::txQueued()
{
    return mcp251xfd_tx_pending(&dev) != 0;
}

CANState MCP251XFDDriver::state()
{
    switch (mcp251xfd_state(&dev))
    {
    case MCP251XFD_RUNNING:
        return CAN_RUNNING;
    case MCP251XFD_BUS_OFF:
        return CAN_BUS_OFF;
    case MCP251XFD_NOT_FOUND:
        return CAN_RECOVERING;
    default:
        return CAN_STOPPED;
    }
}

uint8_t MCP251XFDDriver::txErrorCounter()
{
    return dev.stats.tec;
}

uint8_t MCP251XFDDriver::rxErrorCounter()
{
    return dev.stats.rec;
}

CANStats MCP251XFDDriver::takeStats()
{
    const mcp251xfd_stats_t &s = dev.stats;
    CANStats out {};
    out.rxOverrun = s.rx_overflows - lastStats.rx_overflows;
    out.txFailed = (s.tx_aborted - lastStats.tx_aborted) + (s.tx_limit_hits - lastStats.tx_limit_hits);
    out.busOff = s.bus_offs - lastStats.bus_offs;
    out.errataResets = s.reconfigs - lastStats.reconfigs;
    out.busErrors = (s.data_tx_errors - lastStats.data_tx_errors) + (s.data_rx_errors - lastStats.data_rx_errors);
    lastStats = s;
    return out;
}

#endif
