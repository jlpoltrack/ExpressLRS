//*******************************************************
// MCP2518FD/MCP251863 CAN FD controller on SPI, for the RP2040/RP2350 with the pico SDK
//*******************************************************
// Extended (29 bit) data frames only. The level of the chip's INT pin tells if there are frames to
// read, an optional interrupt callback signals them. Not thread safe, all calls must come from one core.
// Expects a 40 MHz chip clock, from a crystal or e.g. a PWM output.
//*******************************************************
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <hardware/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP251XFD_FILTERS_MAX   8
#define MCP251XFD_DATA_LEN_MAX  64
#define MCP251XFD_IRQ_DEVS_MAX  2   // instances that can use irq_callback at the same time


typedef enum {
    MCP251XFD_TX_ATTEMPTS_ONE = 0,       // no retransmission
    MCP251XFD_TX_ATTEMPTS_THREE = 1,
    MCP251XFD_TX_ATTEMPTS_UNLIMITED = 3, // retried until acknowledged or flushed
} mcp251xfd_tx_attempts_t;


// Set mask bits must match, fifo is 0 or 1
typedef struct {
    uint32_t id;
    uint32_t mask;
    uint8_t fifo;
} mcp251xfd_filter_t;


typedef struct {
    spi_inst_t *spi;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t cs_pin;
    uint8_t int_pin;        // INT, not INT0 or INT1, low while a rx fifo is not empty
    uint32_t spi_hz;        // 17 MHz max with a 40 MHz clock, 16 MHz recommended
    bool canfd;             // mixed CAN FD / classic mode with 64 byte objects, else CAN 2.0 with 8 byte objects
    bool standby;           // only reset the chip into low power mode, the transceiver stays in standby
    mcp251xfd_tx_attempts_t tx_attempts;
    uint8_t tx_fifo_size;   // objects, 1...32, 0 = default: 32 for CAN 2.0, 8 for CAN FD
    uint8_t rx_fifo_size;   // objects per rx fifo, 1...32, 0 = default: 32 for CAN 2.0, 10 for CAN FD
    uint32_t nbtcfg;        // raw bit timing registers, 0 = 1 Mbit/s nominal (90% sample point),
    uint32_t dbtcfg;        // 4 Mbit/s data (80%), with a 40 MHz clock
    uint32_t tdc;
    const mcp251xfd_filter_t *filters; // copied, NULL or 0 filters accepts all extended frames into fifo 0
    uint8_t filter_num;
    // NULL = polled. Else called in interrupt context, on the core that called mcp251xfd_begin(), when
    // frames are pending. It must not call the driver: call mcp251xfd_receive() until false, then it rearms
    void (*irq_callback)(void *ctx);
    void *irq_ctx;
} mcp251xfd_config_t;


typedef struct {
    uint32_t id;            // 29 bit extended id
    uint8_t len;            // in bytes, rounded up to a valid CAN FD length on transmit
    bool fdf;               // CAN FD frame, sent with bit rate switching
    uint8_t data[MCP251XFD_DATA_LEN_MAX];
} mcp251xfd_frame_t;


// Counters are cumulative and wrap, tec, rec, opmode and iocon are the current values
typedef struct {
    uint32_t rx_frames;
    uint32_t rx_overflows;  // chip rx fifo overflows, each drops frames
    uint32_t rx_dropped;    // remote frames, classic frames with DLC > 8, frames hit by an ecc error
    uint32_t tx_frames;     // handed to the chip
    uint32_t tx_completed;  // sent and acknowledged
    uint32_t tx_aborted;    // dropped from the chip by a flush, the retry limit or a reconfiguration
    uint32_t tx_limit_hits; // the chip gave up after tx_attempts
    uint32_t spi_crc_errors; // reads that failed the crc check on every retry
    uint32_t ecc_errors;
    uint32_t bus_offs;      // bus on to bus off transitions
    uint32_t reconfigs;     // chip was not found or had left normal mode, and was configured anew
    uint32_t data_tx_errors; // CAN FD data phase
    uint32_t data_rx_errors;
    uint8_t tec;
    uint8_t rec;
    uint8_t opmode;         // CiCON.OPMOD
    uint32_t iocon;         // XSTBYEN = bit 6, GPIO0 = bit 16 is the XSTBY pin level
} mcp251xfd_stats_t;


typedef enum {
    MCP251XFD_STOPPED = 0,
    MCP251XFD_RUNNING,
    MCP251XFD_BUS_OFF,      // the chip recovers by itself
    MCP251XFD_NOT_FOUND,    // retried by mcp251xfd_service() every second
    MCP251XFD_SLEEP,        // low power mode, see mcp251xfd_config_t.standby
} mcp251xfd_state_t;


// Fields are internal, except stats which may be read at any time
typedef struct {
    mcp251xfd_config_t cfg;
    mcp251xfd_stats_t stats;
    mcp251xfd_filter_t filters[MCP251XFD_FILTERS_MAX];
    uint8_t filter_num;
    uint8_t state;
    uint8_t opmode;         // the normal mode the chip is put into
    bool opmode_reached;
    bool bus_off;
    bool tx_stall_seen;
    uint8_t payload_size;
    uint8_t obj_size;
    uint8_t tx_ci;          // tx fifo index of the next frame to send, as last read
    uint8_t tx_pending;     // frames in the chip tx fifo, as of the last status read
    uint32_t check_tlast_us;
    uint8_t irq_slot;       // 0 = polled, else slot + 1
} mcp251xfd_t;


// Returns false if the config is invalid or more than MCP251XFD_IRQ_DEVS_MAX use irq_callback
// (state stays STOPPED), or no chip is found (state NOT_FOUND).
// Takes about 5 ms with a chip, 25 ms without. Call mcp251xfd_end() before calling it again
bool mcp251xfd_begin(mcp251xfd_t *dev, const mcp251xfd_config_t *cfg);

// Resets the chip, which puts the transceiver into standby, and releases the pins and SPI
void mcp251xfd_end(mcp251xfd_t *dev);

// Up to MCP251XFD_FILTERS_MAX, 0 rejects all frames
void mcp251xfd_set_filters(mcp251xfd_t *dev, const mcp251xfd_filter_t *filters, uint8_t num);

// Call often: reconfigures the chip if it left normal mode, reads the bus health every 100 ms,
// and tracks sent frames while any are pending
void mcp251xfd_service(mcp251xfd_t *dev);

// INT pin level, no SPI access
bool mcp251xfd_rx_pending(const mcp251xfd_t *dev);

// Returns false if there is no frame, which also rearms the irq_callback
bool mcp251xfd_receive(mcp251xfd_t *dev, mcp251xfd_frame_t *frame);

// Returns 1 if queued, 0 if the chip tx fifo is full or the chip is not in normal mode yet,
// -1 if the frame is not valid for the mode
int mcp251xfd_transmit(mcp251xfd_t *dev, const mcp251xfd_frame_t *frame);

// Aborts the frame in flight and drops all pending ones
void mcp251xfd_flush_tx(mcp251xfd_t *dev);

// Frames in the chip tx fifo, as of the last mcp251xfd_service() or mcp251xfd_transmit()
uint8_t mcp251xfd_tx_pending(const mcp251xfd_t *dev);

mcp251xfd_state_t mcp251xfd_state(const mcp251xfd_t *dev);

#ifdef __cplusplus
}
#endif
