//*******************************************************
// MCP2518FD/MCP251863 CAN FD controller on SPI, for the RP2040/RP2350 with the pico SDK
//*******************************************************
// refs (microchip.com): MCP251863 data sheet DS20006624B, MCP25xxFD
// family reference manual DS20005678E, MCP251863 errata DS80000984B
//*******************************************************

#include "mcp251xfd.h"

#include <string.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/timer.h>


//-------------------------------------------------------
// MCP251xFD registers
//-------------------------------------------------------

#define MCP_CMD_RESET             0x00
#define MCP_CMD_WRITE             0x20
#define MCP_CMD_READ_CRC          0xB0

#define MCP_REG_CON               0x000
#define MCP_REG_NBTCFG            0x004
#define MCP_REG_DBTCFG            0x008
#define MCP_REG_TDC               0x00C
#define MCP_REG_INT               0x01C
#define MCP_REG_TREC              0x034 // is followed by BDIAG0
#define MCP_REG_BDIAG0            0x038
#define MCP_REG_FIFOCON(n)        (0x050 + 12 * (n)) // n = 1...31
#define MCP_REG_FIFOSTA(n)        (0x054 + 12 * (n)) // is followed by FIFOUA
#define MCP_REG_FLTCON(n)         (0x1D0 + (n))
#define MCP_REG_FLTOBJ(n)         (0x1F0 + 8 * (n))
#define MCP_REG_MASK(n)           (0x1F4 + 8 * (n))
#define MCP_REG_OSC               0xE00
#define MCP_REG_IOCON             0xE04
#define MCP_REG_ECCCON            0xE0C
#define MCP_REG_ECCSTAT           0xE10

#define MCP_RAM_ADDR              0x400
#define MCP_RAM_SIZE              2048

#define MCP_CON_ISOCRCEN          0x00000020
#define MCP_CON_PXEDIS            0x00000040
#define MCP_CON_RTXAT             0x00010000 // FIFOCON.TXAT is used
#define MCP_CON_OPMOD_SHIFT       21
#define MCP_CON_REQOP_SHIFT       24
#define MCP_CON_TXBWS_2BITS       0x10000000 // pause between own frames, as TransmitPause on STM32

#define MCP_OPMODE_NORMAL_FD      0 // mixed, receives both classic CAN and CAN FD frames
#define MCP_OPMODE_SLEEP          1
#define MCP_OPMODE_CONFIG         4
#define MCP_OPMODE_NORMAL_CAN20   6

// bit timings for 40 MHz, prescaler 1
// nominal 90% sample point as on STM32/ArduPilot, data 80% (STM32 has 75%, not possible with 10 tq)
#define MCP_NBTCFG_1MBPS          0x00220303 // 1 + 35 + 4 tq, sjw 4
#define MCP_DBTCFG_4MBPS          0x00060101 // 1 + 7 + 2 tq, sjw 2, same data bit rate as on STM32G4
#define MCP_TDC_4MBPS             0x00020700 // auto, TDCO = 7

#define MCP_INT_RXIE              0x00020000

#define MCP_TREC_TXBO             0x00200000

#define MCP_OSC_OSCRDY            0x00000400
#define MCP_OSC_LPMEN             0x08

#define MCP_IOCON_XSTBYEN         0x40
#define MCP_IOCON_TRIS1           0x02 // TRIS0 = 0: XSTBY only drives the pin when it is an output

#define MCP_ECCCON_ECCEN          0x01
#define MCP_ECCCON_SECIE          0x02
#define MCP_ECCCON_DEDIE          0x04

#define MCP_ECCSTAT_SECIF         0x02
#define MCP_ECCSTAT_DEDIF         0x04
#define MCP_ECCSTAT_ERRADDR_SHIFT 16

#define MCP_FIFOCON_TFNRFNIE      0x00000001
#define MCP_FIFOCON_TXEN          0x00000080
#define MCP_FIFOCON_UINC          0x00000100
#define MCP_FIFOCON_TXREQ         0x00000200
#define MCP_FIFOCON_FRESET        0x00000400
#define MCP_FIFOCON_TXAT_SHIFT    21
#define MCP_FIFOCON_FSIZE_SHIFT   24
#define MCP_FIFOCON_PLSIZE_8      0x00000000
#define MCP_FIFOCON_PLSIZE_64     0xE0000000

#define MCP_FIFOSTA_TFNRFNIF      0x01 // tx fifo not full, rx fifo not empty
#define MCP_FIFOSTA_TFERFFIF      0x04 // tx fifo empty, rx fifo full
#define MCP_FIFOSTA_RXOVIF        0x08
#define MCP_FIFOSTA_TXATIF        0x10
#define MCP_FIFOSTA_FIFOCI_SHIFT  8
#define MCP_FIFOSTA_FIFOCI_MASK   0x1F

#define MCP_FLTCON_FLTEN          0x80
#define MCP_FLTOBJ_EXIDE          0x40000000 // match only extended ids
#define MCP_MASK_MIDE             0x40000000

#define MCP_OBJ_SID_MASK          0x000007FF
#define MCP_OBJ_EID_MASK          0x0003FFFF
#define MCP_OBJ_EID_SHIFT         11
#define MCP_OBJ_DLC_MASK          0x0000000F
#define MCP_OBJ_IDE               0x00000010
#define MCP_OBJ_RTR               0x00000020
#define MCP_OBJ_BRS               0x00000040
#define MCP_OBJ_FDF               0x00000080

#define MCP_OBJ_HEADER_SIZE       8
#define MCP_EXT_ID_MASK           0x1FFFFFFF

// the two rx fifos follow the tx fifo in ram
#define MCP_TX_FIFO               1
#define MCP_RX_FIFO0              2
#define MCP_RX_FIFO1              3

#define MCP_CHECK_INTERVAL_US     100000
#define MCP_RETRY_INTERVAL_US     1000000 // no chip found, a configure attempt blocks for 25 ms


static const uint8_t dlc_to_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static uint8_t dlc_from_data_len(uint8_t data_len)
{
    for (uint8_t dlc = 0; dlc < 16; dlc++) {
        if (dlc_to_len[dlc] >= data_len) return dlc;
    }
    return 15;
}

// 29 bit id to the SID, EID layout of the chip's message and filter objects
static uint32_t mcp_obj_from_id(uint32_t id)
{
    id &= MCP_EXT_ID_MASK;
    return (id >> 18) | ((id & MCP_OBJ_EID_MASK) << MCP_OBJ_EID_SHIFT);
}


//-------------------------------------------------------
// spi access
//-------------------------------------------------------

static void mcp_write(mcp251xfd_t *dev, uint16_t addr, const void *data, uint16_t len)
{
    uint8_t cmd[2] = { (uint8_t)(MCP_CMD_WRITE | (addr >> 8)), (uint8_t)addr };
    gpio_put(dev->cfg.cs_pin, 0);
    spi_write_blocking(dev->cfg.spi, cmd, 2);
    spi_write_blocking(dev->cfg.spi, (const uint8_t *)data, len);
    gpio_put(dev->cfg.cs_pin, 1);
}

// poly 0x8005, msb first, the chip starts with 0xFFFF on nCS low
static uint16_t mcp_crc16_table[256];
static bool mcp_crc16_ready = false;

static void mcp_crc16_init(void)
{
    if (mcp_crc16_ready) return;
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint8_t n = 0; n < 8; n++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
        mcp_crc16_table[i] = crc;
    }
    mcp_crc16_ready = true;
}

static uint16_t mcp_crc16(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--) crc = (crc << 8) ^ mcp_crc16_table[(crc >> 8) ^ *data++];
    return crc;
}

// all reads are crc checked, errata: a plain read can return wrong data for CiCON, CiFIFOSTAm
// and others. len must be a multiple of 4 for ram
// returns false if the crc does not match even with retries
static bool mcp_read(mcp251xfd_t *dev, uint16_t addr, void *data, uint8_t len)
{
    // the length field is in bytes for registers, and in words for ram
    uint8_t n = (addr >= MCP_RAM_ADDR && addr < MCP_RAM_ADDR + MCP_RAM_SIZE) ? len / 4 : len;
    uint8_t cmd[3] = { (uint8_t)(MCP_CMD_READ_CRC | (addr >> 8)), (uint8_t)addr, n };
    for (uint8_t retry = 0; retry < 3; retry++) {
        uint8_t crc[2];
        gpio_put(dev->cfg.cs_pin, 0);
        spi_write_blocking(dev->cfg.spi, cmd, 3);
        spi_read_blocking(dev->cfg.spi, 0, (uint8_t *)data, len);
        spi_read_blocking(dev->cfg.spi, 0, crc, 2);
        gpio_put(dev->cfg.cs_pin, 1);
        uint16_t crc16 = mcp_crc16(mcp_crc16(0xFFFF, cmd, 3), (uint8_t *)data, len);
        if (crc16 == (((uint16_t)crc[0] << 8) | crc[1])) return true;
    }
    dev->stats.spi_crc_errors++;
    return false;
}

// registers are little endian, as is the rp
// reads as all ones on a crc failure, which no caller takes for a valid value
static uint32_t mcp_read32(mcp251xfd_t *dev, uint16_t addr)
{
    uint32_t value;
    if (!mcp_read(dev, addr, &value, 4)) return UINT32_MAX;
    return value;
}

static void mcp_write32(mcp251xfd_t *dev, uint16_t addr, uint32_t value)
{
    mcp_write(dev, addr, &value, 4);
}

static void mcp_write8(mcp251xfd_t *dev, uint16_t addr, uint8_t value)
{
    mcp_write(dev, addr, &value, 1);
}

static uint8_t mcp_opmode(mcp251xfd_t *dev)
{
    return (mcp_read32(dev, MCP_REG_CON) >> MCP_CON_OPMOD_SHIFT) & 0x07;
}

// the chip has to be in configuration mode to be reset
static void mcp_reset(mcp251xfd_t *dev)
{
    mcp_write8(dev, MCP_REG_CON + 3, MCP_OPMODE_CONFIG);
    uint32_t tstart_us = time_us_32();
    while (mcp_opmode(dev) != MCP_OPMODE_CONFIG) {
        if ((time_us_32() - tstart_us) > 2000) break;
    }

    uint8_t cmd[2] = { MCP_CMD_RESET, 0 };
    gpio_put(dev->cfg.cs_pin, 0);
    spi_write_blocking(dev->cfg.spi, cmd, 2);
    gpio_put(dev->cfg.cs_pin, 1);
}


//-------------------------------------------------------
// chip configuration
//-------------------------------------------------------

// only extended ids, frames matching no filter are rejected
static void mcp_apply_filters(mcp251xfd_t *dev)
{
    for (uint8_t n = 0; n < MCP251XFD_FILTERS_MAX; n++) {
        mcp_write8(dev, MCP_REG_FLTCON(n), 0); // a filter must be disabled to be changed
        if (n >= dev->filter_num) continue;

        uint8_t fifo = (dev->filters[n].fifo == 1) ? MCP_RX_FIFO1 : MCP_RX_FIFO0;
        mcp_write32(dev, MCP_REG_FLTOBJ(n), mcp_obj_from_id(dev->filters[n].id) | MCP_FLTOBJ_EXIDE);
        mcp_write32(dev, MCP_REG_MASK(n), mcp_obj_from_id(dev->filters[n].mask) | MCP_MASK_MIDE);
        mcp_write8(dev, MCP_REG_FLTCON(n), MCP_FLTCON_FLTEN | fifo);
    }
}

// frames in the chip tx fifo are lost
static void mcp_tx_forget(mcp251xfd_t *dev)
{
    dev->stats.tx_aborted += dev->tx_pending;
    dev->tx_pending = 0;
    dev->tx_ci = 0;
    dev->tx_stall_seen = false;
}

// returns false if no chip is found
static bool mcp_configure(mcp251xfd_t *dev)
{
    const mcp251xfd_config_t *cfg = &dev->cfg;

    dev->check_tlast_us = time_us_32();
    mcp_tx_forget(dev);
    dev->bus_off = false;

    mcp_reset(dev);

    // wait for the oscillator, takes about 3 ms. a floating miso reads as 0 or 0xFF
    uint32_t tstart_us = time_us_32();
    while (1) {
        uint32_t osc = mcp_read32(dev, MCP_REG_OSC);
        if ((osc & MCP_OSC_OSCRDY) && osc != UINT32_MAX) break;
        if ((time_us_32() - tstart_us) > 20000) return false;
    }
    if (mcp_opmode(dev) != MCP_OPMODE_CONFIG) return false;
    if (cfg->standby) { // IOCON is at POR, the STBY pull-up keeps the transceiver in standby
        // low power mode, ~4 uA. nCS wakes it like a POR, so no SPI access after this
        mcp_write8(dev, MCP_REG_OSC, MCP_OSC_LPMEN);
        mcp_write8(dev, MCP_REG_CON + 3, MCP_OPMODE_SLEEP);
        dev->stats.opmode = MCP_OPMODE_SLEEP;
        return true;
    }

    // INT0 pin drives transceiver STBY, low while awake. single-byte write, errata: multi-byte clears LAT
    mcp_write8(dev, MCP_REG_IOCON, MCP_IOCON_XSTBYEN | MCP_IOCON_TRIS1);

    // ecc needs the ram to be initialized
    uint8_t zeros[64] = {0};
    for (uint16_t n = 0; n < MCP_RAM_SIZE; n += sizeof(zeros)) {
        mcp_write(dev, MCP_RAM_ADDR + n, zeros, sizeof(zeros));
    }
    mcp_write8(dev, MCP_REG_ECCCON, MCP_ECCCON_DEDIE | MCP_ECCCON_SECIE | MCP_ECCCON_ECCEN); // errata: ie needed for the flags

    // no tx queue, no tx event fifo, iso crc, retransmissions set per fifo
    mcp_write32(dev, MCP_REG_CON, MCP_CON_TXBWS_2BITS |
        ((uint32_t)MCP_OPMODE_CONFIG << MCP_CON_REQOP_SHIFT) | MCP_CON_RTXAT | MCP_CON_PXEDIS | MCP_CON_ISOCRCEN);

    mcp_write32(dev, MCP_REG_NBTCFG, (cfg->nbtcfg) ? cfg->nbtcfg : MCP_NBTCFG_1MBPS);
    mcp_write32(dev, MCP_REG_DBTCFG, (cfg->dbtcfg) ? cfg->dbtcfg : MCP_DBTCFG_4MBPS);
    mcp_write32(dev, MCP_REG_TDC, (cfg->tdc) ? cfg->tdc : MCP_TDC_4MBPS);

    uint32_t plsize = (cfg->canfd) ? MCP_FIFOCON_PLSIZE_64 : MCP_FIFOCON_PLSIZE_8;
    mcp_write32(dev, MCP_REG_FIFOCON(MCP_TX_FIFO),
        plsize | ((uint32_t)(cfg->tx_fifo_size - 1) << MCP_FIFOCON_FSIZE_SHIFT) |
        ((uint32_t)cfg->tx_attempts << MCP_FIFOCON_TXAT_SHIFT) | MCP_FIFOCON_TXEN);

    // INT pin is low as long as a rx fifo is not empty
    uint32_t fifocon = plsize | ((uint32_t)(cfg->rx_fifo_size - 1) << MCP_FIFOCON_FSIZE_SHIFT) | MCP_FIFOCON_TFNRFNIE;
    mcp_write32(dev, MCP_REG_FIFOCON(MCP_RX_FIFO0), fifocon);
    mcp_write32(dev, MCP_REG_FIFOCON(MCP_RX_FIFO1), fifocon);
    mcp_write32(dev, MCP_REG_INT, MCP_INT_RXIE);

    mcp_apply_filters(dev);

    // the chip enters the mode by itself as soon as it sees the bus idle
    dev->opmode = (cfg->canfd) ? MCP_OPMODE_NORMAL_FD : MCP_OPMODE_NORMAL_CAN20;
    dev->opmode_reached = false;
    mcp_write8(dev, MCP_REG_CON + 3, (MCP_CON_TXBWS_2BITS >> 24) | dev->opmode);

    return true;
}

// level triggered: fires again at once if frames are still pending, e.g. after a crc failure.
// only while running, a missing chip may hold INT low, armed again once it is configured
static void mcp_irq_arm(mcp251xfd_t *dev)
{
    if (dev->irq_slot && dev->state == MCP251XFD_RUNNING) gpio_set_irq_enabled(dev->cfg.int_pin, GPIO_IRQ_LEVEL_LOW, true);
}

static void mcp_reconfigure(mcp251xfd_t *dev)
{
    dev->stats.reconfigs++;
    dev->state = (mcp_configure(dev)) ? MCP251XFD_RUNNING : MCP251XFD_NOT_FOUND;
    mcp_irq_arm(dev);
}


//-------------------------------------------------------
// rx
//-------------------------------------------------------

// returns true if a frame was read out
static bool mcp_receive_obj(mcp251xfd_t *dev, uint8_t fifo, uint32_t ua, mcp251xfd_frame_t *frame)
{
    union {
        uint32_t r[2];
        uint8_t buf[MCP_OBJ_HEADER_SIZE + MCP251XFD_DATA_LEN_MAX];
    } obj;

    // read header and the first 8 bytes, which is all there is for classic frames
    uint16_t addr = MCP_RAM_ADDR + (ua & 0x0FFF);
    if (!mcp_read(dev, addr, obj.buf, MCP_OBJ_HEADER_SIZE + 8)) return false; // frame stays in the fifo

    bool fdf = (obj.r[1] & MCP_OBJ_FDF) != 0;
    uint8_t data_len = dlc_to_len[obj.r[1] & MCP_OBJ_DLC_MASK];
    if (data_len > dev->payload_size) data_len = dev->payload_size;
    // drop remote frames and classic frames with DLC > 8, the chip's filters cannot match on RTR
    bool drop = !fdf && ((obj.r[1] & MCP_OBJ_RTR) || dlc_to_len[obj.r[1] & MCP_OBJ_DLC_MASK] > 8);
    if (!drop && data_len > 8) {
        if (!mcp_read(dev, addr + MCP_OBJ_HEADER_SIZE + 8, obj.buf + MCP_OBJ_HEADER_SIZE + 8, data_len - 8)) return false;
    }

    // errata: ecc single error correction does not work in all cases, so drop the frame on any ecc
    // error within it. ERRADDR is the absolute address of the last error
    uint32_t eccstat = mcp_read32(dev, MCP_REG_ECCSTAT);
    if (eccstat != UINT32_MAX && (eccstat & (MCP_ECCSTAT_DEDIF | MCP_ECCSTAT_SECIF))) {
        mcp_write8(dev, MCP_REG_ECCSTAT, 0); // clears the flags
        dev->stats.ecc_errors++;
        uint16_t erraddr = (eccstat >> MCP_ECCSTAT_ERRADDR_SHIFT) & 0x0FFF;
        if (erraddr >= addr && erraddr < addr + dev->obj_size) drop = true;
    }

    mcp_write8(dev, MCP_REG_FIFOCON(fifo) + 1, MCP_FIFOCON_UINC >> 8);
    if (drop) {
        dev->stats.rx_dropped++;
        return false;
    }

    // filters only pass extended ids
    frame->id = ((obj.r[0] & MCP_OBJ_SID_MASK) << 18) | ((obj.r[0] >> MCP_OBJ_EID_SHIFT) & MCP_OBJ_EID_MASK);
    frame->len = data_len;
    frame->fdf = fdf;
    memcpy(frame->data, obj.buf + MCP_OBJ_HEADER_SIZE, data_len);
    memset(frame->data + data_len, 0, MCP251XFD_DATA_LEN_MAX - data_len);
    dev->stats.rx_frames++;
    return true;
}

static bool mcp_receive(mcp251xfd_t *dev, mcp251xfd_frame_t *frame)
{
    if (dev->state != MCP251XFD_RUNNING) return false;

    // bounded, dropped frames are skipped
    for (uint8_t n = 0; n < 2 * dev->cfg.rx_fifo_size; n++) {
        if (gpio_get(dev->cfg.int_pin)) return false; // high, both rx fifos are empty

        // FIFOSTA, FIFOUA of rx fifo 0, FIFOCON, FIFOSTA, FIFOUA of rx fifo 1
        uint32_t regs[5];
        if (!mcp_read(dev, MCP_REG_FIFOSTA(MCP_RX_FIFO0), regs, sizeof(regs))) return false;
        if (regs[0] & MCP_FIFOSTA_RXOVIF) {
            dev->stats.rx_overflows++;
            mcp_write8(dev, MCP_REG_FIFOSTA(MCP_RX_FIFO0), 0); // clears the flag
        }
        if (regs[3] & MCP_FIFOSTA_RXOVIF) {
            dev->stats.rx_overflows++;
            mcp_write8(dev, MCP_REG_FIFOSTA(MCP_RX_FIFO1), 0);
        }

        bool ok;
        if (regs[0] & MCP_FIFOSTA_TFNRFNIF) {
            ok = mcp_receive_obj(dev, MCP_RX_FIFO0, regs[1], frame);
        } else
        if (regs[3] & MCP_FIFOSTA_TFNRFNIF) {
            ok = mcp_receive_obj(dev, MCP_RX_FIFO1, regs[4], frame);
        } else {
            return false;
        }
        if (ok) return true;
    }
    return false;
}

bool mcp251xfd_receive(mcp251xfd_t *dev, mcp251xfd_frame_t *frame)
{
    if (mcp_receive(dev, frame)) return true;
    mcp_irq_arm(dev);
    return false;
}

bool mcp251xfd_rx_pending(const mcp251xfd_t *dev)
{
    return dev->state == MCP251XFD_RUNNING && !gpio_get(dev->cfg.int_pin);
}


//-------------------------------------------------------
// tx
//-------------------------------------------------------

static void mcp_tx_fifo_reset(mcp251xfd_t *dev)
{
    // clearing TXREQ requests an abort of the frame in flight, then the fifo can be reset
    mcp_write8(dev, MCP_REG_FIFOCON(MCP_TX_FIFO) + 1, 0);
    uint32_t tstart_us = time_us_32();
    while (mcp_read32(dev, MCP_REG_FIFOCON(MCP_TX_FIFO)) & MCP_FIFOCON_TXREQ) {
        if ((time_us_32() - tstart_us) > 1000) break;
    }
    mcp_write8(dev, MCP_REG_FIFOCON(MCP_TX_FIFO) + 1, MCP_FIFOCON_FRESET >> 8);
    tstart_us = time_us_32();
    while (mcp_read32(dev, MCP_REG_FIFOCON(MCP_TX_FIFO)) & MCP_FIFOCON_FRESET) {
        if ((time_us_32() - tstart_us) > 1000) break;
    }
    mcp_write8(dev, MCP_REG_FIFOSTA(MCP_TX_FIFO), 0); // clears TXATIF
    mcp_tx_forget(dev);
}

// reads FIFOCON, FIFOSTA, FIFOUA of the tx fifo, counts sent frames and resets a stalled fifo
// returns false if the status could not be read or the fifo was reset
static bool mcp_tx_status(mcp251xfd_t *dev, uint32_t regs[3])
{
    // FIFOUA is not valid before the chip has left configuration mode
    if (!dev->opmode_reached) {
        dev->opmode_reached = (mcp_opmode(dev) == dev->opmode);
        if (!dev->opmode_reached) return false;
    }

    if (!mcp_read(dev, MCP_REG_FIFOCON(MCP_TX_FIFO), regs, 3 * sizeof(uint32_t))) return false;
    uint32_t sta = regs[1];

    // FIFOCI is the next frame to send, the fifo holds at most tx_fifo_size frames, so an empty
    // fifo is the only case where all pending frames went out
    uint8_t ci = (sta >> MCP_FIFOSTA_FIFOCI_SHIFT) & MCP_FIFOSTA_FIFOCI_MASK;
    uint8_t sent;
    if (sta & MCP_FIFOSTA_TFERFFIF) {
        sent = dev->tx_pending;
    } else {
        sent = (uint8_t)(ci + dev->cfg.tx_fifo_size - dev->tx_ci) % dev->cfg.tx_fifo_size;
        if (sent > dev->tx_pending) sent = dev->tx_pending;
    }
    dev->tx_ci = ci;
    dev->tx_pending -= sent;
    dev->stats.tx_completed += sent;

    // the chip gave up on a frame after tx_attempts, and has stopped sending: drop all pending frames.
    // TXREQ clears before FIFOSTA shows the fifo empty, so that combination alone is only
    // believed when it is seen twice in a row, TXATIF is definite
    bool stalled = !(regs[0] & MCP_FIFOCON_TXREQ) && !(sta & MCP_FIFOSTA_TFERFFIF);
    bool confirmed = stalled && dev->tx_stall_seen;
    dev->tx_stall_seen = stalled;
    if (stalled && !confirmed && !(sta & MCP_FIFOSTA_TXATIF)) return false; // look again next time
    if ((sta & MCP_FIFOSTA_TXATIF) || confirmed) {
        dev->stats.tx_limit_hits++;
        mcp_tx_fifo_reset(dev);
        return false;
    }
    return true;
}

int mcp251xfd_transmit(mcp251xfd_t *dev, const mcp251xfd_frame_t *frame)
{
    if (frame->len > ((frame->fdf) ? MCP251XFD_DATA_LEN_MAX : 8)) return -1;
    if (frame->fdf && !dev->cfg.canfd) return -1;
    if (dev->state != MCP251XFD_RUNNING) return 0;

    uint32_t regs[3]; // FIFOCON, FIFOSTA, FIFOUA
    if (!mcp_tx_status(dev, regs)) return 0;
    if (!(regs[1] & MCP_FIFOSTA_TFNRFNIF)) return 0; // chip tx fifo full

    union {
        uint32_t t[2];
        uint8_t buf[MCP_OBJ_HEADER_SIZE + MCP251XFD_DATA_LEN_MAX];
    } obj;
    uint8_t dlc = dlc_from_data_len(frame->len);
    uint8_t len = dlc_to_len[dlc];
    obj.t[0] = mcp_obj_from_id(frame->id);
    obj.t[1] = dlc | MCP_OBJ_IDE;
    if (frame->fdf) obj.t[1] |= MCP_OBJ_FDF | MCP_OBJ_BRS;
    memcpy(obj.buf + MCP_OBJ_HEADER_SIZE, frame->data, frame->len);
    memset(obj.buf + MCP_OBJ_HEADER_SIZE + frame->len, 0, len - frame->len); // padding to the dlc length

    // ram is written in whole words
    mcp_write(dev, MCP_RAM_ADDR + (regs[2] & 0x0FFF), obj.buf, MCP_OBJ_HEADER_SIZE + ((len + 3) & ~3));
    mcp_write8(dev, MCP_REG_FIFOCON(MCP_TX_FIFO) + 1, (MCP_FIFOCON_UINC | MCP_FIFOCON_TXREQ) >> 8);

    dev->tx_pending++;
    dev->stats.tx_frames++;
    return 1;
}

void mcp251xfd_flush_tx(mcp251xfd_t *dev)
{
    if (dev->state != MCP251XFD_RUNNING || !dev->opmode_reached) return;
    mcp_tx_fifo_reset(dev);
}

uint8_t mcp251xfd_tx_pending(const mcp251xfd_t *dev)
{
    return dev->tx_pending;
}


//-------------------------------------------------------
// bus health
//-------------------------------------------------------

// BDIAG0 counters only count up, so they are cleared after each read
static void mcp_read_bus_health(mcp251xfd_t *dev)
{
    uint32_t regs[2]; // TREC, BDIAG0
    if (!mcp_read(dev, MCP_REG_TREC, regs, sizeof(regs))) return;
    mcp_write32(dev, MCP_REG_BDIAG0, 0);
    dev->stats.iocon = mcp_read32(dev, MCP_REG_IOCON);

    bool bus_off = (regs[0] & MCP_TREC_TXBO) != 0;
    if (bus_off && !dev->bus_off) dev->stats.bus_offs++;
    dev->bus_off = bus_off;

    dev->stats.tec = (regs[0] >> 8) & 0xFF;
    dev->stats.rec = regs[0] & 0xFF;
    dev->stats.data_tx_errors += regs[1] >> 24;
    dev->stats.data_rx_errors += (regs[1] >> 16) & 0xFF;
}


//-------------------------------------------------------
// INT pin interrupt
//-------------------------------------------------------

// the raw gpio handlers take no argument, so there is one per slot
static mcp251xfd_t *mcp_irq_devs[MCP251XFD_IRQ_DEVS_MAX];

static void mcp_irq_handle(uint8_t slot)
{
    mcp251xfd_t *dev = mcp_irq_devs[slot];
    if (dev == NULL || !(gpio_get_irq_event_mask(dev->cfg.int_pin) & GPIO_IRQ_LEVEL_LOW)) return;
    gpio_set_irq_enabled(dev->cfg.int_pin, GPIO_IRQ_LEVEL_LOW, false); // rearmed by mcp251xfd_receive()
    dev->cfg.irq_callback(dev->cfg.irq_ctx);
}

static void mcp_irq_handler0(void) { mcp_irq_handle(0); }
static void mcp_irq_handler1(void) { mcp_irq_handle(1); }
static void (*const mcp_irq_handlers[MCP251XFD_IRQ_DEVS_MAX])(void) = { mcp_irq_handler0, mcp_irq_handler1 };

// returns false if all slots are taken
static bool mcp_irq_claim(mcp251xfd_t *dev)
{
    for (uint8_t n = 0; n < MCP251XFD_IRQ_DEVS_MAX; n++) {
        if (mcp_irq_devs[n] != NULL) continue;
        mcp_irq_devs[n] = dev;
        dev->irq_slot = n + 1;
        return true;
    }
    return false;
}

static void mcp_irq_start(mcp251xfd_t *dev)
{
    gpio_add_raw_irq_handler(dev->cfg.int_pin, mcp_irq_handlers[dev->irq_slot - 1]);
    irq_set_enabled(IO_IRQ_BANK0, true); // armed once the chip is configured
}

static void mcp_irq_release(mcp251xfd_t *dev)
{
    if (!dev->irq_slot) return;
    gpio_set_irq_enabled(dev->cfg.int_pin, GPIO_IRQ_LEVEL_LOW, false);
    gpio_remove_raw_irq_handler(dev->cfg.int_pin, mcp_irq_handlers[dev->irq_slot - 1]);
    mcp_irq_devs[dev->irq_slot - 1] = NULL;
    dev->irq_slot = 0;
}


//-------------------------------------------------------
// api
//-------------------------------------------------------

bool mcp251xfd_begin(mcp251xfd_t *dev, const mcp251xfd_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    dev->state = MCP251XFD_STOPPED;
    dev->cfg = *cfg;
    dev->cfg.filters = NULL; // copied below, the caller's array may go away

    if (cfg->tx_attempts != MCP251XFD_TX_ATTEMPTS_ONE && cfg->tx_attempts != MCP251XFD_TX_ATTEMPTS_THREE &&
        cfg->tx_attempts != MCP251XFD_TX_ATTEMPTS_UNLIMITED) return false;
    dev->payload_size = (cfg->canfd) ? MCP251XFD_DATA_LEN_MAX : 8;
    dev->obj_size = MCP_OBJ_HEADER_SIZE + dev->payload_size;
    if (dev->cfg.tx_fifo_size == 0) dev->cfg.tx_fifo_size = (cfg->canfd) ? 8 : 32;
    if (dev->cfg.rx_fifo_size == 0) dev->cfg.rx_fifo_size = (cfg->canfd) ? 10 : 32;
    if (dev->cfg.tx_fifo_size > 32 || dev->cfg.rx_fifo_size > 32) return false;
    if ((dev->cfg.tx_fifo_size + 2 * dev->cfg.rx_fifo_size) * dev->obj_size > MCP_RAM_SIZE) return false;

    if (cfg->filters != NULL && cfg->filter_num > 0) {
        if (cfg->filter_num > MCP251XFD_FILTERS_MAX) return false;
        memcpy(dev->filters, cfg->filters, cfg->filter_num * sizeof(cfg->filters[0]));
        dev->filter_num = cfg->filter_num;
    } else {
        dev->filter_num = 1; // id 0, mask 0: accept all into fifo 0
    }
    if (cfg->irq_callback != NULL && !cfg->standby && !mcp_irq_claim(dev)) return false;

    mcp_crc16_init();

    gpio_init(cfg->cs_pin);
    gpio_put(cfg->cs_pin, 1);
    gpio_set_dir(cfg->cs_pin, GPIO_OUT);

    spi_init(cfg->spi, cfg->spi_hz); // 8 bits, mode 0, msb first
    gpio_set_function(cfg->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(cfg->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(cfg->miso_pin, GPIO_FUNC_SPI);
    gpio_pull_up(cfg->miso_pin); // SDO floats while nCS is high, would raise the sleep current

    gpio_init(cfg->int_pin); // input
    gpio_pull_up(cfg->int_pin);
    if (dev->irq_slot) mcp_irq_start(dev);

    bool ok = mcp_configure(dev);
    if (cfg->standby) {
        dev->state = (ok) ? MCP251XFD_SLEEP : MCP251XFD_NOT_FOUND;
    } else {
        dev->state = (ok) ? MCP251XFD_RUNNING : MCP251XFD_NOT_FOUND;
        mcp_irq_arm(dev);
    }
    return ok;
}

void mcp251xfd_end(mcp251xfd_t *dev)
{
    if (dev->state == MCP251XFD_STOPPED) return;
    mcp_irq_release(dev);

    // in low power mode nCS would wake the chip, a reset puts IOCON back to POR otherwise
    if (dev->state != MCP251XFD_SLEEP) {
        mcp_tx_forget(dev);
        mcp_reset(dev);
    }

    spi_deinit(dev->cfg.spi);
    gpio_deinit(dev->cfg.sck_pin);
    gpio_deinit(dev->cfg.mosi_pin);
    gpio_deinit(dev->cfg.miso_pin);
    gpio_deinit(dev->cfg.int_pin);
    gpio_deinit(dev->cfg.cs_pin);
    gpio_pull_up(dev->cfg.cs_pin); // keeps the chip deselected
    dev->state = MCP251XFD_STOPPED;
}

void mcp251xfd_set_filters(mcp251xfd_t *dev, const mcp251xfd_filter_t *filters, uint8_t num)
{
    if (num > MCP251XFD_FILTERS_MAX) num = MCP251XFD_FILTERS_MAX;
    if (num > 0) memcpy(dev->filters, filters, num * sizeof(filters[0]));
    dev->filter_num = num;
    if (dev->state == MCP251XFD_RUNNING) mcp_apply_filters(dev); // else applied once configured
}

void mcp251xfd_service(mcp251xfd_t *dev)
{
    if (dev->state == MCP251XFD_STOPPED || dev->state == MCP251XFD_SLEEP) return;

    // configure the chip anew if it was not found or has left normal mode. it leaves it on a
    // system error, e.g. a ram ecc error, and then ignores all tx requests, and a brown out resets it
    uint32_t tnow_us = time_us_32();
    uint32_t interval_us = (dev->state == MCP251XFD_NOT_FOUND) ? MCP_RETRY_INTERVAL_US : MCP_CHECK_INTERVAL_US;
    if ((tnow_us - dev->check_tlast_us) > interval_us) {
        dev->check_tlast_us = tnow_us;
        if (dev->state == MCP251XFD_NOT_FOUND) {
            if (dev->cfg.standby) {
                dev->state = (mcp_configure(dev)) ? MCP251XFD_SLEEP : MCP251XFD_NOT_FOUND;
                return;
            }
            mcp_reconfigure(dev);
        } else {
            dev->stats.opmode = mcp_opmode(dev);
            if (dev->stats.opmode != dev->opmode) {
                mcp_reconfigure(dev);
            } else {
                mcp_read_bus_health(dev); // TXBO is also set in configuration mode, so only in normal mode
            }
        }
    }
    if (dev->state != MCP251XFD_RUNNING) return;

    if (dev->tx_pending > 0) {
        uint32_t regs[3];
        mcp_tx_status(dev, regs);
    }
}

mcp251xfd_state_t mcp251xfd_state(const mcp251xfd_t *dev)
{
    if (dev->state == MCP251XFD_RUNNING && dev->bus_off) return MCP251XFD_BUS_OFF;
    return (mcp251xfd_state_t)dev->state;
}
