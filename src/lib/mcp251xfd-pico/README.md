# mcp251xfd-pico

Driver for the Microchip MCP2518FD / MCP251863 CAN FD controller on SPI, for the RP2040 and RP2350 with the pico SDK (Arduino-Pico or plain SDK).

- Extended (29 bit) data frames only, CAN 2.0 or mixed CAN FD mode
- Polled or interrupt driven: the chip's INT pin level tells if there are frames to read, so there is no SPI traffic when idle; an optional callback is called from the INT pin interrupt
- Not thread safe: make every call from one core. Cross-core buffering is left to the caller
- 1 Mbit/s nominal, 4 Mbit/s data by default, from a 40 MHz chip clock (crystal or a PWM output); raw bit timing registers can be passed instead

## Errata and recovery handled

From the MCP251863 errata (DS80000984B):

- All reads use the CRC read command, a plain read can return wrong data for CiCON, CiFIFOSTAm and others
- IOCON is written one byte at a time, a multi-byte write clears LAT
- ECC single error correction is not trusted: a received frame with any ECC error in it is dropped; ECC interrupt enables are set, as the flags need them
- A tx fifo that stopped after the retry limit is reset

Every 100 ms the chip is reconfigured if it has left normal mode (system error, brown out); if it was not found it is retried every second. The bus health (TEC, REC, bus off, CAN FD data phase errors) is read at the same time.

## Low power

With `standby` set, `mcp251xfd_begin()` only resets the chip and puts it into low power mode (about 4 uA). IOCON stays at its POR value, so the STBY pull-up keeps the transceiver in standby. There is no SPI access after that, as nCS would wake the chip.

## Wiring

| Chip pin | Connect to | Notes |
|---|---|---|
| SCK, SDI, SDO, nCS | `sck_pin`, `mosi_pin`, `miso_pin`, `cs_pin` | SPI mode 0, SDO gets the internal pull-up |
| INT | `int_pin` | The main INT, not INT0/INT1. Low while a rx fifo is not empty, gets the internal pull-up |
| INT0/GPIO0/XSTBY | transceiver STBY, with a pull-up | Optional, see below |
| INT1 | not connected | Not used |
| OSC1 | 40 MHz crystal or clock | e.g. a PWM output of the RP |

SPI clock is at most 17 MHz with a 40 MHz chip clock; 16 MHz is recommended for margin. The RP's SPI divider rounds down, so asking for 16 MHz gives 15 MHz with a 150 MHz system clock (RP2350 default).

### Transceiver standby

The driver always sets INT0 as XSTBY: it drives the pin low while the chip runs, and the chip's reset puts it back to an input. With a pull-up on the transceiver's STBY pin, the transceiver is then in standby whenever the chip is in reset, not found, in low power mode (`standby`) or after `mcp251xfd_end()`, and it never drives the bus before the chip is configured.

This is optional. STBY can be tied low instead (or to an RP pin the application drives), and INT0 left unconnected: the driver needs no setting for it, as driving an unconnected INT0 is harmless. The cost is that the transceiver stays powered all the time, so the low power mode no longer reaches the ~4 uA it would otherwise, and the transceiver stays active while the controller is in reset. INT0 must not be wired to anything else, as the driver drives it.

## Use

```c
mcp251xfd_t can;
mcp251xfd_config_t cfg = { .spi = spi0, .sck_pin = 2, .mosi_pin = 3, .miso_pin = 4, .cs_pin = 5,
                           .int_pin = 1, .spi_hz = 16000000, .tx_attempts = MCP251XFD_TX_ATTEMPTS_UNLIMITED };
mcp251xfd_begin(&can, &cfg);

// loop
mcp251xfd_service(&can);
while (mcp251xfd_receive(&can, &frame)) { ... }
mcp251xfd_transmit(&can, &frame);   // 1 queued, 0 chip fifo full, -1 invalid
```

## Interrupt

Set `irq_callback` (and `irq_ctx`) to be called from the INT pin interrupt when frames are pending, e.g. to wake a task. The callback runs on the core that called `mcp251xfd_begin()` and must not call the driver: no SPI is done in interrupt context. The interrupt is level triggered and disabled when it fires; `mcp251xfd_receive()` rearms it when it returns false, so drain with it until then. Up to `MCP251XFD_IRQ_DEVS_MAX` (2) instances can use it. It is not used with `standby`.

```c
static volatile bool can_rx;
static void on_can_irq(void *ctx) { can_rx = true; }
cfg.irq_callback = on_can_irq;

// loop
if (can_rx) { can_rx = false; while (mcp251xfd_receive(&can, &frame)) { ... } }
```

`mcp251xfd_service()` must still be called regularly. See `examples/interrupt`.

`stats` in `mcp251xfd_t` holds cumulative counters; `stats.tx_completed` together with `mcp251xfd_tx_pending()` and `mcp251xfd_flush_tx()` lets the caller detect and drop frames nobody acknowledges. See `examples/basic`.

## Examples

`tools/build_example.sh` builds the examples for the RP2350 as a compile check.
