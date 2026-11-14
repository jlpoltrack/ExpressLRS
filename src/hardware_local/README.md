# Local hardware targets

Products not yet in the [targets repo](https://github.com/ExpressLRS/targets), laid out like `hardware/`
(`targets.json`, `RX/`, `TX/`). `UnifiedConfiguration.py` merges `targets.json` over the upstream one, so
these are selected the same way, e.g. `ELRS_UNIFIED_CONFIG=diy.rx_dual.rp2350-lr2021f33`, or from the
interactive menu. To upstream a product, move its entry and layout file into the targets repo unchanged.

## DIY RP2350 NiceRF LR2021F33 RX

- CAN transceiver for MAVLink DroneCAN: `can_rx` GPIO26, `can_tx` GPIO27.
- `radio_dio1` is GPIO6, wired to the LR2021's DIO9 (the driver's IRQ DIO).
- `radio_tcxo` 7 is 3.3 V; `radio_tcxo_delay` 64000 is 2 ms in 32 MHz steps (the LR2021 does not use the LR1121's 30.52 us RTC steps).
- `radio_rfsw_ctrl` is DIO5-11; bits 4..0 = TX HF, RX HF, TX LF, RX LF, standby: DIO5 RX HF, DIO6 TX LF,
  DIO7 RX+TX HF, DIO8 TX HF, DIO9 IRQ.
- `power_values` (sub-GHz) and `power_values_dual` (2.4 GHz) are LR2021 register units (0.5 dB) for
  10, 25, 50, 100, 250, 500 and 1000 mW, from mLRS's power-meter calibration of this module (915 MHz for sub-GHz).
  2.4 GHz at 10 mW runs at about 12 dBm, the module's minimum.

## RP CAN / RP CANFD RX

**Not yet validated on hardware**: both assemblies (can2040 and MCP251863) still need testing once boards are built.

[RP-mLRS-DUAL](https://github.com/jlpoltrack/mLRS-Receivers/tree/main/RP-mLRS) (RP2354A, same NiceRF LR2021F33 as above, so the RF settings are copied unchanged). One PCB, two assemblies:

- `rp-mlrs-dual-can` (`Unified_RP2350_LR2021_RX`): SN65HVD230 with can2040, `can_rx` GPIO1, `can_tx` GPIO4.
- `rp-mlrs-dual-canfd` (`Unified_RP2350_LR2021_CANFD_RX`): MCP251863 on SPI0, `can_sck`/`can_mosi`/`can_miso`/`can_cs` GPIO2/3/4/5, `can_int` GPIO1, own 40 MHz crystal.
- Radio on SPI1: `radio_sck` 14, `radio_mosi` 11, `radio_miso` 8, `radio_nss` 13, `radio_busy` 12, `radio_dio1` 6, `radio_rst` 7.
- The main serial is on the TX/RX pads (GPIO20/21, UART1), `serial1` is TX only on the TX1 pad (GPIO28, UART0).
- LEDs: green GPIO19 is the status `led`, active high; red GPIO29 is unused (ELRS receivers drive one status LED). No button.
- The core runs from an external 1.1 V LDO; the internal regulator is left on (its output is not connected), which only costs idle current.
