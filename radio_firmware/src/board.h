// Board wiring: RP2350 <-> LR2021. The only place pin numbers live; the board is chosen per build target (HZ_BOARD):
//   HZ_BOARD_HAT     1  the Horizon HAT: Pico 2 + NiceRF LoRa2021F33 (Pi transmitter, receiver)
//   (2: retired)
//   HZ_BOARD_AIR  3  integrated video TX board: Luckfox Pico, RP2354A and NiceRF LoRa2021F33 on one 51x31 mm
//                         carrier (air_radio)
//   HZ_BOARD_GROUND_UNIT 4  Horizon Dev Ground Unit rev B: RP2354A and Seeed Wio-LR2021 on one USB receiver board
//                         (ground_unit)
#pragma once

#include "hardware/spi.h"

#define HZ_BOARD_HAT    1
#define HZ_BOARD_AIR 3
#define HZ_BOARD_GROUND_UNIT 4
#ifndef HZ_BOARD
#define HZ_BOARD HZ_BOARD_HAT
#endif

#if HZ_BOARD == HZ_BOARD_GROUND_UNIT
#define HZ_BOARD_NAME   "ground-unit"
// Seeed Wio-LR2021 module on SPI0, 470 R series resistors at the module on every radio line (Seeed's design guide).
// Pins from the board (Backplane first-board/boards/lora-node, rev B; same pin map as rev 1).
#define RADIO_SPI       spi0
#define PIN_RADIO_MISO  20  // SPI0 RX  <- module pin 4
#define PIN_RADIO_NSS   17  // plain GPIO chip-select -> module pin 7
#define PIN_RADIO_SCK   18  // SPI0 SCK -> module pin 6
#define PIN_RADIO_MOSI  19  // SPI0 TX  -> module pin 5
#define PIN_RADIO_BUSY  16  // input, internal pull-up (no pull-up on the board: a missing module reads as busy)
#define PIN_RADIO_IRQ   11  // input, no pull (module pin 16 = LR2021 DIO11)
#define PIN_RADIO_RESET 13  // output, active low -> module pin 3
#define PIN_LED         12  // red LED D1, high = on: receiver status (rx.c)
// Module DIO7 (pin 22) -> GPIO1 and DIO8 (pin 19) -> GPIO5 are wired but unused: they stay inputs.
#define RADIO_BUSY_PULLUP 1
#define RADIO_IRQ_DIO   LR20XX_SYSTEM_DIO_11
// The Wio-LR2021's TCXO runs from the LR2021's VTCXO at 3.0 V (with 1.8 V calibration fails: HF_XOSC_START).
#define RADIO_TCXO_CTRL LR20XX_SYSTEM_TCXO_CTRL_3_0V
// No front-end switching: the module's RF pins go straight to its pads (LF pin 10, HF pin 13), no DIO RF switch.
#define RADIO_NO_FRONT_END 1
#elif HZ_BOARD == HZ_BOARD_AIR
#define HZ_BOARD_NAME   "air"  /* firmware before 2026-11-25: "video-tx" */
// Radio on SPI1. The module's CE input enables its LDO: the radio only answers with CE high.
#define RADIO_SPI       spi1
#define PIN_RADIO_MISO  8   // SPI1 RX
#define PIN_RADIO_NSS   9   // plain GPIO chip-select (10k pull-up on the board)
#define PIN_RADIO_SCK   10  // SPI1 SCK
#define PIN_RADIO_MOSI  11  // SPI1 TX
#define PIN_RADIO_BUSY  5   // input, no pull
#define PIN_RADIO_IRQ   7   // input, no pull (module IRQ = LR2021 DIO9)
#define PIN_RADIO_RESET 6   // output, active low
#define PIN_RADIO_CE    12  // output, high = module powered
#define PIN_LED         13  // red LED, high = on: status (tx.c led_tick())
// Host link: the Luckfox Pico's SPI0 master. A PIO state machine is the SPI slave (mode 0, as on the Luckfox
// carrier). GP14 (HOST_READY to Luckfox GPIO1_C6) is not used by this protocol; it stays an input (100k pull-down).
#define PIN_HOST_SCK    2   // <- Luckfox pin 14 SPI0 CLK
#define PIN_HOST_RX     0   // <- Luckfox pin 15 SPI0 MOSI (video records into the RP2350)
#define PIN_HOST_TX     3   // -> Luckfox pin 16 SPI0 MISO (status frame out of the RP2350)
#define PIN_HOST_CS     1   // <- Luckfox pin 12 SPI0 CS0
#else
#define RADIO_SPI       spi0
#define PIN_RADIO_MISO  16  // SPI0 RX
#define PIN_RADIO_NSS   5   // plain GPIO chip-select
#define PIN_RADIO_SCK   18  // SPI0 SCK
#define PIN_RADIO_MOSI  19  // SPI0 TX
#define PIN_RADIO_BUSY  26  // input, no pull
#define PIN_RADIO_IRQ   27  // input, no pull (module IRQ = LR2021 DIO9)
#define PIN_RADIO_RESET 9   // output, active low

#define HZ_BOARD_NAME   "hat"  /* firmware before 2026-11-25: "vlink" */
// Host link (hat_tx only): the Raspberry Pi sends video records over SPI; the RP2350 is the SPI slave.
// SPI1 pins. Mode 1 (CPOL 0, CPHA 1) so the Pi can hold CS low for a whole record burst.
#define HOST_SPI        spi1
// In SLAVE mode the block receives on its RX pin and transmits on its TX pin, so:
#define PIN_HOST_MISO   8   // SPI1 RX  = data IN from the Pi   <-> Pi GPIO10 (MOSI)
#define PIN_HOST_SCK    10  // SPI1 SCK                         <-> Pi GPIO11 (SCLK)
#define PIN_HOST_MOSI   11  // SPI1 TX  = status OUT to the Pi  <-> Pi GPIO9  (MISO)
#define PIN_HOST_CS     13  // SPI1 CSn                         <-> Pi GPIO8  (CE0)
#endif

// Radio module defaults (the NiceRF LoRa2021F33 modules of boards 1..3): IRQ on DIO9, TCXO at 1.8 V, BUSY without
// pull, DIO5..8 drive the module's RF switch and 2.4 GHz front end (radio.c).
#ifndef RADIO_IRQ_DIO
#define RADIO_IRQ_DIO   LR20XX_SYSTEM_DIO_9
#endif
#ifndef RADIO_TCXO_CTRL
#define RADIO_TCXO_CTRL LR20XX_SYSTEM_TCXO_CTRL_1_8V
#endif
#ifndef RADIO_BUSY_PULLUP
#define RADIO_BUSY_PULLUP 0
#endif

#if HZ_BOARD == HZ_BOARD_GROUND_UNIT && defined(LIVE_TX)
#error "the Ground Unit is a receiver board (ground_unit)"
#endif
#if defined(HZ_HOST_PIO) && HZ_BOARD == HZ_BOARD_HAT
#error "HZ_HOST_PIO needs the Horizon Dev Air board (HZ_BOARD 3)"
#endif

#define RADIO_SPI_HZ_SLOW 3000000u   // boot / config
#define RADIO_SPI_HZ_FAST 16000000u  // after config (exact at 160 MHz, the Air; 15 MHz at the stock 150 MHz)
