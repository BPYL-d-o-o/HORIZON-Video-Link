// Board wiring: RP2350 <-> LR2021.
// This is the ONLY place pin numbers live. The board is chosen per build target (CMakeLists.txt, HZ_BOARD):
//   HZ_BOARD_VLINK     1  Pico 2 on the NiceRF vlink board (Pi transmitter, receivers)
//   HZ_BOARD_LUCKFOX   2  RP2354A on the Luckfox Pico Mini carrier (vlive_tx_luckfox)
//   HZ_BOARD_VIDEO_TX  3  integrated video TX board: Luckfox Pico, RP2354A and NiceRF LoRa2021F33 on one 51x31 mm
//                         carrier (vlive_tx_video_tx)
// The board code and its name are also what the firmware reports as its identity ("W" line / identity frame).
#pragma once

#include "hardware/spi.h"

#define HZ_BOARD_VLINK    1
#define HZ_BOARD_LUCKFOX  2
#define HZ_BOARD_VIDEO_TX 3
#ifndef HZ_BOARD
#define HZ_BOARD HZ_BOARD_VLINK
#endif

#if HZ_BOARD == HZ_BOARD_VIDEO_TX
#define HZ_BOARD_NAME   "video-tx"
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
#define PIN_LED         13  // red LED, high = on: status (main.c led_tick())
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

#if HZ_BOARD == HZ_BOARD_LUCKFOX
#define HZ_BOARD_NAME   "luckfox-carrier"
// Host link of vlive_tx_luckfox: a Luckfox Pico (RV1103 / RV1106) on the Luckfox carrier is the SPI master.
// The carrier wires the Luckfox data lines to GP11 / GP8, which is swapped relative to the SPI1
// RX/TX pin functions, so a PIO state machine is the SPI slave instead of the SPI1 block.
// SPI mode 0 (CPOL 0, CPHA 0), 8 bit, MSB first. Same records and status frame as the Pi link.
#define PIN_HOST_SCK    10  // <- Luckfox pin 7  SPI0 CLK
#define PIN_HOST_RX     11  // <- Luckfox pin 8  SPI0 MOSI (video records into the RP2350)
#define PIN_HOST_TX     8   // -> Luckfox pin 9  SPI0 MISO (status frame out of the RP2350)
#define PIN_HOST_CS     13  // <- Luckfox pin 6  SPI0 CS0
#else
#define HZ_BOARD_NAME   "vlink"
// Host link (vlive_tx only): the Raspberry Pi sends video records over SPI; the RP2350 is the SPI slave.
// SPI1 pins. Mode 1 (CPOL 0, CPHA 1) so the Pi can hold CS low for a whole record burst.
#define HOST_SPI        spi1
// In SLAVE mode the block receives on its RX pin and transmits on its TX pin, so:
#define PIN_HOST_MISO   8   // SPI1 RX  = data IN from the Pi   <-> Pi GPIO10 (MOSI)
#define PIN_HOST_SCK    10  // SPI1 SCK                         <-> Pi GPIO11 (SCLK)
#define PIN_HOST_MOSI   11  // SPI1 TX  = status OUT to the Pi  <-> Pi GPIO9  (MISO)
#define PIN_HOST_CS     13  // SPI1 CSn                         <-> Pi GPIO8  (CE0)
#endif
#endif

#if defined(VLIVE_HOST_PIO) && HZ_BOARD == HZ_BOARD_VLINK
#error "VLIVE_HOST_PIO needs a Luckfox board (HZ_BOARD 2 or 3)"
#endif

#define RADIO_SPI_HZ_SLOW 3000000u   // boot / config
#define RADIO_SPI_HZ_FAST 16000000u  // after config (exact at 160 MHz, the Air; 15 MHz at the stock 150 MHz)
