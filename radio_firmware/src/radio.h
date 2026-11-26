#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "lr20xx_radio_lora_types.h"
#include "lr20xx_system_types.h"

#include "link_config.h"
#define FIXED_FREQ_HZ HZ_FREQ_HZ
#define RF_FREQ_HZ      FIXED_FREQ_HZ  // single-frequency builds
// seq16 LE (added by the TX RP2350) + a 253-byte entry from the host (format/README.md "Packet contents").
// The LR2021 datasheet lists 253 bytes as the maximum for long interleaving with CRC on (LORA_PL_LR2); 255 works.
#define PAYLOAD_LEN     255
#define LORA_PREAMBLE   12
#define LORA_SYNCWORD   0x12
// TX power in 0.5 dBm steps at the LR2021 pin; 44 = 22 dBm, the maximum. On the bench keep the TX and RX antennas
// apart (or attenuated): too much input power can damage a nearby receiver.
#ifndef TX_POWER_HALF_DBM
#define TX_POWER_HALF_DBM (44)
#endif

#define RADIO_IRQS (LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT | \
                    LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR | \
                    LR20XX_SYSTEM_IRQ_ERROR)

extern const void* const radio;  // driver context (unused by our HAL)

// Returns 0 on success, negative step number on failure.
int  radio_init(void);
// radio_fail_cause(rc), for the "W" line's radio= and ERROR radio_<cause>:
//   step RADIO_STEP_RESET    busy_stuck_high  BUSY still high 0.5 s after reset: no module or no power, firmware for
//                                             another board (other pins), BUSY or NRESET not soldered
//   step RADIO_STEP_VERSION  spi_no_response  BUSY works but the version reads 0x0000 / 0xFFFF: MISO, SCK, MOSI or
//                                             NSS not connected, or firmware for another board
//   step RADIO_STEP_TCXO     tcxo_start       calibration reports HF_XOSC_START: the module's TCXO did not start
//                                             (firmware for another module: wrong TCXO voltage). Ground Unit only.
//   any other step           init_failed
#define RADIO_STEP_RESET   1
#define RADIO_STEP_VERSION 2
#define RADIO_STEP_TCXO    31
const char* radio_fail_cause(int rc);
bool radio_set_freq(uint32_t hz);
// Takes a few ms (standby, set frequency, front-end calibration, back to FS); call only between packets.
bool radio_retune(uint32_t hz);
// As radio_retune(), but the calibration covers the hop band lo..hi (3 points; within +-10 MHz it is radio_retune()).
bool radio_retune_span(uint32_t hz, uint32_t lo, uint32_t hi);
// On the 2.4 GHz path the power stays at RADIO_HF_POWER; the request is kept for the sub-GHz path.
bool radio_set_power(int8_t half_dbm);
// A retune to RADIO_HF_MIN_HZ or above re-initialises the radio on the HF path (and the NiceRF 2.4 GHz front end),
// below it back on sub-GHz. Same LoRa settings on both.
#define RADIO_HF_MIN_HZ 1500000000u
// -39 = -19.5 dBm at the LR2021 pin, the minimum: the module's front-end PA adds about +20 dB, so about 0 dBm leaves it.
#define RADIO_HF_POWER  (-39)
bool radio_is_hf(void);
bool radio_set_payload_len(uint8_t len);
