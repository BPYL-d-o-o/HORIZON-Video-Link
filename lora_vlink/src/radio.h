#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "lr20xx_radio_lora_types.h"
#include "lr20xx_system_types.h"

#ifdef VLIVE
#include "vlive_config.h"
#define FIXED_FREQ_HZ VLIVE_FREQ_HZ
#endif
#ifdef FIXED_FREQ_HZ
#define RF_FREQ_HZ      FIXED_FREQ_HZ  // single-frequency builds
#else
#define RF_FREQ_HZ      915000000u
#endif
#ifdef VLIVE
// Live video (format v4): 255-byte packets (SF5 / 1 MHz / long-interleaver 4/5).
// seq16 LE (added by the TX RP2350) + a 253-byte entry from the host (see luckfox_vtx/hz_swfec.h).
// Note: the LR2021 datasheet lists 253 bytes as the maximum for long interleaving with CRC on (LORA_PL_LR2); 255 is
// what format v1 already used (and what was asked for).
#define PAYLOAD_LEN     255
#else
#define PAYLOAD_LEN     255    // 5-byte header + 250-byte FEC shard (flash-image demo targets)
#endif
#define LORA_PREAMBLE   12
#define LORA_SYNCWORD   0x12
// TX power in 0.5 dBm steps at the LR2021 pin (SetTxParams "TxPower").
// 44 = 22 dBm, the LR2021 maximum. On the bench keep the TX and RX antennas apart (or attenuated): too much
// input power can overload or damage a nearby receiver.
#ifndef TX_POWER_HALF_DBM
#define TX_POWER_HALF_DBM (44)
#endif

#define RADIO_IRQS (LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT | \
                    LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR | \
                    LR20XX_SYSTEM_IRQ_ERROR)

extern const void* const radio;  // driver context (unused by our HAL)

// Full init. Returns 0 on success, negative step number on failure.
int  radio_init(void);
bool radio_set_freq(uint32_t hz);
// Runtime change of carrier frequency: standby, set frequency, front-end calibration around it,
// back to FS. Takes a few ms; call only between packets. Leaves the modem settings untouched.
bool radio_retune(uint32_t hz);
// Frequency hopping: as radio_retune(), but the front-end calibration covers the whole hop band lo..hi (the LR2021
// takes 3 calibration points: lo, centre, hi; within +-10 MHz of the centre it is exactly radio_retune()).
bool radio_retune_span(uint32_t hz, uint32_t lo, uint32_t hi);
// Runtime change of TX power in 0.5 dBm steps (SetTxParams); PA config stays as set at init. On the 2.4 GHz path the
// power stays at RADIO_HF_POWER whatever is asked (see radio_is_hf); the request is kept for the sub-GHz path.
bool radio_set_power(int8_t half_dbm);
// The 2.4 GHz band: a retune to RADIO_HF_MIN_HZ or above re-initialises the radio on the HF path (LR2021 PA_HF and
// RX path HF, and on the NiceRF LoRa2021F33-2G4 its 2.4 GHz front end: DIO7 supply on, DIO8 its PA while
// transmitting, DIO5 its LNA), and back on the sub-GHz path below it. Same LoRa settings (SF5 / 1000 kHz / LI 4/5),
// so packets and slots are as on sub-GHz.
#define RADIO_HF_MIN_HZ 1500000000u
// TX power on the 2.4 GHz path, in 0.5 dBm at the LR2021 pin: -39 = -19.5 dBm, the minimum. The module's front-end PA
// adds about +20 dB (its table: -18 -> 10.8 dBm), so about 0 dBm leaves the module (to be measured).
#define RADIO_HF_POWER  (-39)
bool radio_is_hf(void);
bool radio_set_payload_len(uint8_t len);
