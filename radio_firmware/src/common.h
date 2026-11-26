// Shared by both roles: protocol constants, hopping, link ID, callsign packets, board identity, crash guard.
// One build is one role: tx.c (LIVE_TX) or rx.c.
#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_system.h"
#include "hardware/gpio.h"
#ifdef LIVE_TX
#include "hardware/dma.h"
#endif
#include "hardware/flash.h"  // TX: callsign; RX: pairing profile
#include "hardware/sync.h"
#ifdef HZ_HOST_PIO
#include "hardware/pio.h"
#include "hardware/sync.h"
#endif
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/exception.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "boot/picoboot_constants.h"  // REBOOT2_FLAG_REBOOT_TYPE_NORMAL
#include "radio.h"
#include "radio_hal.h"

#define ENTRY        253u     // host entry: PAYLOAD_LEN 255 minus seq16
#ifndef SLOT_US
#define SLOT_US      SLOT_US_LIVE
#endif
#ifndef AIR_US
#define AIR_US       AIR_US_LIVE
#endif
#ifndef AIR_US_LIVE
// SetTx -> TX_DONE is ~17.21 ms for a 255 B packet ("air_us=" in the T line), plus ~0.25 ms margin.
#define AIR_US_LIVE  17470u
#define SLOT_US_LIVE 17540u
#endif
#define RX_PRE_US    400u     // RX window opens this long before expected packet start
#define RX_POST_US   500u     // and closes this long after expected packet end
#define LOST_MISSES  32u      // consecutive misses before re-acquisition (~0.56 s)

// Identity. USB line, at boot and on "W":
//   W board=<name> board_id=<n> fw=<build target> git=<commit>[-dirty] fmt=<over-the-air format> uid=<16 hex>
//     radio=<starting | ok | radio_fail_cause()>
// The receiver answers "W" while it retries the radio start, so the setup site can say what is wrong.
// TX builds: the SPI record A5 1F 01 FE makes the RP2350 send this 32-byte frame on MISO for WHO_FRAME_US:
//   0 C3  1 49  2 HZ_ID_VERSION  3 board id  4 over-the-air format  5..8 commit (u32 LE, first 8 hex digits)
//   9 bit 0: dirty build  10..17 chip unique id  18..29 build target name (ASCII, zero padded, e.g. "air_radio";
//   firmware before 2026-11-25 left out a "vlive_" prefix: "tx_video_tx")
//   30 radio state (bits 0..1) | abs(radio rc) << 2  31 xor(bytes 2..30) ^ 0x5A
#ifndef HZ_FW_TARGET
#define HZ_FW_TARGET "unknown"
#endif
#ifndef HZ_FW_GIT
#define HZ_FW_GIT "unknown"
#endif
#define HZ_OTA_FORMAT 5u  // see format/README.md
#define HZ_ID_VERSION 2u

// Crash guard: a HardFault or interrupts stuck off would leave a USB serial port that never answers. A HardFault
// saves pc/lr and reboots; a watchdog fed from a timer interrupt catches the rest. The W line then shows
// reset=crash pc=.. lr=.. (look them up in the .dis) or reset=watchdog.
#define HZ_CRASH_MAGIC 0x48435253u  // watchdog scratch 0..2 (4..7 belong to the SDK); they survive the reboot

// Frequency hopping: see format/README.md "Frequency hopping". Centre = g_freq_hz; n = 0 is fixed frequency.
#define HOP_MAX 128u
typedef struct { uint8_t n; uint16_t step_khz; } hop_t;
extern hop_t g_hop;              // the current plan (common.c)
extern uint8_t hop_tab[HOP_MAX];
static inline unsigned hop_k(uint32_t seq) { return hop_tab[(uint16_t)seq % g_hop.n]; }
static inline int64_t hop_hz(uint32_t centre, const hop_t* h, unsigned k) {
    return (int64_t)centre + ((int64_t)k - (int64_t)(h->n / 2u)) * (int64_t)h->step_khz * 1000;
}
#define chan_idx(seq) (g_hop.n ? hop_k(seq) : 0u)  // the channel in P/M lines (fixed frequency: 0)

// Command slots: see format/README.md "Command slots". The ACK is a filler packet (FF FF FE) echoing the command and
// its effective slot. A TX-side power change (host record or USB "P <half_dbm>") is announced downlink-only in the
// ACK format with command id 0: CMD_ACKS times after a change and at boot, then every ANN_PERIOD slots, so receivers
// report the transmitter's real power.
#define ANN_PERIOD   512u        // ~8.9 s, 0.2 % of airtime
#define HOST_PWR_SYNC 0xC7u      // host -> TX power record: A5 C7 pwr ~pwr pwr^5A
#define CMD_PERIOD   64u         // must divide 65536
#define CMD_ACKS     3u          // ACK packets sent after a command
#define CMD_EFF      6u          // change takes effect CMD_EFF slots after the command slot
#define CMD_QUIET    2u          // silent slots for a frequency change
#define CMD_LEN      16u         // command packets are short (2.2 ms on air): they fit the command slot easily
#define CMD_F_FREQ   1u
#define CMD_F_PWR    2u
#define CMD_F_HOP    4u          // frequency-hopping plan (hz = centre); n = 0 means fixed frequency
#define CMD_F_CALL   8u          // one 7-byte part of the callsign blob
#define PLAN_ANN_SLOTS 144u      // hopping TX: plan announcement on the centre channel (takes a video slot)
#define VFREQ_MIN_HZ 150000000u  // LR2021 sub-GHz (LF) path
#define VFREQ_MAX_HZ 960000000u
#define VFREQ_HF_MIN_HZ 2400000000u  // the 2.4 GHz band (LR2021 HF path, radio.c): 2400..2483.5 MHz
#define VFREQ_HF_MAX_HZ 2483500000u
#define VPWR_MIN     (-19)       // -9.5 dBm, LF PA
#define VPWR_MAX     (44)        // +22 dBm, LF PA

typedef struct { uint8_t id, flags; uint32_t hz; int8_t pwr; uint16_t eff; uint8_t hn; uint16_t hstep;
                 uint8_t cpn, cd[7]; } vcmd_t;  // cpn / cd: CMD_F_CALL part (part << 4 | parts - 1) and its 7 bytes


static inline int16_t sdiff16(uint16_t a, uint16_t b) { return (int16_t)(uint16_t)(a - b); }

// Link ID: see format/README.md "Link ID". The TX RP2350 keys what its host sends (hosts build unkeyed entries).
//   commands: the marker C3 3C becomes C3^lo 3C^hi; ACK / announcement (FF FF FE): link in bytes 20..21 (LE);
//   idle packets: in the Air status (entry 21..22).
// The receiver keeps it in its pairing profile and reports it in "C cfg ... link=N".

// Pairing profile: second-to-last flash sector,
// "HZPR" ver=3 hz[4] n step_khz[2] pwr link[2] xor (all bytes from ver).
#define PROF_FLASH_OFF (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)
#define PROF_LEN 16u

// Callsign identification: see format/README.md "Callsign identification". The receiver prints
// "I seq part parts <20 hex digits>". Setting it from the receiver, one command per 7 bytes, ACKed like a frequency
// change:
//   command (16 bytes): seq16 | C3 3C A1 | id | 08 | part << 4 | (parts - 1) | d0..d5 | chk (over 5..13, 15) | d6
//   ACK (255 bytes):    seq16 | FF FF FE | id | 08 | pn | d0..d5 | chk | d6 | eff16 | xor(15..17) ^ 0x5A
// The TX keeps the callsign in the last flash sector.
#define CALL_MAX        112u     // blob bytes: 16 uplink parts of 7
#define IDENT_PART      10u      // blob bytes per ident packet
#define IDENT_DELAY_US  1000u    // ident packet starts this long into the command slot (the RX listens from the slot start)
#define IDENT_FIRST_US  5000000u
// Rules ask for an ident at least every 5 min: 4 min 50 s plus the command-slot wait and a second part stay under.
#define IDENT_PERIOD_US 290000000ull
// No callsign stored: identify as N0CALL; cleared on purpose (A5 1D with n = 0): do not transmit at all.
#define HOST_CALL_SYNC  0x1Du    // host -> TX: A5 1D n blob[n] xor(n, blob) ^ 0x5A    set the callsign (n = 0: none)
#define HOST_IDENT_SYNC 0x1Eu    // host -> TX: A5 1E m ~m   m = 1: identify now, 2: identify, then stop transmitting
#define HOST_WHO_SYNC   0x1Fu    // host -> TX: A5 1F 01 FE  send the identity frame for WHO_FRAME_US
#define HOST_STAT_SYNC  0x1Bu    // host -> TX: A5 1B code ~code   host error (Air status byte 11) until its next video record
#define HOST_PLAN_SYNC  0x1Cu    // host -> TX: A5 1C hz[4] n step_khz[2] xor   frequency / hop plan (n = 0: fixed),
                                 // applied like a TX-side "H": announced ahead, so receivers switch in the same slot

// Air status, in every idle packet (entry FF FF FF) in otherwise-zero bytes:
//   entry 3..5   48 53 01      "HS", version 1
//         6      state         1 video (a record in the last second)  2 the host polls but sends no video
//                              3 the host went silent (it was there before)  4 no host since the radio started
//                              5 end of transmission requested (the TX service stops)  6 the host reported an error
//         7..8   s since the host's last video record (u16 LE, 0xFFFF: none since the radio started)
//         9..10  s since the host last clocked SPI / wrote USB at all (u16 LE, 0xFFFF: never)
//         11     the host's error (HOST_STAT_SYNC): 0 none, 1 no camera detected, 2 camera / ISP does not start,
//                3 encoder does not start, 4 other start-up error
//         12..15 s since the radio started (u32 LE)   16..19 firmware commit (u32 LE, its first 8 hex digits)
//         20     xor(entry 3..19) ^ 0x5A
//         21..22 the link ID (LE): receivers ignore another system's status
#define AIR_ST_VIDEO    1u
#define AIR_ST_NO_VIDEO 2u
#define AIR_ST_SILENT   3u
#define AIR_ST_NO_HOST  4u
#define AIR_ST_STOPPING 5u
#define AIR_ST_ERROR    6u

// common.c
extern char g_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
extern volatile uint8_t g_radio_state;
extern volatile int g_radio_rc;
const char* radio_state_name(void);
extern char g_reset[48];
void __used hz_crash(const uint32_t* frame);
void hz_guard_start(void);
void hz_guard_stop(void);
int who_line(char* out, unsigned sz);
void who_print(void);
extern uint32_t g_freq_hz;
extern hop_t g_hop;
extern uint8_t hop_tab[HOP_MAX];
void hop_set(const hop_t* h);
bool hop_same(const hop_t* a, const hop_t* b);
uint32_t chan_hz(uint32_t seq);
uint32_t wait_irq(uint32_t timeout_us);
void recover(void);
bool vfreq_ok(int64_t hz);
bool hop_valid(uint32_t centre, const hop_t* h);
bool hop_retune(void);
bool vcmd_valid(const vcmd_t* c);
extern const uint8_t LINK_DEFAULT_BLOB[5];  // N0CALL, packed
extern uint16_t g_link;
extern uint8_t g_link_par[];
uint16_t link_of(const uint8_t* b, unsigned n);
void link_set(uint16_t l);
void link_key_entry(uint8_t* e);
bool link_ack_ok(const uint8_t* p);
void vcmd_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack);
bool vcmd_decode(const uint8_t* p, bool ack, vcmd_t* c);
bool hop_parse(const char* s, uint32_t* centre, hop_t* h);
void prof_encode(uint8_t* p, uint32_t hz, const hop_t* h, int8_t pwr, uint16_t link);
extern uint64_t g_host_rec_us;
extern uint64_t g_host_seen_us;
extern volatile uint8_t g_host_err;
int hexval(int c);
unsigned blob_len(const uint8_t* b, unsigned max);
void ident_encode(uint8_t* p, uint16_t seq, const uint8_t* blob, unsigned len, unsigned part);
bool ident_check(const uint8_t* p);
void vcall_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack);
bool vcall_decode(const uint8_t* p, bool ack, vcmd_t* c);
void radio_start(void (*on_fail)(void), void (*poll)(void));  // radio_init() until it starts (common.c)

// tx.c / rx.c
void tx_main(void);
void rx_main(void);
