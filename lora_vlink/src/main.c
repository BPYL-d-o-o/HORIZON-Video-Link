// Horizon video link: A streams a flash-resident packet image with per-packet
// frequency hopping (US 902-928 MHz); B follows the hop pattern and forwards
// every packet to the host over USB as one text line.
//
// Packet (PAYLOAD_LEN = 255): seq16 LE | block16 LE | shard8 | 250-byte shard.
// The image holds (block16, shard8, shard[250]) entries = 253 bytes each,
// 20 entries per block (10 data + 10 Reed-Solomon parity), block-major.
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
#if defined(LIVE_TX) || defined(VLIVE)
#include "hardware/flash.h"  // TX: callsign; RX: pairing profile
#include "hardware/sync.h"
#endif
#ifdef VLIVE_HOST_PIO
#include "hardware/pio.h"
#include "hardware/sync.h"
#endif
#include "pico/stdlib.h"
#if defined(LIVE_TX) && !defined(VLIVE_HOST_PIO)
#include "pico/stdio_usb.h"
#endif
#include "pico/unique_id.h"
#include "radio.h"
#include "radio_hal.h"

#define NCH          32u
#define CH0_HZ       902600000u
#define CH_STEP_HZ   800000u
#ifdef VLIVE
#define ENTRY        253u     // host entry: PAYLOAD_LEN 255 minus seq16
#ifndef SLOT_US
#define SLOT_US      SLOT_US_V4
#endif
#ifndef AIR_US
#define AIR_US       AIR_US_V4
#endif
#else
#define ENTRY        253u
#ifndef SLOT_US
#define SLOT_US      18700u   // per-packet slot (air time + 1.3 ms retune margin)
#endif
#ifndef AIR_US
#define AIR_US       17400u   // SetTx -> TX_DONE / RX_DONE expected duration: SF5/1 MHz/LI 4/5/255 B = 17.031 ms + ~0.28 ms, rounded up
#endif
#endif
#ifndef AIR_US_V4
// Format v4 (SF5 / 1 MHz / long-interleaver 4/5 / 255 B): SetTx -> TX_DONE measured 17210..17215 us (vlive_tx_luckfox,
// "air_us=" in the T line); + ~0.25 ms. The slot leaves only ~0.33 ms between packets (as format v3: the RX re-arms right
// after a packet, see the tracking loop).
#define AIR_US_V4    17470u
#define SLOT_US_V4   17540u
#endif
#define RX_PRE_US    400u     // RX window opens this long before expected packet start
#define RX_POST_US   500u     // and closes this long after expected packet end
#ifdef VLIVE
#define LOST_MISSES  32u      // consecutive misses before re-acquisition (~0.56 s of 17.54 ms slots)
#else
#define LOST_MISSES  24u      // consecutive misses before re-acquisition
#endif

static char g_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
static uint8_t perm[NCH];

// ---------------------------------------------------------------------------
// Identity ("who am I"): which board this is, which firmware it runs and the chip's unique id, so a host can tell
// what it is connected to and which firmware update it needs. Every build:
//   USB line, at boot and on the command "W":
//     W board=<name> board_id=<n> fw=<build target> git=<commit>[-dirty] fmt=<over-the-air format> uid=<16 hex>
// TX builds also answer their SPI host (Luckfox / Pi): the record A5 1F 01 FE makes the RP2350 send this 32-byte
// identity frame on MISO instead of the status frame for WHO_FRAME_US (hosts that never ask see no change):
//   0 C3  1 49  2 identity version (1)  3 board id  4 over-the-air format  5..8 commit (u32 LE, its first 8 hex
//   digits)  9 bit 0: dirty build  10..17 chip unique id  18..29 build target name without its "vlive_" prefix
//   (ASCII, zero padded: "tx_video_tx" = vlive_tx_video_tx)  30 0
//   31 xor(bytes 2..30) ^ 0x5A
// ---------------------------------------------------------------------------
#ifndef HZ_FW_TARGET
#define HZ_FW_TARGET "unknown"
#endif
#ifndef HZ_FW_GIT
#define HZ_FW_GIT "unknown"
#endif
#define HZ_OTA_FORMAT 5u  // 5: link ID (binding phrase) keys the packets, see "Link ID"
#define HZ_ID_VERSION 2u   // 2: byte 30 = radio state (who_frame)
// Radio state for the host (TX): the host link starts before the radio, so a radio that does not start is reported
// (identity frame byte 30) instead of looking like a dead SPI link. 0 = not tried yet, 1 = running, 2 = failing.
static volatile uint8_t g_radio_state;
static volatile int g_radio_rc;

static int who_line(char* out, unsigned sz) {
    return snprintf(out, sz, "W board=%s board_id=%u fw=%s git=%s fmt=%u uid=%s\n", HZ_BOARD_NAME, (unsigned)HZ_BOARD,
                    HZ_FW_TARGET, HZ_FW_GIT, HZ_OTA_FORMAT, g_id);
}

static void who_print(void) {
    char l[160];
    if (who_line(l, sizeof l) > 0) printf("%s", l);
}

static void make_perm(void) {
    for (unsigned i = 0; i < NCH; i++) perm[i] = (uint8_t)i;
    uint32_t x = 0x5EED1234u;
    for (unsigned i = NCH - 1; i > 0; i--) {
        x = x * 1664525u + 1013904223u;
        unsigned j = (x >> 8) % (i + 1);
        uint8_t t = perm[i]; perm[i] = perm[j]; perm[j] = t;
    }
}
#ifdef FIXED_FREQ_HZ
// Single-channel build: no hopping, every slot on FIXED_FREQ_HZ (perm still drives the logged index).
// The value is the boot default; the live link can change it over the air (see "Over-the-air control").
static uint32_t g_freq_hz = FIXED_FREQ_HZ;
#ifdef VLIVE
// ---- Frequency hopping (vlive, optional; every board boots in fixed-frequency mode) ----
// Plan = centre frequency (g_freq_hz) + n channels + spacing. Channel k (0..n-1) is centre + (k - n/2) * spacing, so
// the centre is always channel n/2. Slot seq16 uses channel hop_tab[seq16 % n] (one packet per channel), a fixed
// pseudo-random order of the n channels that depends only on n: TX and RX compute the same channel from the slot
// number every packet already carries, nothing extra goes on air. n = 0 is the fixed-frequency mode (as before).
#define HOP_MAX 128u
typedef struct { uint8_t n; uint16_t step_khz; } hop_t;
static hop_t g_hop;                 // current plan
static uint8_t hop_tab[HOP_MAX];
static void hop_set(const hop_t* h) {
    g_hop = *h;
    if (!h->n) { g_hop.step_khz = 0; return; }
    for (unsigned i = 0; i < h->n; i++) hop_tab[i] = (uint8_t)i;
    uint32_t x = 0x48F0C3A5u ^ h->n;
    for (unsigned i = h->n - 1u; i > 0; i--) {
        x = x * 1664525u + 1013904223u;
        unsigned j = (x >> 8) % (i + 1u);
        uint8_t t = hop_tab[i]; hop_tab[i] = hop_tab[j]; hop_tab[j] = t;
    }
}
static bool hop_same(const hop_t* a, const hop_t* b) {
    return a->n == b->n && (!a->n || a->step_khz == b->step_khz);
}
static inline unsigned hop_k(uint32_t seq) { return hop_tab[(uint16_t)seq % g_hop.n]; }
static inline int64_t hop_hz(uint32_t centre, const hop_t* h, unsigned k) {
    return (int64_t)centre + ((int64_t)k - (int64_t)(h->n / 2u)) * (int64_t)h->step_khz * 1000;
}
static uint32_t chan_hz(uint32_t seq) { return g_hop.n ? (uint32_t)hop_hz(g_freq_hz, &g_hop, hop_k(seq)) : g_freq_hz; }
#define chan_idx(seq) (g_hop.n ? hop_k(seq) : perm[(seq) % NCH])
#else
static uint32_t chan_hz(uint32_t seq) { (void)seq; return g_freq_hz; }
#define chan_idx(seq) perm[(seq) % NCH]
#endif
#else
static uint32_t chan_hz(uint32_t seq) { return CH0_HZ + CH_STEP_HZ * perm[seq % NCH]; }
#define chan_idx(seq) perm[(seq) % NCH]
#endif

static uint32_t wait_irq(uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (!radio_hal_irq_pin()) {
        if (time_us_32() - t0 > timeout_us) {
            uint32_t irq = 0;
            lr20xx_system_get_and_clear_irq_status(radio, &irq);
            return irq;
        }
    }
    return radio_hal_read_clear_irq_fast();
}

static void recover(void) {
    lr20xx_system_set_standby_mode(radio, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    lr20xx_radio_fifo_clear_rx(radio);
    lr20xx_radio_fifo_clear_tx(radio);
}


// ---------------------------------------------------------------------------
// Over-the-air control (vlive only): change RF frequency / TX power from the receiver side.
//
// Every CMD_PERIOD-th slot (seq16 % CMD_PERIOD == 0) is a command slot. The TX does not send video
// in it: it listens, and the RX, if the host asked for a change, transmits one command packet
// (same modem settings and 255-byte length as video packets) in that slot. Timeline, S = command slot:
//   S      RX -> TX  command (id, flags, freq, power)
//   S+1    video as usual
//   S+2..4 TX -> RX  ACK (a filler packet, block 0xFFFF, shard byte 0xFE, echoes the command + effective slot)
//   S+5    video as usual
//   S+6    both sides apply the change. A frequency change takes CMD_QUIET silent slots (retune + front-end
//          calibration), video resumes on the new frequency at S+8. A power change alone has no gap.
// If the RX hears no ACK the change is not applied anywhere and the RX retries at the next command slot.
// If the ACKs are lost but the TX did switch, the RX's acquisition also scans the previous/new/boot
// frequencies and finds the TX again. No LoRa setting, packet format or FEC is touched.
//
// TX-side power changes (no uplink needed): the host on the TX (Luckfox: `service.sh power N`; Pi: SPI record)
// or the TX's own USB port ("P <half_dbm>") sets the power locally. The TX then announces its power downlink-only,
// in the ACK format with command id 0 (receivers treat it as filler): CMD_ACKS times after a change and at boot,
// and once every ANN_PERIOD slots so a receiver started later learns it. The RX reports it as its "pwr" value,
// so the receivers show the transmitter's real power. Remote changes (from the receiver) keep working as before.
#define ANN_PERIOD   512u        // periodic power announcement: 1 slot in 512 (~8.9 s, 0.2 % of airtime)
#define HOST_PWR_SYNC 0xC7u      // host -> TX power record: A5 C7 pwr ~pwr pwr^5A
// ---------------------------------------------------------------------------
#ifdef VLIVE
#define CMD_PERIOD   64u         // 1 slot in 64 (1.1 s) is a command slot; must divide 65536
#define CMD_ACKS     3u          // ACK packets sent after a command
#define CMD_EFF      6u          // change takes effect CMD_EFF slots after the command slot
#define CMD_QUIET    2u          // silent slots for a frequency change
#define CMD_LEN      16u         // command packets are short (2.2 ms on air): they fit the command slot easily
#define CMD_F_FREQ   1u
#define CMD_F_PWR    2u
#define CMD_F_HOP    4u          // frequency-hopping plan (hz = centre); n = 0 means fixed frequency
#define CMD_F_CALL   8u          // one part (7 bytes) of the station identification (callsign), see "Identification"
#define PLAN_ANN_SLOTS 144u      // hopping TX: plan announcement on the centre channel about every 2.5 s (for receivers
                                 // that join late; each one takes a video slot)
#define VFREQ_MIN_HZ 150000000u  // LR2021 sub-GHz (LF) path
#define VFREQ_MAX_HZ 960000000u
#define VFREQ_HF_MIN_HZ 2400000000u  // the 2.4 GHz band (LR2021 HF path, radio.c): 2400..2483.5 MHz
#define VFREQ_HF_MAX_HZ 2483500000u
// A carrier the radio may use: the sub-GHz range or the 2.4 GHz band.
static bool vfreq_ok(int64_t hz) {
    return (hz >= VFREQ_MIN_HZ && hz <= VFREQ_MAX_HZ) || (hz >= VFREQ_HF_MIN_HZ && hz <= VFREQ_HF_MAX_HZ);
}
#define VPWR_MIN     (-19)       // -9.5 dBm, LF PA
#define VPWR_MAX     (44)        // +22 dBm, LF PA

typedef struct { uint8_t id, flags; uint32_t hz; int8_t pwr; uint16_t eff; uint8_t hn; uint16_t hstep;
                 uint8_t cpn, cd[7]; } vcmd_t;  // cpn / cd: CMD_F_CALL part (part << 4 | parts - 1) and its 7 bytes

// All hop channels inside one band: the LR2021 sub-GHz range or the 2.4 GHz band (the band plan is the user's to set).
static bool hop_valid(uint32_t centre, const hop_t* h) {
    if (!vfreq_ok(centre)) return false;
    if (!h->n) return true;
    if (h->n < 2u || h->n > HOP_MAX || !h->step_khz) return false;
    int64_t lo = hop_hz(centre, h, 0), hi = hop_hz(centre, h, h->n - 1u);
    return vfreq_ok(lo) && vfreq_ok(hi) && (lo >= VFREQ_HF_MIN_HZ) == (hi >= VFREQ_HF_MIN_HZ);
}
// Retune + front-end calibration for the current plan (fixed: exactly radio_retune(g_freq_hz) as before).
static bool hop_retune(void) {
    if (!g_hop.n) return radio_retune(g_freq_hz);
    return radio_retune_span(g_freq_hz, (uint32_t)hop_hz(g_freq_hz, &g_hop, 0), (uint32_t)hop_hz(g_freq_hz, &g_hop, g_hop.n - 1u));
}

static int16_t sdiff16(uint16_t a, uint16_t b) { return (int16_t)(uint16_t)(a - b); }

static bool vcmd_valid(const vcmd_t* c) {
    if (!(c->flags & (CMD_F_FREQ | CMD_F_PWR | CMD_F_HOP)) || (c->flags & ~(CMD_F_FREQ | CMD_F_PWR | CMD_F_HOP))) return false;
    if ((c->flags & CMD_F_FREQ) && !vfreq_ok(c->hz)) return false;
    // (the LR2021's whole range: an announcement from a TX on the 2.4 GHz path reports RADIO_HF_POWER, below the
    // sub-GHz minimum; radio_set_power() keeps a sub-GHz request at that path's minimum)
    if ((c->flags & CMD_F_PWR) && (c->pwr < RADIO_HF_POWER || c->pwr > VPWR_MAX)) return false;
    if (c->flags & CMD_F_HOP) {
        hop_t h = {c->hn, c->hstep};
        if (!hop_valid(c->hz, &h)) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Link ID, format v5: the binding phrase (the callsign) keys every packet, so a receiver drops another Horizon
// system's packets as if they were damaged, and neither board acts on another system's commands or announcements.
//   link = 16 bits of FNV-1a over the callsign blob (see "Identification"; 0 is never used): link_of()
//   video / FEC entries: the entry's CRC-16 (entry 235..236) is XORed with the link, and the Reed-Solomon parity
//            (237..252) with the parity of that change (both are linear, so a keyed entry is still a valid RS codeword:
//            receivers repair it as before, then check the CRC with their link). The TX RP2350 applies it to what its
//            host sends (hosts build v4 entries), so with link 0 the packets are exactly format v4.
//   commands (receiver -> TX): the marker C3 3C becomes C3^lo 3C^hi
//   ACK / announcement (TX -> receivers, FF FF FE): the link in bytes 20..21 (LE)
//   idle packets (FF FF FF): the link in the Air status (entry 21..22, see "Air status")
//   identification (1D C5): not keyed: anyone may read who transmits
// The TX derives its link from its callsign; the receiver keeps it in its pairing profile (the setup site's Pair
// computes it from the same callsign) and reports it in "C cfg ... link=N", where the receiver apps learn it.
#include "../../luckfox_vtx/hz_swfec.h"  // CRC-16 and RS(253,237) of format v4 (the hosts' code)
static const uint8_t LINK_DEFAULT_BLOB[] = {0x86, 0xBB, 0x5B, 0x2C, 0x08};  // N0CALL: the default on both sides
static uint16_t g_link;
static uint8_t g_link_par[HZ_SW_RS_PAR];
static uint16_t link_of(const uint8_t* b, unsigned n) {
    uint32_t h = 2166136261u;
    for (unsigned i = 0; i < n; i++) h = (h ^ b[i]) * 16777619u;
    uint16_t l = (uint16_t)(h ^ (h >> 16));
    return l ? l : 1u;
}
static void link_set(uint16_t l) {
    static bool ready;
    if (!ready) { hz_sw_init(); ready = true; }
    g_link = l;
    uint8_t v[HZ_SW_RS_DATA];
    memset(v, 0, sizeof v);
    v[HZ_SW_CRC_AT] = (uint8_t)l; v[HZ_SW_CRC_AT + 1] = (uint8_t)(l >> 8);
    hz_rs_encode(v, HZ_SW_RS_DATA, g_link_par);
}
static void link_key_entry(uint8_t* e) {  // e: an entry (packet byte 2 on)
    e[HZ_SW_CRC_AT] ^= (uint8_t)g_link; e[HZ_SW_CRC_AT + 1] ^= (uint8_t)(g_link >> 8);
    for (unsigned i = 0; i < HZ_SW_RS_PAR; i++) e[HZ_SW_RS_DATA + i] ^= g_link_par[i];
}
static bool link_ack_ok(const uint8_t* p) { return p[20] == (uint8_t)g_link && p[21] == (uint8_t)(g_link >> 8); }

// CMD:  seq16 | C3 3C A1 | id flags hz[4] pwr eff16 chk | zeros     ACK: seq16 | FF FF FE | same body | zeros
// With CMD_F_HOP (hopping plan, hz = centre; older firmware rejects the unknown flag and ignores the packet):
//   CMD (16 bytes): byte 11 = n, 12..13 = spacing kHz (instead of pwr / eff)
//   ACK / announcement: the body as above (pwr, eff), then 15 = n, 16..17 = spacing kHz, 18 = chk2
static void vcmd_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack) {
    memset(p, 0, PAYLOAD_LEN);
    p[0] = (uint8_t)seq; p[1] = (uint8_t)(seq >> 8);
    p[2] = ack ? 0xFF : (uint8_t)(0xC3 ^ g_link); p[3] = ack ? 0xFF : (uint8_t)(0x3C ^ (g_link >> 8)); p[4] = ack ? 0xFE : 0xA1;
    if (ack) { p[20] = (uint8_t)g_link; p[21] = (uint8_t)(g_link >> 8); }
    p[5] = c->id; p[6] = c->flags;
    p[7] = (uint8_t)c->hz; p[8] = (uint8_t)(c->hz >> 8); p[9] = (uint8_t)(c->hz >> 16); p[10] = (uint8_t)(c->hz >> 24);
    p[11] = (uint8_t)c->pwr; p[12] = (uint8_t)c->eff; p[13] = (uint8_t)(c->eff >> 8);
    if ((c->flags & CMD_F_HOP) && !ack) {
        p[11] = c->hn; p[12] = (uint8_t)c->hstep; p[13] = (uint8_t)(c->hstep >> 8);
    }
    uint8_t x = 0x5A;
    for (unsigned i = 5; i <= 13; i++) x ^= p[i];
    p[14] = x;
    if ((c->flags & CMD_F_HOP) && ack) {
        p[15] = c->hn; p[16] = (uint8_t)c->hstep; p[17] = (uint8_t)(c->hstep >> 8);
        p[18] = (uint8_t)(0x5A ^ p[15] ^ p[16] ^ p[17]);
    }
}
static bool vcmd_decode(const uint8_t* p, bool ack, vcmd_t* c) {
    if (ack ? !(p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE && link_ack_ok(p))
            : !(p[2] == (uint8_t)(0xC3 ^ g_link) && p[3] == (uint8_t)(0x3C ^ (g_link >> 8)) && p[4] == 0xA1)) return false;
    uint8_t x = 0x5A;
    for (unsigned i = 5; i <= 13; i++) x ^= p[i];
    bool hop = (p[6] & CMD_F_HOP) != 0;
    if (x != p[14]) return false;
    c->id = p[5]; c->flags = p[6];
    c->hz = (uint32_t)p[7] | (uint32_t)p[8] << 8 | (uint32_t)p[9] << 16 | (uint32_t)p[10] << 24;
    c->pwr = (int8_t)p[11]; c->eff = (uint16_t)(p[12] | p[13] << 8);
    c->hn = 0; c->hstep = 0;
    if (hop && !ack) {
        c->hn = p[11]; c->hstep = (uint16_t)(p[12] | p[13] << 8);
        c->pwr = 0; c->eff = 0;
    } else if (hop) {
        if ((uint8_t)(0x5A ^ p[15] ^ p[16] ^ p[17]) != p[18]) return false;
        c->hn = p[15]; c->hstep = (uint16_t)(p[16] | p[17] << 8);
    }
    return vcmd_valid(c);
}
// Text "H <centre_hz> <n> <spacing_hz>" (n = 0: fixed frequency at centre_hz) -> plan. False if malformed.
static bool hop_parse(const char* s, uint32_t* centre, hop_t* h) {
    unsigned long hz = 0, n = 0, step = 0;
    int got = sscanf(s, "%lu %lu %lu", &hz, &n, &step);
    if (got < 2) return false;
    if (n == 0) step = 0;
    else if (got < 3 || n > HOP_MAX || step % 1000u || step / 1000u > 65535u) return false;
    *centre = (uint32_t)hz;
    h->n = (uint8_t)n; h->step_khz = (uint16_t)(step / 1000u);
    return hop_valid(*centre, h);
}

// Pairing profile flash layout, shared by the receiver and the transmitter (see "Pairing profile"):
// second-to-last flash sector, "HZPR" ver=3 hz[4] n step_khz[2] pwr link[2] xor (all bytes from ver).
#define PROF_FLASH_OFF (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)
#define PROF_LEN 16u
static void prof_encode(uint8_t* p, uint32_t hz, const hop_t* h, int8_t pwr, uint16_t link) {
    p[0] = 'H'; p[1] = 'Z'; p[2] = 'P'; p[3] = 'R'; p[4] = 3;
    p[5] = (uint8_t)hz; p[6] = (uint8_t)(hz >> 8); p[7] = (uint8_t)(hz >> 16); p[8] = (uint8_t)(hz >> 24);
    p[9] = h->n; p[10] = (uint8_t)h->step_khz; p[11] = (uint8_t)(h->step_khz >> 8); p[12] = (uint8_t)pwr;
    p[13] = (uint8_t)link; p[14] = (uint8_t)(link >> 8);
    uint8_t x = 0x5A;
    for (unsigned i = 4; i < 15; i++) x ^= p[i];
    p[15] = x;
}

// ---------------------------------------------------------------------------
// Identification (callsign), format v4.
//
// The callsign is kept as a "blob" (encoded by the receiver / the TX host, the firmware only stores and sends it):
//   blob[0] = bit 7: packed (1) / raw (0), bits 0..6: character count n (0 = no callsign)
//   packed: characters A-Z, 0-9, "/", "-", " " mapped to 1..39 (0 = padding), three per 16-bit little-endian word
//           (c0 * 1600 + c1 * 40 + c2): 1 + 2 * ceil(n / 3) bytes; e.g. "S57ABC" = 5 bytes
//   raw:    n bytes of UTF-8 text: 1 + n bytes (anything that does not fit the packed alphabet)
// On air (TX -> RX) it goes out in a command slot, where the TX would otherwise only listen, so it costs no video:
//   ident packet (CMD_LEN = 16 bytes, ~2.2 ms on air, sent IDENT_DELAY_US into the slot):
//   seq16 | 1D C5 | part << 4 | (parts - 1) | 10 blob bytes | xor(bytes 2..14) ^ 0x5A
// A packed callsign of up to 12 characters is one packet. The TX sends it IDENT_FIRST_US after boot, every
// IDENT_PERIOD_US while it transmits, once more when the host asks (end of transmission: the TX service stops), and right
// after a new callsign is set. The receiver prints "I seq part parts <20 hex digits>" (the receivers show the callsign).
// Setting it from the receiver (RX -> TX, a short-range feature: one command per 7 bytes, ACKed like a frequency change):
//   command (16 bytes): seq16 | C3 3C A1 | id | 08 | part << 4 | (parts - 1) | d0..d5 | chk (over 5..13, 15) | d6
//   ACK (255 bytes):    seq16 | FF FF FE | id | 08 | pn | d0..d5 | chk | d6 | eff16 | xor(15..17) ^ 0x5A
// The TX keeps the callsign in the last flash sector (it survives power cycles); the TX host can also set it
// (record A5 1D n blob[n] xor, e.g. hz_vtx --callsign).
#define CALL_MAX        112u     // blob bytes: 16 uplink parts of 7
#define IDENT_PART      10u      // blob bytes per ident packet
#define IDENT_DELAY_US  1000u    // ident packet starts this long into the command slot (the RX listens from the slot start)
#define IDENT_FIRST_US  5000000u
// Slovenian amateur rules: identify at least every 5 minutes. 4 min 50 s plus the wait for a command slot (<= 1.1 s)
// and a second ident packet for a long callsign keep every gap under 5 minutes.
#define IDENT_PERIOD_US 290000000ull  // 4 min 50 s
// No callsign stored: the TX identifies as the N0CALL default (debugging, call_load); a callsign cleared on purpose
// (record A5 1D with n = 0) means no transmission at all, until one is set (hz_vtx --callsign / the setup site Pair).
#define HOST_CALL_SYNC  0x1Du    // host -> TX: A5 1D n blob[n] xor(n, blob) ^ 0x5A    set the callsign (n = 0: none)
#define HOST_IDENT_SYNC 0x1Eu    // host -> TX: A5 1E m ~m   m = 1: identify now, 2: identify, then stop transmitting
#define HOST_WHO_SYNC   0x1Fu    // host -> TX: A5 1F 01 FE  send the identity frame (see "Identity") for WHO_FRAME_US
#define HOST_STAT_SYNC  0x1Bu    // host -> TX: A5 1B code ~code   the host cannot send video (code: see "Air status"), until its next
                                 // video record; carried in the Air status (below), so receivers can say why
#define HOST_PLAN_SYNC  0x1Cu    // host -> TX: A5 1C hz[4] n step_khz[2] xor   frequency / hop plan (n = 0: fixed)
                                 // (Horizon Dev pairing: the Luckfox keeps the paired plan and sends it at every start;
                                 // applied like a TX-side "H": announced ahead, so receivers switch in the same slot)
                                 // (end of transmission) until the next video record

// ---------------------------------------------------------------------------
// Air status, format v4 (TX -> RX, for "why is there no picture?"). Every idle packet (entry FF FF FF, sent when no
// video waits) carries it in bytes that were zero before, so it costs no airtime and older receivers still see filler:
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
//         21..22 the link ID (LE, see "Link ID"): receivers ignore another system's status
// The receivers (Horizon Receiver, the setup site) show it when the picture stops.
#define AIR_ST_VIDEO    1u
#define AIR_ST_NO_VIDEO 2u
#define AIR_ST_SILENT   3u
#define AIR_ST_NO_HOST  4u
#define AIR_ST_STOPPING 5u
#define AIR_ST_ERROR    6u
static uint64_t g_host_rec_us;     // the host's last video record (0: none yet)
static uint64_t g_host_seen_us;    // the host's last byte on SPI / USB, polls included (0: never)
static volatile uint8_t g_host_err;  // HOST_STAT_SYNC code, cleared by the next video record

static int hexval(int c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; }
// Bytes the blob really uses (from its header), 0 if it does not fit in max.
static unsigned blob_len(const uint8_t* b, unsigned max) {
    if (!max) return 0;
    unsigned n = b[0] & 0x7Fu;
    unsigned len = (b[0] & 0x80u) ? 1u + 2u * ((n + 2u) / 3u) : 1u + n;
    return len <= max && len <= CALL_MAX ? len : 0;
}
static void ident_encode(uint8_t* p, uint16_t seq, const uint8_t* blob, unsigned len, unsigned part) {
    unsigned parts = (len + IDENT_PART - 1u) / IDENT_PART;
    memset(p, 0, CMD_LEN);
    p[0] = (uint8_t)seq; p[1] = (uint8_t)(seq >> 8);
    p[2] = 0x1D; p[3] = 0xC5; p[4] = (uint8_t)(part << 4 | (parts - 1u));
    for (unsigned i = 0; i < IDENT_PART && part * IDENT_PART + i < len; i++) p[5 + i] = blob[part * IDENT_PART + i];
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 14; i++) x ^= p[i];
    p[15] = x;
}
static bool ident_check(const uint8_t* p) {
    if (p[2] != 0x1D || p[3] != 0xC5) return false;
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 14; i++) x ^= p[i];
    return x == p[15] && (p[4] >> 4) <= (p[4] & 15u);
}
static void vcall_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack) {
    memset(p, 0, ack ? PAYLOAD_LEN : CMD_LEN);
    p[0] = (uint8_t)seq; p[1] = (uint8_t)(seq >> 8);
    p[2] = ack ? 0xFF : (uint8_t)(0xC3 ^ g_link); p[3] = ack ? 0xFF : (uint8_t)(0x3C ^ (g_link >> 8)); p[4] = ack ? 0xFE : 0xA1;
    if (ack) { p[20] = (uint8_t)g_link; p[21] = (uint8_t)(g_link >> 8); }
    p[5] = c->id; p[6] = CMD_F_CALL; p[7] = c->cpn;
    memcpy(p + 8, c->cd, 6);
    p[15] = c->cd[6];
    uint8_t x = 0x5A;
    for (unsigned i = 5; i <= 13; i++) x ^= p[i];
    p[14] = x ^ p[15];
    if (ack) { p[16] = (uint8_t)c->eff; p[17] = (uint8_t)(c->eff >> 8); p[18] = (uint8_t)(0x5A ^ p[15] ^ p[16] ^ p[17]); }
}
static bool vcall_decode(const uint8_t* p, bool ack, vcmd_t* c) {
    if (ack ? !(p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE && link_ack_ok(p))
            : !(p[2] == (uint8_t)(0xC3 ^ g_link) && p[3] == (uint8_t)(0x3C ^ (g_link >> 8)) && p[4] == 0xA1)) return false;
    if (p[6] != CMD_F_CALL) return false;
    uint8_t x = 0x5A;
    for (unsigned i = 5; i <= 13; i++) x ^= p[i];
    if ((uint8_t)(x ^ p[15]) != p[14]) return false;
    if (ack && (uint8_t)(0x5A ^ p[15] ^ p[16] ^ p[17]) != p[18]) return false;
    if ((p[7] >> 4) > (p[7] & 15u)) return false;
    memset(c, 0, sizeof *c);
    c->id = p[5]; c->flags = CMD_F_CALL; c->cpn = p[7];
    memcpy(c->cd, p + 8, 6);
    c->cd[6] = p[15];
    c->eff = ack ? (uint16_t)(p[16] | p[17] << 8) : 0;
    return true;
}
#endif

// ---------------------------------------------------------------------------
// TX (A)
// ---------------------------------------------------------------------------
#ifdef VIDEO_TX
extern const uint8_t video_image[], video_image_end[];

static void run_tx(void) {
    const uint32_t entries = (uint32_t)(video_image_end - video_image) / ENTRY;
    printf("TX image entries=%lu blocks=%lu slot_us=%u\n", (unsigned long)entries, (unsigned long)(entries / 20),
           SLOT_US);
    uint8_t pkt[PAYLOAD_LEN];
    uint32_t seq = 0, sent = 0, fail = 0, late = 0, txmax = 0, txmin = ~0u;
    uint64_t txsum = 0;
    // Keep TCXO and PLL running between packets (and before the first one):
    // a TX from standby adds the TCXO start-up time and shifts the first slot.
    lr20xx_system_set_fs_mode(radio);
    uint64_t t = time_us_64() + 200000;
    for (;;) {
        pkt[0] = (uint8_t)seq; pkt[1] = (uint8_t)(seq >> 8);
        memcpy(pkt + 2, video_image + (seq % entries) * ENTRY, ENTRY);
        lr20xx_radio_fifo_write_tx(radio, pkt, PAYLOAD_LEN);
        while (time_us_64() < t) tight_loop_contents();
        radio_set_freq(chan_hz(seq));
        uint32_t t0 = time_us_32();
        lr20xx_radio_common_set_tx(radio, 0);
        uint32_t irq = wait_irq(SLOT_US + 20000);
        uint32_t dt = time_us_32() - t0;
        if (irq & LR20XX_SYSTEM_IRQ_TX_DONE) {
            sent++; txsum += dt;
            if (dt > txmax) txmax = dt;
            if (dt < txmin) txmin = dt;
        } else {
            fail++;
            recover();
        }
        // Next slot; if we overran, skip whole slots so seq stays on schedule.
        seq++; t += SLOT_US;
        uint64_t now = time_us_64();
        while (now + 150 > t) { t += SLOT_US; seq++; late++; }
        if (sent + fail >= 500) {
            printf("T seq=%lu sent=%lu fail=%lu late=%lu tx_us mean=%lu min=%lu max=%lu\n", (unsigned long)seq,
                   (unsigned long)sent, (unsigned long)fail, (unsigned long)late,
                   sent ? (unsigned long)(txsum / sent) : 0, (unsigned long)txmin, (unsigned long)txmax);
            txmax = 0; txmin = ~0u; txsum = 0; sent = 0; fail = 0;
        }
    }
}
#endif

// ---------------------------------------------------------------------------
// Live TX (vlive_tx): host-fed packets on one fixed frequency (FIXED_FREQ_HZ).
// Host -> RP2350 records (over SPI, see board.h): A5 5A | entry[253] (block16 LE, shard8, 250-byte shard) | xor8(entry).
// One queued entry goes out per SLOT_US slot; with the queue empty a filler
// (block 0xFFFF) goes out instead, so the receiver stays locked between frames.
// RP2350 -> host, every slot: "Q depth accepted" (accepted = good records taken from USB),
// every 5 s: "T seq=.. sent=.. fail=.. late=.. data=.. idle=.. bad=..".
// ---------------------------------------------------------------------------
#ifdef LIVE_TX
#define LQ_CAP  24u
#define REC_LEN (2u + ENTRY + 1u)
static uint8_t lq[LQ_CAP][ENTRY];
static uint32_t lq_head, lq_tail, rec_ok, rec_bad;
static uint8_t rec[REC_LEN];
static unsigned rec_n;

// Host records arrive over SPI (RP2350 = slave). A DMA channel drains the SPI RX FIFO into a 4 KB ring
// buffer with no CPU involvement, so no byte is lost while the CPU is busy with the radio.
// poll_host() parses what the DMA has written, but only as many bytes as fit in the queue: the sender
// paces itself from the "Q depth accepted" status lines (still sent over USB), as before.
#define HOST_RING_BITS 12u
#define HOST_RING_SIZE (1u << HOST_RING_BITS)
static uint8_t host_ring[HOST_RING_SIZE] __attribute__((aligned(HOST_RING_SIZE)));
static unsigned host_dma, host_rd, host_tx_dma;
static uint32_t g_host_bytes;                   // bytes taken from the host ring (the "H" line's host link counters)
static void host_status_publish(void);
#ifndef VLIVE_HOST_PIO
static void usb_host_poll_throttled(void);
#endif

// Status frame, 32 bytes, repeated endlessly on MISO by a second DMA channel (the Pi reads it with any SPI
// transfer and finds the frame by its sync bytes + checksum; a frame torn by an update fails the checksum):
//  0 C3  1 3C  2 queue depth  3 event counter  4..7 records accepted (u32)  8..9 sent  10..11 fail
//  12..13 late  14..15 data  16..17 idle  18 bad  19 TX power (half dBm, i8)  20..23 freq_hz (u32)
//  24..25 seq16  26 command slots entered  27 command-slot IRQs  28 last command-slot flags (below)
//  29..30 arrival of the last command-slot IRQ, 16 us units from 2 ms before the slot start
//  31 xor(bytes 2..30) ^ 0x5A      (counters are cumulative, 16-bit, wrap)
//  command-slot flags: 1 RX_DONE  2 CRC error  4 length error  8 other IRQ  16 wrong length  32 decoded  64 not a command
//  byte 28 bit 7: flips after every completed identification (the host waits for it before it stops)
#define STATUS_LEN 32u
static uint8_t host_status[STATUS_LEN] __attribute__((aligned(STATUS_LEN)));
static uint16_t tot_sent, tot_fail, tot_late, tot_data, tot_idle;
static uint8_t ev_count;
static int8_t g_txpwr = TX_POWER_HALF_DBM;
// The power the LR2021 really transmits with (0.5 dBm at its pin), as reported to the host and the receivers: the
// requested power on sub-GHz; on the 2.4 GHz path radio.c keeps RADIO_HF_POWER whatever is asked. (What leaves the
// antenna depends on the module: a module with a PA adds its gain.)
static int8_t pwr_out(int8_t requested) { return radio_is_hf() ? (int8_t)RADIO_HF_POWER : requested; }
#define WHO_FRAME_US 300000u
static uint64_t g_who_until;                    // identity frame on MISO until then (host record A5 1F)
static int16_t g_local_pwr_req = INT16_MIN;  // TX-side power request (host record / USB), applied between packets
static volatile bool g_local_hop_req;           // TX-side hop plan request (USB "H ..."), applied between packets
static hop_t g_local_hop;
static uint32_t g_local_hop_hz;
static void tx_hop_text(const char* line) {
    uint32_t hz; hop_t h;
    if (hop_parse(line + 1, &hz, &h)) { g_local_hop_hz = hz; g_local_hop = h; g_local_hop_req = true; }
    else printf("C fail hop (H <centre_hz> <n 0|2..%u> <spacing_hz, kHz steps>, all channels %lu..%lu Hz)\n",
                HOP_MAX, (unsigned long)VFREQ_MIN_HZ, (unsigned long)VFREQ_MAX_HZ);
}
static uint32_t g_txseq;
static uint8_t cmd_slots, cmd_irqs, cmd_flags;  // command-slot diagnostics, reported in the status frame
static uint16_t cmd_arr;

// Identification (see "Identification" above): callsign blob, kept in the last flash sector.
static uint8_t g_call[CALL_MAX];
static unsigned g_call_len;                  // 0 = no callsign: nothing is sent
static volatile bool g_call_save;            // write g_call to flash between two packets
static volatile uint8_t g_ident_req;         // host: 1 identify now, 2 identify and then stop transmitting
static uint8_t g_ident_toggle;               // flips after every completed identification (status frame byte 28 bit 7)
static uint64_t g_ident_next;                // next periodic identification (0 = at the next command slot)
static bool g_quiet;                         // end of transmission: nothing is sent until the next video record
static uint32_t g_idents;
static uint8_t call_stage[CALL_MAX];         // callsign arriving from the receiver in parts of 7 bytes
static uint32_t call_stage_mask;
static unsigned call_stage_n;
#define CALL_FLASH_OFF (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// No callsign stored: identify as the standard dummy "N0CALL" (debugging; packed blob, swfec.encode_call). Not saved:
// the first real callsign (TX host / receiver) replaces it. An explicitly cleared callsign (n = 0) still means silence.
static const uint8_t CALL_DEFAULT[] = {0x86, 0xBB, 0x5B, 0x2C, 0x08};
static bool g_call_default;                  // g_call is the N0CALL default (identity frame byte 9 bit 2)
static void call_load(void) {
    const uint8_t* f = (const uint8_t*)(XIP_BASE + CALL_FLASH_OFF);
    memcpy(g_call, CALL_DEFAULT, sizeof CALL_DEFAULT);
    g_call_len = sizeof CALL_DEFAULT;
    g_call_default = true;
    if (f[0] != 'C' || f[1] != 'A' || f[2] != 'L' || f[3] != 'L' || f[4] > CALL_MAX) return;
    uint8_t x = 0x5A;
    for (unsigned i = 0; i < f[4]; i++) x ^= f[5 + i];
    if (x != f[5 + f[4]] || (f[4] && blob_len(f + 5, f[4]) != f[4])) return;
    memcpy(g_call, f + 5, f[4]);
    g_call_len = f[4];       // (0: cleared on purpose: silent)
    g_call_default = false;
}
static void call_link(void) {  // the link ID from the callsign (see "Link ID")
    link_set(g_call_len ? link_of(g_call, g_call_len) : link_of(LINK_DEFAULT_BLOB, sizeof LINK_DEFAULT_BLOB));
    printf("C link=%u\n", g_link);
}
// Erase + program the last sector: ~50 ms with interrupts off (one core runs; the host SPI keeps going by DMA).
static void call_store(void) {
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    page[0] = 'C'; page[1] = 'A'; page[2] = 'L'; page[3] = 'L'; page[4] = (uint8_t)g_call_len;
    uint8_t x = 0x5A;
    for (unsigned i = 0; i < g_call_len; i++) { page[5 + i] = g_call[i]; x ^= g_call[i]; }
    page[5 + g_call_len] = x;
    uint32_t ir = save_and_disable_interrupts();
    flash_range_erase(CALL_FLASH_OFF, FLASH_SECTOR_SIZE);
    flash_range_program(CALL_FLASH_OFF, page, FLASH_PAGE_SIZE);
    restore_interrupts(ir);
}
static void call_print(const char* what) {
    printf("C call %s len=%u blob=", what, g_call_len);
    for (unsigned i = 0; i < g_call_len; i++) printf("%02x", g_call[i]);
    printf("\n");
}
// New callsign (the receiver over the air, or the TX host): stored, saved to flash, and sent at the next command slot.
static void call_set(const uint8_t* b, unsigned len) {
    if (!len || !(b[0] & 0x7Fu)) len = 0;
    if (len == g_call_len && !memcmp(b, g_call, len)) { call_print("unchanged"); return; }
    memcpy(g_call, b, len);
    g_call_len = len;
    g_call_default = false;
    g_call_save = true;
    g_ident_next = 0;
    call_print("set");
    call_link();
}
static void call_stage_part(const vcmd_t* c) {
    unsigned part = c->cpn >> 4, n = (c->cpn & 15u) + 1u;
    if (part == 0 || n != call_stage_n) { call_stage_mask = 0; call_stage_n = n; memset(call_stage, 0, sizeof call_stage); }
    memcpy(call_stage + part * 7u, c->cd, 7);
    call_stage_mask |= 1u << part;
}
static void call_stage_apply(const vcmd_t* c) {
    unsigned part = c->cpn >> 4, n = (c->cpn & 15u) + 1u;
    if (part + 1u != n) { printf("C applied call part %u/%u\n", part + 1u, n); return; }
    unsigned len = blob_len(call_stage, n * 7u);
    if (call_stage_n != n || call_stage_mask != (1u << n) - 1u || !len) { printf("C fail call (parts missing)\n"); return; }
    call_set(call_stage, len);
}
// ---- Pairing profile, transmitter side (the Raspberry Pi HAT, vlive_tx): the receiver's "K" lines and flash layout
// (see "Pairing profile"), so the setup site pairs the HAT over its USB port exactly like the Ground. The saved plan
// and power are used from boot; a host's plan / power records (live_sender.py / hz_vtx --freq, --hop, --tx-power) still
// override them until the next restart. The link is this transmitter's own (from its callsign): stored for reading
// back only.
//   "K"                                           -> K profile freq_hz=.. hop=n,spacing_hz pwr=.. saved=0|1 link=..
//                                                    callblob=<hex>|-   (the saved plan, else the current one)
//   "K <centre_hz> <n> <spacing_hz> <pwr> [link]" -> saved to flash between two packets, then announced to the
//                                                    receivers and applied like "H" / "P"; replies "K saved ..."
static bool g_tx_prof_saved;
static volatile bool g_tx_prof_save;            // write g_tx_prof_* to flash between two packets
static uint32_t g_tx_prof_hz;
static hop_t g_tx_prof_hop;
static int8_t g_tx_prof_pwr;
static bool tx_prof_read(void) {
    const uint8_t* f = (const uint8_t*)(XIP_BASE + PROF_FLASH_OFF);
    if (f[0] != 'H' || f[1] != 'Z' || f[2] != 'P' || f[3] != 'R' || f[4] != 3) return false;
    uint8_t x = 0x5A;
    for (unsigned i = 4; i < 15; i++) x ^= f[i];
    if (x != f[15]) return false;
    uint32_t hz = (uint32_t)f[5] | (uint32_t)f[6] << 8 | (uint32_t)f[7] << 16 | (uint32_t)f[8] << 24;
    hop_t h = {f[9], (uint16_t)(f[10] | f[11] << 8)};
    int8_t pwr = (int8_t)f[12];
    if (!hop_valid(hz, &h) || pwr < VPWR_MIN || pwr > VPWR_MAX) return false;
    g_tx_prof_hz = hz; g_tx_prof_hop = h; g_tx_prof_pwr = pwr;
    return true;
}
// At boot (before the first packet): onto the saved plan (2.4 GHz: radio.c switches to the HF path in the retune),
// and the saved power at the first slot.
static void tx_prof_load(void) {
    if (!tx_prof_read()) { printf("K profile none saved: build default freq_hz=%lu\n", (unsigned long)g_freq_hz); return; }
    g_tx_prof_saved = true;
    g_freq_hz = g_tx_prof_hz; hop_set(&g_tx_prof_hop);
    if (!hop_retune()) printf("radio: retune to the profile's plan failed\n");
    g_local_pwr_req = g_tx_prof_pwr;
}
static void tx_prof_print(const char* what) {
    uint32_t hz = g_tx_prof_saved ? g_tx_prof_hz : g_freq_hz;
    const hop_t* h = g_tx_prof_saved ? &g_tx_prof_hop : &g_hop;
    printf("K %s freq_hz=%lu hop=%u,%lu pwr=%d saved=%d link=%u callblob=", what, (unsigned long)hz, h->n,
           (unsigned long)h->step_khz * 1000u, g_tx_prof_saved ? g_tx_prof_pwr : g_txpwr, g_tx_prof_saved, g_link);
    for (unsigned i = 0; i < g_call_len; i++) printf("%02x", g_call[i]);
    printf("%s\n", g_call_len ? "" : "-");
}
static void tx_prof_command(const char* s) {
    while (*s == ' ') s++;
    if (!*s) { tx_prof_print("profile"); return; }
    unsigned long hz = 0, n = 0, step = 0, link = 0;
    long pwr = 0;
    char plan[64];
    uint32_t centre; hop_t h;
    if (sscanf(s, "%lu %lu %lu %ld %lu", &hz, &n, &step, &pwr, &link) < 4) {
        printf("C fail profile (K <centre_hz> <n 0|2..%u> <spacing_hz> <pwr_half_dbm> [<link>])\n", HOP_MAX);
        return;
    }
    snprintf(plan, sizeof plan, "%lu %lu %lu", hz, n, step);
    if (!hop_parse(plan, &centre, &h) || pwr < VPWR_MIN || pwr > VPWR_MAX) { printf("C fail profile range\n"); return; }
    g_tx_prof_hz = centre; g_tx_prof_hop = h; g_tx_prof_pwr = (int8_t)pwr;
    g_tx_prof_save = true;
}
// Called between two packets (interrupts off ~50 ms, as call_store()).
static void tx_prof_store(void) {
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    prof_encode(page, g_tx_prof_hz, &g_tx_prof_hop, g_tx_prof_pwr, g_link);
    uint32_t ir = save_and_disable_interrupts();
    flash_range_erase(PROF_FLASH_OFF, FLASH_SECTOR_SIZE);
    flash_range_program(PROF_FLASH_OFF, page, FLASH_PAGE_SIZE);
    restore_interrupts(ir);
    if (memcmp((const void*)(XIP_BASE + PROF_FLASH_OFF), page, PROF_LEN)) { printf("C fail profile (flash write)\n"); return; }
    g_tx_prof_saved = true;
    if (g_tx_prof_hz != g_freq_hz || !hop_same(&g_tx_prof_hop, &g_hop)) {  // announced, then applied in step with the receivers
        g_local_hop_hz = g_tx_prof_hz; g_local_hop = g_tx_prof_hop; g_local_hop_req = true;
    }
    g_local_pwr_req = g_tx_prof_pwr;
    tx_prof_print("saved");
}

// Small host records (not video): bytes needed in total, 0 = not known yet.
static unsigned host_small_len(const uint8_t* r, unsigned n) {
    switch (r[1]) {
    case HOST_PWR_SYNC: return 5u;
    case HOST_IDENT_SYNC: return 4u;
    case HOST_WHO_SYNC: return 4u;
    case HOST_CALL_SYNC: return n >= 3u ? 4u + r[2] : 0u;
    case HOST_PLAN_SYNC: return 10u;
    case HOST_STAT_SYNC: return 4u;
    default: return 0u;
    }
}
static bool host_sync2(int c) {
    return c == 0x5A || c == HOST_PWR_SYNC || c == HOST_CALL_SYNC || c == HOST_IDENT_SYNC || c == HOST_WHO_SYNC ||
           c == HOST_PLAN_SYNC || c == HOST_STAT_SYNC;
}
// Carries out a complete small record; false if it is malformed.
static bool host_small_record(const uint8_t* r) {
    if (r[1] == HOST_PWR_SYNC) {  // A5 C7 pwr ~pwr pwr^5A
        int8_t pw = (int8_t)r[2];
        if ((uint8_t)~r[2] != r[3] || (uint8_t)(r[2] ^ 0x5A) != r[4] || pw < VPWR_MIN || pw > VPWR_MAX) return false;
        g_local_pwr_req = pw;
        return true;
    }
    if (r[1] == HOST_WHO_SYNC) {  // A5 1F 01 FE: identity frame instead of the status frame for a moment
        if (r[2] != 1u || r[3] != 0xFEu) return false;
        g_who_until = time_us_64() + WHO_FRAME_US;
        host_status_publish();
        return true;
    }
    if (r[1] == HOST_STAT_SYNC) {  // A5 1B code ~code
        if ((uint8_t)~r[2] != r[3] || r[2] < 1 || r[2] > 15) return false;
        g_host_err = r[2];
        printf("C host error %u\n", r[2]);
        return true;
    }
    if (r[1] == HOST_IDENT_SYNC) {  // A5 1E m ~m
        if ((uint8_t)~r[2] != r[3] || r[2] < 1 || r[2] > 2) return false;
        if (r[2] > g_ident_req) g_ident_req = r[2];
        return true;
    }
    if (r[1] == HOST_PLAN_SYNC) {  // A5 1C hz[4 LE] n step_khz[2 LE] xor(r[2..8]) ^ 0x5A
        uint8_t x = 0x5A;
        for (unsigned i = 2; i < 9; i++) x ^= r[i];
        if (x != r[9]) return false;
        uint32_t hz = (uint32_t)r[2] | (uint32_t)r[3] << 8 | (uint32_t)r[4] << 16 | (uint32_t)r[5] << 24;
        hop_t h = {r[6], (uint16_t)(r[7] | r[8] << 8)};
        if (!h.n) h.step_khz = 0;
        if (!hop_valid(hz, &h)) return false;
        if (hz == g_freq_hz && hop_same(&h, &g_hop)) return true;  // already on it (every start sends it): nothing to do
        g_local_hop_hz = hz; g_local_hop = h; g_local_hop_req = true;
        return true;
    }
    if (r[1] == HOST_CALL_SYNC) {  // A5 1D n blob[n] xor
        unsigned n = r[2];
        uint8_t x = (uint8_t)(0x5A ^ n);
        for (unsigned i = 0; i < n; i++) x ^= r[3 + i];
        if (x != r[3 + n] || n > CALL_MAX || (n && blob_len(r + 3, n) != n)) return false;
        call_set(r + 3, n);
        return true;
    }
    return false;
}

#ifdef VLIVE_HOST_PIO
// SPI slave in PIO (vlive_tx_luckfox, pins in board.h). Mode 0: the master changes MOSI after SCK falls and
// samples MISO when SCK rises. Two state machines, each fed / drained by an endless DMA channel exactly like
// the SPI1 version below, so the rest of the firmware (record parser, status frame) is unchanged.
//   RX (MOSI -> ring):   0 wait 1 CS | 1 wait 0 CS | 2 mov isr,null | 3 wait 1 SCK | 4 in pins,1 | 5 wait 0 SCK | 6 jmp pin(CS) 1
//                        wrap 6 -> 3. Autopush every 8 bits (MSB first). A deselect (CS high) seen after a bit
//                        clears any partial byte, so every select starts on a byte boundary.
//   TX (status -> MISO): 0 out pins,1 | 1 wait 1 SCK | 2 wait 0 SCK | 3 out pins,1   wrap 3 -> 1. Autopull every 8 bits;
//                        the next bit is put on MISO after each falling edge. The host finds the frame by its sync.
// Byte alignment of the status direction: after every transfer, once CS has stayed high for HOST_IDLE_US, the TX
// state machine is restarted (shift register emptied; FIFO and DMA untouched), so a partial transfer can misalign
// the status at most until the next gap. Without this the status once stayed 2 bits off after an interrupted
// transfer, the host could no longer read it, and it handed over a record only every other slot. Only TX is
// restarted: a restart that coincides with the start of the next transfer costs at most one status read, whereas
// restarting RX cost records (13 in 8104 measured). RX alignment is guarded by host_check_resync() below.
static PIO host_pio = pio0;
static unsigned host_rx_sm, host_tx_sm, host_rx_off, host_tx_off;
static uint32_t host_junk;  // unexpected (non-zero, non-record) bytes since the last good record
static uint32_t host_resyncs;

static dma_channel_config host_rx_cfg, host_tx_cfg;

// (Re)starts both state machines and their DMA channels on a byte boundary. The DMA channels are stopped
// before the FIFOs are cleared and re-armed afterwards: a FIFO cleared under a running channel leaves the
// channel's DREQ credit out of step with the FIFO and it stalls. The RX ring keeps its current position.
static void host_pio_start(void) {
    pio_sm_set_enabled(host_pio, host_rx_sm, false);
    pio_sm_set_enabled(host_pio, host_tx_sm, false);
    dma_channel_abort(host_dma);
    dma_channel_abort(host_tx_dma);
    unsigned wr = (unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
    pio_sm_clear_fifos(host_pio, host_rx_sm);
    pio_sm_clear_fifos(host_pio, host_tx_sm);
    pio_sm_restart(host_pio, host_rx_sm);
    pio_sm_restart(host_pio, host_tx_sm);
    pio_sm_clkdiv_restart(host_pio, host_rx_sm);
    pio_sm_clkdiv_restart(host_pio, host_tx_sm);
    pio_sm_exec(host_pio, host_rx_sm, pio_encode_jmp(host_rx_off));
    pio_sm_exec(host_pio, host_tx_sm, pio_encode_jmp(host_tx_off));
    dma_channel_configure(host_dma, &host_rx_cfg, host_ring + wr, &host_pio->rxf[host_rx_sm], dma_encode_endless_transfer_count(), true);
    dma_channel_configure(host_tx_dma, &host_tx_cfg, &host_pio->txf[host_tx_sm], host_status, dma_encode_endless_transfer_count(), true);
    pio_enable_sm_mask_in_sync(host_pio, (1u << host_rx_sm) | (1u << host_tx_sm));
}

static void host_cs_rise(uint gpio, uint32_t events);

static void host_spi_init(void) {
    uint16_t rx[7], tx[4];
    rx[0] = pio_encode_wait_gpio(true, PIN_HOST_CS);
    rx[1] = pio_encode_wait_gpio(false, PIN_HOST_CS);
    rx[2] = pio_encode_mov(pio_isr, pio_null);
    rx[3] = pio_encode_wait_gpio(true, PIN_HOST_SCK);
    rx[4] = pio_encode_in(pio_pins, 1);
    rx[5] = pio_encode_wait_gpio(false, PIN_HOST_SCK);
    rx[6] = pio_encode_jmp_pin(1);  // relative to the program: fixed up below
    tx[0] = pio_encode_out(pio_pins, 1);
    tx[1] = pio_encode_wait_gpio(true, PIN_HOST_SCK);
    tx[2] = pio_encode_wait_gpio(false, PIN_HOST_SCK);
    tx[3] = pio_encode_out(pio_pins, 1);
    const pio_program_t rxp = {.instructions = rx, .length = 7, .origin = -1};
    const pio_program_t txp = {.instructions = tx, .length = 4, .origin = -1};
    host_rx_off = pio_add_program(host_pio, &rxp);  // pio_add_program relocates JMP targets by the offset
    host_tx_off = pio_add_program(host_pio, &txp);
    host_rx_sm = pio_claim_unused_sm(host_pio, true);
    host_tx_sm = pio_claim_unused_sm(host_pio, true);

    pio_gpio_init(host_pio, PIN_HOST_SCK);
    pio_gpio_init(host_pio, PIN_HOST_RX);
    pio_gpio_init(host_pio, PIN_HOST_CS);
    pio_gpio_init(host_pio, PIN_HOST_TX);
    gpio_pull_up(PIN_HOST_CS);     // no host attached = deselected
    gpio_pull_down(PIN_HOST_SCK);  // mode 0 idles low
    gpio_pull_down(PIN_HOST_RX);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, PIN_HOST_RX);
    sm_config_set_jmp_pin(&c, PIN_HOST_CS);
    sm_config_set_in_shift(&c, false, true, 8);  // shift left (MSB first), autopush at 8 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_wrap(&c, host_rx_off + 3, host_rx_off + 6);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(host_pio, host_rx_sm, host_rx_off, &c);
    pio_sm_set_consecutive_pindirs(host_pio, host_rx_sm, PIN_HOST_RX, 1, false);

    pio_sm_config t = pio_get_default_sm_config();
    sm_config_set_out_pins(&t, PIN_HOST_TX, 1);
    sm_config_set_out_shift(&t, false, true, 8);  // MSB first; an 8-bit DMA write fills all byte lanes
    sm_config_set_fifo_join(&t, PIO_FIFO_JOIN_TX);
    sm_config_set_wrap(&t, host_tx_off + 1, host_tx_off + 3);
    sm_config_set_clkdiv_int_frac(&t, 1, 0);
    pio_sm_init(host_pio, host_tx_sm, host_tx_off, &t);
    pio_sm_set_consecutive_pindirs(host_pio, host_tx_sm, PIN_HOST_TX, 1, true);

    host_dma = dma_claim_unused_channel(true);
    host_rx_cfg = dma_channel_get_default_config(host_dma);
    channel_config_set_transfer_data_size(&host_rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&host_rx_cfg, false);
    channel_config_set_write_increment(&host_rx_cfg, true);
    channel_config_set_ring(&host_rx_cfg, true, HOST_RING_BITS);  // wrap the write address
    channel_config_set_dreq(&host_rx_cfg, pio_get_dreq(host_pio, host_rx_sm, false));
    dma_channel_set_write_addr(host_dma, host_ring, false);
    host_tx_dma = dma_claim_unused_channel(true);
    host_tx_cfg = dma_channel_get_default_config(host_tx_dma);
    channel_config_set_transfer_data_size(&host_tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&host_tx_cfg, true);
    channel_config_set_write_increment(&host_tx_cfg, false);
    channel_config_set_ring(&host_tx_cfg, false, 5);  // wrap the read address inside the 32-byte frame
    channel_config_set_dreq(&host_tx_cfg, pio_get_dreq(host_pio, host_tx_sm, true));
    host_status_publish();
    host_pio_start();
    gpio_set_irq_enabled_with_callback(PIN_HOST_CS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, host_cs_rise);
}

#define HOST_IDLE_US 20u
static volatile uint32_t host_cs_rise_t;
static volatile bool host_cs_pending;  // CS rose and the state machines have not been realigned since

static void host_cs_rise(uint gpio, uint32_t events) {
    if (gpio != PIN_HOST_CS) return;
    if (events & GPIO_IRQ_EDGE_FALL) host_cs_pending = false;
    if (events & GPIO_IRQ_EDGE_RISE) { host_cs_rise_t = time_us_32(); host_cs_pending = true; }
}

// Called from poll_host(): realigns once CS has been high for HOST_IDLE_US.
static void host_realign_if_idle(void) {
    if (!host_cs_pending || time_us_32() - host_cs_rise_t < HOST_IDLE_US) return;
    uint32_t irq = save_and_disable_interrupts();
    if (!host_cs_pending || !gpio_get(PIN_HOST_CS)) { restore_interrupts(irq); return; }  // a transfer began
    host_cs_pending = false;
    pio_sm_set_enabled(host_pio, host_tx_sm, false);
    pio_sm_restart(host_pio, host_tx_sm);
    pio_sm_exec(host_pio, host_tx_sm, pio_encode_jmp(host_tx_off));
    pio_sm_set_enabled(host_pio, host_tx_sm, true);
    restore_interrupts(irq);
}

// Bit-slip guard. With whole-byte transfers the state machines never lose byte alignment; if they ever did
// (a glitch on SCK while the host was booting), records would stop parsing and only junk would arrive. Then,
// while the host is deselected, both state machines are restarted on a byte boundary.
static void host_check_resync(void) {
    if (host_junk < 32u || !gpio_get(PIN_HOST_CS)) return;
    host_pio_start();
    // Everything not parsed yet was received misaligned: drop it.
    host_rd = (unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
    host_junk = 0;
    host_resyncs++;
    rec_n = 0;
    printf("H resync %lu\n", (unsigned long)host_resyncs);
}
#else
static void host_spi_init(void) {
    spi_init(HOST_SPI, 1000000);  // baud is irrelevant in slave mode (clock comes from the Pi)
    spi_set_format(HOST_SPI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(HOST_SPI, true);
    gpio_set_function(PIN_HOST_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_HOST_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_HOST_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_HOST_CS, GPIO_FUNC_SPI);
    host_dma = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(host_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, HOST_RING_BITS);  // wrap the write address
    channel_config_set_dreq(&c, spi_get_dreq(HOST_SPI, false));
    dma_channel_configure(host_dma, &c, host_ring, &spi_get_hw(HOST_SPI)->dr, dma_encode_endless_transfer_count(), true);
    host_status_publish();
    host_tx_dma = dma_claim_unused_channel(true);
    dma_channel_config t = dma_channel_get_default_config(host_tx_dma);
    channel_config_set_transfer_data_size(&t, DMA_SIZE_8);
    channel_config_set_read_increment(&t, true);
    channel_config_set_write_increment(&t, false);
    channel_config_set_ring(&t, false, 5);  // wrap the read address inside the 32-byte frame
    channel_config_set_dreq(&t, spi_get_dreq(HOST_SPI, true));
    dma_channel_configure(host_tx_dma, &t, &spi_get_hw(HOST_SPI)->dr, host_status, dma_encode_endless_transfer_count(), true);
}
#endif

// Identity frame (see "Identity" at the top), shown instead of the status frame until g_who_until.
static void who_frame(uint8_t* f) {
    memset(f, 0, STATUS_LEN);
    f[0] = 0xC3; f[1] = 0x49;
    f[2] = HZ_ID_VERSION; f[3] = (uint8_t)HZ_BOARD; f[4] = HZ_OTA_FORMAT;
    uint32_t h = 0;
    const char* g = HZ_FW_GIT;
    for (unsigned i = 0; i < 8 && hexval(g[i]) >= 0; i++) h = h << 4 | (uint32_t)hexval(g[i]);
    f[5] = (uint8_t)h; f[6] = (uint8_t)(h >> 8); f[7] = (uint8_t)(h >> 16); f[8] = (uint8_t)(h >> 24);
    f[9] = strstr(g, "dirty") ? 1u : 0u;
#ifdef LIVE_TX
    if (g_call_len) f[9] |= 2u;      // bit 1: the TX has a callsign (without one it stays silent)
    if (g_call_default) f[9] |= 4u;  // bit 2: it is the N0CALL default (none stored)
#endif
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    memcpy(f + 10, uid.id, 8);
    const char* name = HZ_FW_TARGET;  // without the common "vlive_" prefix (12 characters fit)
    if (!strncmp(name, "vlive_", 6)) name += 6;
    strncpy((char*)f + 18, name, 12);
    // 30: radio state (bits 0..1, see g_radio_state) | last radio_init() error code (bits 2..7, absolute value)
    f[30] = (uint8_t)((g_radio_state & 3u) | (((unsigned)(g_radio_rc < 0 ? -g_radio_rc : g_radio_rc) & 0x3Fu) << 2));
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 30; i++) x ^= f[i];
    f[31] = x;
}

static void host_status_publish(void) {
    if (g_who_until) {
        if (time_us_64() < g_who_until) {
            uint8_t w[STATUS_LEN];
            who_frame(w);
            memcpy(host_status, w, STATUS_LEN);
            return;
        }
        g_who_until = 0;
    }
    uint8_t f[STATUS_LEN] = {0};
    uint32_t acc = rec_ok, hz = g_freq_hz;
    f[0] = 0xC3; f[1] = 0x3C;
    f[2] = (uint8_t)(lq_head - lq_tail); f[3] = ev_count;
    f[4] = (uint8_t)acc; f[5] = (uint8_t)(acc >> 8); f[6] = (uint8_t)(acc >> 16); f[7] = (uint8_t)(acc >> 24);
    f[8] = (uint8_t)tot_sent; f[9] = (uint8_t)(tot_sent >> 8);
    f[10] = (uint8_t)tot_fail; f[11] = (uint8_t)(tot_fail >> 8);
    f[12] = (uint8_t)tot_late; f[13] = (uint8_t)(tot_late >> 8);
    f[14] = (uint8_t)tot_data; f[15] = (uint8_t)(tot_data >> 8);
    f[16] = (uint8_t)tot_idle; f[17] = (uint8_t)(tot_idle >> 8);
    f[18] = (uint8_t)rec_bad; f[19] = (uint8_t)pwr_out(g_txpwr);
    f[20] = (uint8_t)hz; f[21] = (uint8_t)(hz >> 8); f[22] = (uint8_t)(hz >> 16); f[23] = (uint8_t)(hz >> 24);
    f[24] = (uint8_t)g_txseq; f[25] = (uint8_t)(g_txseq >> 8);
    f[26] = cmd_slots; f[27] = cmd_irqs; f[28] = (uint8_t)((cmd_flags & 0x7Fu) | (g_ident_toggle ? 0x80u : 0u));
    f[29] = (uint8_t)cmd_arr; f[30] = (uint8_t)(cmd_arr >> 8);
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 30; i++) x ^= f[i];
    f[31] = x;
    memcpy(host_status, f, STATUS_LEN);
}

// Status LED (PIN_LED, the red LED by the RP2354A): what the radio controller is doing, readable without a laptop.
//   on            sending video from the Luckfox
//   slow blink    radio running, no video from the Luckfox (1 s on, 1 s off): the Luckfox is starting, or its
//                 camera or transmitter program is down (its own LED says which)
//   double blink  silent: no callsign set, or stopped by the Luckfox (two short flashes every 2 s)
//   fast blink    the radio module does not start (5 per second)
//   off           no power, or the RP2354A is not running
// Called from poll_host(), which runs many times per slot and while the radio start is retried.
#ifdef PIN_LED
static void led_tick(void) {
    static uint32_t last_ok, last_video_ms;
    uint32_t ms = time_us_32() / 1000u;
    if (rec_ok != last_ok) { last_ok = rec_ok; last_video_ms = ms; }
    bool on;
    if (g_radio_state == 2) on = (ms / 100u) & 1u;
    else if (!g_call_len || g_quiet) { uint32_t p = ms % 2000u; on = p < 150u || (p >= 300u && p < 450u); }
    else if (g_radio_state == 1 && ms - last_video_ms < 1000u) on = true;
    else on = ms % 2000u < 1000u;
    gpio_put(PIN_LED, on);
}
#else
static void led_tick(void) {}
#endif

static void poll_host(void) {
    led_tick();
    // the identity window ends on time even when nothing else publishes a status (transmitter stopped or silent):
    // otherwise the host kept reading the identity frame for minutes
    if (g_who_until && time_us_64() >= g_who_until) host_status_publish();
    unsigned wr =(unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
    int avail = (int)((wr - host_rd) & (HOST_RING_SIZE - 1));
    if (avail > 64) avail = 64;  // bound the time spent per call
    if (avail > 0) g_host_seen_us = time_us_64();
    g_host_bytes += (uint32_t)avail;
    uint32_t ok0 = rec_ok;
    // Bytes that are not part of a record (the Pi's status-poll zeros) are always consumed. A record start
    // is only taken when the queue has room for it, otherwise it stays in the ring and the sender (paced from
    // the status frame) is pushed back.
    for (int k = 0; k < avail; k++) {
        int c = host_ring[host_rd];
        if (rec_n == 0) {
            if (c == 0xA5) { if (lq_head - lq_tail >= LQ_CAP) break; rec[rec_n++] = 0xA5; }
#ifdef VLIVE_HOST_PIO
            else if (c != 0) host_junk++;
#endif
            host_rd = (host_rd + 1) & (HOST_RING_SIZE - 1);
            continue;
        }
        host_rd = (host_rd + 1) & (HOST_RING_SIZE - 1);
        if (rec_n == 1) {
            if (host_sync2(c)) rec[rec_n++] = (uint8_t)c;
            else if (c != 0xA5) rec_n = 0;
            continue;
        }
        rec[rec_n++] = (uint8_t)c;
        if (rec[1] != 0x5A) {  // power / callsign / identify record
            unsigned need = host_small_len(rec, rec_n);
            if (rec[1] == HOST_CALL_SYNC && rec_n >= 3 && rec[2] > CALL_MAX) { rec_n = 0; rec_bad++; continue; }
            if (!need || rec_n < need) continue;
            rec_n = 0;
            if (host_small_record(rec)) {
#ifdef VLIVE_HOST_PIO
                host_junk = 0;
#endif
            } else {
                rec_bad++;
            }
            continue;
        }
        if (rec_n < REC_LEN) continue;
        rec_n = 0;
        uint8_t x = 0;
        for (unsigned i = 2; i < 2 + ENTRY; i++) x ^= rec[i];
        if (x != rec[REC_LEN - 1]) {
            rec_bad++;
#ifdef VLIVE_HOST_PIO
            host_junk += 32u;
#endif
            continue;
        }
        memcpy(lq[lq_head % LQ_CAP], rec + 2, ENTRY);
        lq_head++;
        rec_ok++;
        g_host_rec_us = time_us_64(); g_host_err = 0;
#ifdef VLIVE_HOST_PIO
        host_junk = 0;
#endif
    }
    if (rec_ok != ok0) host_status_publish();
#ifdef VLIVE_HOST_PIO
    host_check_resync();
    host_realign_if_idle();
#else
    usb_host_poll_throttled();
#endif
}

#define HOST_QUIET_US 100u  // no USB work this close to a packet start: start time stays exact


// TX USB text: "I" identify now, "I <hex blob>" set the callsign (blob as swfec.py / hz_vtx encode it), "I -" clear it.
static void tx_call_text(const char* line) {
    const char* h = line + 1;
    while (*h == ' ') h++;
    if (!*h) { if (g_ident_req < 1) g_ident_req = 1; printf("C ident requested\n"); return; }
    if (*h == '-') { uint8_t z = 0; call_set(&z, 0); return; }
    uint8_t b[CALL_MAX];
    unsigned n = 0;
    while (h[0] && h[1] && n < CALL_MAX && hexval(h[0]) >= 0 && hexval(h[1]) >= 0) { b[n++] = (uint8_t)(hexval(h[0]) << 4 | hexval(h[1])); h += 2; }
    if (!n || *h || blob_len(b, n) != n) { printf("C fail call (I <hex blob>, as swfec.py encodes it)\n"); return; }
    call_set(b, n);
}

#ifdef VLIVE_HOST_PIO
// TX USB port command (when the TX RP2350 is plugged into a computer): "P <half_dbm>" sets the power locally.
// Non-blocking; called once per slot while a packet is on air.
static void tx_usb_poll(void) {
    static char buf[256];
    static unsigned n;
    uint8_t tmp[16];
    int got = stdio_get_until((char*)tmp, sizeof tmp, get_absolute_time());
    for (int k = 0; k < got; k++) {
        if (tmp[k] == '\n' || tmp[k] == '\r') {
            long v;
            buf[n] = 0;
            if (n && buf[0] == 'P' && sscanf(buf + 1, "%ld", &v) == 1) {
                if (v >= VPWR_MIN && v <= VPWR_MAX) g_local_pwr_req = (int16_t)v;
                else printf("C fail range pwr=%ld\n", v);
            } else if (n && buf[0] == 'H') {
                tx_hop_text(buf);
            } else if (n && buf[0] == 'I') {
                tx_call_text(buf);
            } else if (n && buf[0] == 'W') {
                who_print();
            } else if (n && buf[0] == 'K') {
                tx_prof_command(buf + 1);
            } else if (n) {
                printf("C fail bad command (TX accepts P <half_dbm>, H <centre_hz> <n> <spacing_hz>, I [hex], W, K [profile])\n");
            }
            n = 0;
        } else if (n < sizeof buf - 1) {
            buf[n++] = (char)tmp[k];
        } else {
            n = 0;
        }
    }
}
#else
// USB host link (vlive_tx, the Pi build): the TX RP2350's USB serial port carries the same records as SPI, so the Pi
// sender (pi5_vtx/live_sender.py --link usb) can drive it over a plain USB cable:
//   host -> RP2350  video record A5 5A entry[126] xor8, power record A5 C7 pwr ~pwr pwr^5A (as on SPI), and text lines
//                   "P <half_dbm>" (TX power), "S 1" / "S 0" (status lines on / off)
//   RP2350 -> host  with status lines on: "S <status frame, 32 bytes as 64 hex digits>" at every slot start (the frame
//                   the SPI host reads on MISO; it replaces the "Q" line), plus the usual T / C text lines.
// Bytes are only taken from USB while the queue has room for a record, so USB flow control holds a fast host back
// (as the status frame paces the SPI host). SPI keeps working unchanged; use one of the two at a time.
static uint8_t urec[REC_LEN];
static unsigned urec_n;
static char ubuf[256];
static unsigned ubuf_n;
static uint8_t uin[64];
static int uin_n, uin_i;
static bool g_usb_status;
static uint32_t usb_poll_t;

static void tx_usb_text(int c) {
    if (c == '\n' || c == '\r') {
        long v;
        ubuf[ubuf_n] = 0;
        if (ubuf_n && ubuf[0] == 'P' && sscanf(ubuf + 1, "%ld", &v) == 1) {
            if (v >= VPWR_MIN && v <= VPWR_MAX) g_local_pwr_req = (int16_t)v;
            else printf("C fail range pwr=%ld\n", v);
        } else if (ubuf_n && ubuf[0] == 'S' && sscanf(ubuf + 1, "%ld", &v) == 1) {
            g_usb_status = v != 0;
        } else if (ubuf_n && ubuf[0] == 'H') {
            tx_hop_text(ubuf);
        } else if (ubuf_n && ubuf[0] == 'I') {
            tx_call_text(ubuf);
        } else if (ubuf_n && ubuf[0] == 'W') {
            who_print();
        } else if (ubuf_n && ubuf[0] == 'K') {
            tx_prof_command(ubuf + 1);
        } else if (ubuf_n) {
            printf("C fail bad command (TX accepts P <half_dbm>, H <centre_hz> <n> <spacing_hz>, I [hex], W, K [profile], S 0|1 and binary records)\n");
        }
        ubuf_n = 0;
    } else if (ubuf_n < sizeof ubuf - 1) {
        ubuf[ubuf_n++] = (char)c;
    } else {
        ubuf_n = 0;
    }
}

static void tx_usb_record(void) {
    uint8_t x = 0;
    for (unsigned i = 2; i < 2 + ENTRY; i++) x ^= urec[i];
    if (x != urec[REC_LEN - 1]) { rec_bad++; return; }
    memcpy(lq[lq_head % LQ_CAP], urec + 2, ENTRY);
    lq_head++;
    rec_ok++;
    g_host_rec_us = g_host_seen_us = time_us_64(); g_host_err = 0;
    host_status_publish();
}

// Non-blocking; called once per slot while a packet is on air, and from poll_host() (throttled).
static void tx_usb_poll(void) {
    for (;;) {
        if (uin_i >= uin_n) {
            if (urec_n == 0 && lq_head - lq_tail >= LQ_CAP) return;  // queue full: leave the bytes in the USB FIFO
            int got = stdio_get_until((char*)uin, sizeof uin, get_absolute_time());
            if (got <= 0) return;
            uin_n = got; uin_i = 0;
        }
        while (uin_i < uin_n) {
            int c = uin[uin_i];
            if (urec_n == 0) {
                if (c == 0xA5) {
                    if (lq_head - lq_tail >= LQ_CAP) return;  // a record starts but there is no room: wait
                    urec[urec_n++] = 0xA5;
                } else {
                    tx_usb_text(c);
                }
                uin_i++;
                continue;
            }
            uin_i++;
            if (urec_n == 1) {
                if (host_sync2(c)) urec[urec_n++] = (uint8_t)c;
                else if (c != 0xA5) { urec_n = 0; tx_usb_text(c); }
                continue;
            }
            urec[urec_n++] = (uint8_t)c;
            if (urec[1] != 0x5A) {  // power / callsign / identify record (as on SPI)
                unsigned need = host_small_len(urec, urec_n);
                if (urec[1] == HOST_CALL_SYNC && urec_n >= 3 && urec[2] > CALL_MAX) { urec_n = 0; rec_bad++; continue; }
                if (!need || urec_n < need) continue;
                urec_n = 0;
                if (!host_small_record(urec)) rec_bad++;
                continue;
            }
            if (urec_n < REC_LEN) continue;
            urec_n = 0;
            tx_usb_record();
        }
    }
}

static void usb_host_poll_throttled(void) {
    uint32_t now = time_us_32();
    if (now - usb_poll_t < 150u) return;
    usb_poll_t = now;
    tx_usb_poll();
}

// Slot start report on USB: the status frame as hex when the USB host asked for it, else the "Q" line.
static void tx_usb_slot_report(void) {
    if (g_usb_status && !stdio_usb_connected()) g_usb_status = false;  // host went away: back to Q lines
    if (!g_usb_status) {
        printf("Q %lu %lu\n", (unsigned long)(lq_head - lq_tail), (unsigned long)rec_ok);
        return;
    }
    static const char hx[] = "0123456789abcdef";
    char line[2 + 2 * STATUS_LEN + 2];
    line[0] = 'S'; line[1] = ' ';
    for (unsigned i = 0; i < STATUS_LEN; i++) {
        line[2 + 2 * i] = hx[host_status[i] >> 4];
        line[3 + 2 * i] = hx[host_status[i] & 15];
    }
    line[2 + 2 * STATUS_LEN] = '\n';
    fwrite(line, 1, 3 + 2 * STATUS_LEN, stdout);
    fflush(stdout);
}
#endif

// Slot start: status frame (seq) for the host, and the Q / S line on USB.
static void slot_report(uint32_t seq) {
#ifdef VLIVE_HOST_PIO
    printf("Q %lu %lu\n", (unsigned long)(lq_head - lq_tail), (unsigned long)rec_ok);
    g_txseq = seq; host_status_publish();
#else
    g_txseq = seq; host_status_publish();
    tx_usb_slot_report();
#endif
}

// One ident packet (part `part` of the callsign) in the command slot that starts at t.
static void tx_ident(uint64_t t, uint16_t s16, unsigned part) {
    uint8_t p[CMD_LEN];
    ident_encode(p, s16, g_call, g_call_len, part);
    radio_set_payload_len(CMD_LEN);
    lr20xx_radio_fifo_clear_tx(radio);
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    lr20xx_radio_fifo_write_tx(radio, p, CMD_LEN);
    while (time_us_64() + HOST_QUIET_US < t + IDENT_DELAY_US) poll_host();
    while (time_us_64() < t + IDENT_DELAY_US) tight_loop_contents();
    uint32_t t0 = time_us_32();
    lr20xx_radio_common_set_tx(radio, 0);
    uint32_t irq = 0;
    for (;;) {
        if (radio_hal_irq_pin()) { irq = radio_hal_read_clear_irq_fast(); break; }
        if (time_us_32() - t0 > 10000u) { lr20xx_system_get_and_clear_irq_status(radio, &irq); break; }
        poll_host();
    }
    if (!(irq & LR20XX_SYSTEM_IRQ_TX_DONE)) recover();
    lr20xx_system_set_fs_mode(radio);
    radio_set_payload_len(PAYLOAD_LEN);
}

// The Air status (see "Air status") into an idle entry e (FF FF FF already set).
static void air_status(uint8_t* e, bool stopping) {
    uint64_t now = time_us_64();
    uint64_t rec = g_host_rec_us ? (now - g_host_rec_us) / 1000000u : 0xFFFFu;
    uint64_t seen = g_host_seen_us ? (now - g_host_seen_us) / 1000000u : 0xFFFFu;
    if (rec > 0xFFFEu && g_host_rec_us) rec = 0xFFFEu;
    if (seen > 0xFFFEu && g_host_seen_us) seen = 0xFFFEu;
    uint8_t st = stopping ? AIR_ST_STOPPING : g_host_err ? AIR_ST_ERROR
               : (g_host_rec_us && now - g_host_rec_us < 1000000u) ? AIR_ST_VIDEO
               : (g_host_seen_us && now - g_host_seen_us < 1000000u) ? AIR_ST_NO_VIDEO
               : (g_host_seen_us || g_host_rec_us) ? AIR_ST_SILENT : AIR_ST_NO_HOST;
    uint32_t up = (uint32_t)(now / 1000000u), h = 0;
    const char* g = HZ_FW_GIT;
    for (unsigned i = 0; i < 8 && hexval(g[i]) >= 0; i++) h = h << 4 | (uint32_t)hexval(g[i]);
    e[3] = 0x48; e[4] = 0x53; e[5] = 1u; e[6] = st;
    e[7] = (uint8_t)rec; e[8] = (uint8_t)(rec >> 8); e[9] = (uint8_t)seen; e[10] = (uint8_t)(seen >> 8);
    e[11] = g_host_err;
    e[12] = (uint8_t)up; e[13] = (uint8_t)(up >> 8); e[14] = (uint8_t)(up >> 16); e[15] = (uint8_t)(up >> 24);
    e[16] = (uint8_t)h; e[17] = (uint8_t)(h >> 8); e[18] = (uint8_t)(h >> 16); e[19] = (uint8_t)(h >> 24);
    uint8_t x = 0x5A;
    for (unsigned i = 3; i < 20; i++) x ^= e[i];
    e[20] = x;
    e[21] = (uint8_t)g_link; e[22] = (uint8_t)(g_link >> 8);
}

static void run_live_tx(void) {
    printf("vlive TX id=%s freq_hz=%lu slot_us=%u\n", g_id, (unsigned long)g_freq_hz, SLOT_US);
    call_load();
    call_link();
    tx_prof_load();
    if (g_call_default) call_print("default N0CALL (none stored; set one with the TX host's --callsign / Pair)");
    else if (g_call_len) call_print("loaded");
    else printf("C call cleared: silent until a callsign is set (the TX host's --callsign, the setup site's Pair)\n");
    bool silent_no_call = false;  // reported once per change
    g_ident_next = time_us_64() + IDENT_FIRST_US;
    bool ident_active = false, ident_quiet = false;
    unsigned ident_part = 0;
    uint8_t pkt[PAYLOAD_LEN];
    uint32_t seq = 0, sent = 0, fail = 0, late = 0, data = 0, idle = 0, last_t = time_us_32();
    uint32_t air_min = 0xFFFFFFFFu, air_max = 0;  // measured SetTx -> TX_DONE, reported in the T line
    int8_t pwr = TX_POWER_HALF_DBM;
    host_status_publish();
    vcmd_t act = {0};
    bool act_valid = false;  // a received command is being acknowledged / waiting to take effect
    uint16_t ack_first = 0;
    unsigned ann_left = CMD_ACKS;  // power announcements still to send (boot: tell the receivers our power)
    lr20xx_system_set_fs_mode(radio);  // keep TCXO and PLL running between packets
    radio_set_freq(g_freq_hz);
    uint32_t cur_hz = g_freq_hz;       // where the synthesizer is (hopping: retuned between packets)
    uint32_t last_plan_ann = 0;
    uint64_t t = time_us_64() + 200000;
    for (;;) {
        uint16_t s16 = (uint16_t)seq;
        if (g_call_save) {  // new callsign: into flash (~50 ms, interrupts off); the slots it takes are skipped below
            g_call_save = false;
            call_store();
            uint64_t now = time_us_64();
            while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
            continue;
        }
        if (g_tx_prof_save) {  // new pairing profile: the same, then it is announced and applied (below)
            g_tx_prof_save = false;
            tx_prof_store();
            uint64_t now = time_us_64();
            while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
            continue;
        }
        if (g_ident_req) {  // the host asks for an identification (2: then stop transmitting: end of transmission)
            if (g_ident_req == 2) ident_quiet = true;
            g_ident_req = 0;
            if (!ident_active) { ident_active = true; ident_part = 0; }
        }
        if (!ident_active && g_call_len && !g_quiet && time_us_64() >= g_ident_next) { ident_active = true; ident_part = 0; }
        bool no_call = !g_call_len;  // no callsign: never transmit (no transmission without identification)
        if (no_call != silent_no_call) {
            silent_no_call = no_call;
            printf(no_call ? "C silent: no callsign set\n" : "C callsign set: transmitting\n");
            host_status_publish();
        }
        if (no_call) {
            lq_tail = lq_head;  // video has nowhere to go
            if (ident_active) {  // an identification the host waits for (end of transmission): nothing to send
                ident_active = false;
                g_ident_toggle ^= 1u;
                if (ident_quiet) { ident_quiet = false; g_quiet = true; }
                host_status_publish();
            }
        }
        if (g_quiet || no_call) {  // end of transmission / no callsign: silent (slots still counted)
            if (!no_call && lq_head != lq_tail) {
                g_quiet = false;
                g_ident_next = time_us_64() + IDENT_FIRST_US;
                printf("C transmitting again\n");
            } else {
                // silent: nothing goes on air, so a host's plan / power request (hz_vtx sends them at every start,
                // before its video) is applied at once; receivers find the TX on it when it transmits again
                if (g_local_hop_req) {
                    g_local_hop_req = false;
                    hop_set(&g_local_hop);
                    g_freq_hz = g_local_hop_hz;
                    bool ok = hop_retune();
                    cur_hz = g_freq_hz;
                    ann_left = CMD_ACKS; last_plan_ann = seq;
                    printf("C applied hop freq_hz=%lu n=%u spacing_hz=%lu pwr=%d cal=%d (while silent)\n",
                           (unsigned long)g_freq_hz, g_hop.n, (unsigned long)g_hop.step_khz * 1000u, pwr, ok);
                    ev_count++; host_status_publish();
                }
                if (g_local_pwr_req != INT16_MIN) {
                    int8_t p = (int8_t)g_local_pwr_req;
                    g_local_pwr_req = INT16_MIN;
                    if (radio_set_power(p)) { pwr = p; g_txpwr = p; ann_left = CMD_ACKS; printf("C local pwr=%d (while silent)\n", p); }
                    ev_count++; host_status_publish();
                }
                while (time_us_64() < t) poll_host();
                slot_report(seq);
                tx_usb_poll();
                seq++; t += SLOT_US;
                continue;
            }
        }
        if (g_local_pwr_req != INT16_MIN) {
            // TX-side power change (host record A5 C7 / USB "P"): applied between two packets, reported in the status
            // frame (event counter + power) and announced downlink, so the receivers show the new value
            int8_t p = (int8_t)g_local_pwr_req;
            g_local_pwr_req = INT16_MIN;
            if (radio_set_power(p)) {
                pwr = p; g_txpwr = p; ann_left = CMD_ACKS;
                printf("C local pwr=%d\n", p);
            } else {
                printf("C fail pwr=%d (radio)\n", p);
            }
            ev_count++; host_status_publish();
        }
        if (g_local_hop_req && !act_valid) {
            // TX-side hop plan change (USB): announced CMD_ACKS times ahead in the ACK format (id 0xFF, the agreed slot
            // in it), then applied like a receiver's command, so every receiver that hears it switches in the same slot
            g_local_hop_req = false;
            act = (vcmd_t){.id = 0xFF, .flags = CMD_F_HOP, .hz = g_local_hop_hz, .pwr = 0, .eff = (uint16_t)(s16 + CMD_EFF),
                           .hn = g_local_hop.n, .hstep = g_local_hop.step_khz};
            act_valid = true;
            ack_first = s16;
            printf("C local hop freq_hz=%lu n=%u spacing_hz=%lu eff=%u\n", (unsigned long)act.hz, act.hn,
                   (unsigned long)act.hstep * 1000u, act.eff);
        }
        if (act_valid && sdiff16(s16, act.eff) >= 0) {  // the agreed slot: apply the change
            if (act.flags & CMD_F_PWR) { radio_set_power(act.pwr); pwr = act.pwr; g_txpwr = pwr; ann_left = CMD_ACKS; }
            if (act.flags == CMD_F_CALL) {
                call_stage_apply(&act);
                ev_count++; host_status_publish();
            } else if (act.flags & CMD_F_HOP) {
                hop_t h = {act.hn, act.hstep};
                hop_set(&h);
                g_freq_hz = act.hz;
                bool ok = hop_retune();
                cur_hz = g_freq_hz;
                ann_left = CMD_ACKS; last_plan_ann = seq;
                int16_t d = sdiff16((uint16_t)(act.eff + CMD_QUIET), s16);
                if (d > 0) { seq += (uint32_t)d; t += (uint64_t)d * SLOT_US; }
                printf("C applied hop freq_hz=%lu n=%u spacing_hz=%lu pwr=%d cal=%d\n", (unsigned long)g_freq_hz,
                       g_hop.n, (unsigned long)g_hop.step_khz * 1000u, pwr, ok);
                ev_count++; host_status_publish();
            } else if ((act.flags & CMD_F_FREQ) && act.hz != g_freq_hz) {
                if (g_hop.n && !hop_valid(act.hz, &g_hop)) { hop_t f = {0}; hop_set(&f); }  // (the RX checks this first)
                bool ok = g_hop.n ? (g_freq_hz = act.hz, hop_retune()) : radio_retune(act.hz);
                g_freq_hz = act.hz;
                cur_hz = g_freq_hz;
                int16_t d = sdiff16((uint16_t)(act.eff + CMD_QUIET), s16);
                if (d > 0) { seq += (uint32_t)d; t += (uint64_t)d * SLOT_US; }
                printf("C applied freq_hz=%lu pwr=%d cal=%d\n", (unsigned long)g_freq_hz, pwr, ok);
                ev_count++; host_status_publish();
            } else {
                printf("C applied freq_hz=%lu pwr=%d\n", (unsigned long)g_freq_hz, pwr);
                ev_count++; host_status_publish();
            }
            act_valid = false;
            uint64_t now = time_us_64();
            while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
            continue;
        }
        if ((s16 % CMD_PERIOD) == 0 && ident_active && !act_valid) {  // command slot used for the identification
            if (chan_hz(seq) != cur_hz) { cur_hz = chan_hz(seq); radio_set_freq(cur_hz); }  // (hopping only)
            slot_report(seq);
            unsigned parts = g_call_len ? (g_call_len + IDENT_PART - 1u) / IDENT_PART : 1u;
            if (g_call_len) tx_ident(t, s16, ident_part);
            if (++ident_part >= parts) {
                ident_active = false;
                g_ident_next = time_us_64() + IDENT_PERIOD_US;
                g_ident_toggle ^= 1u;
                g_idents++;
                if (ident_quiet) {
                    ident_quiet = false;
                    g_quiet = true;
                    lq_tail = lq_head;  // (the host stopped: anything left belongs to no complete frame)
                    printf("C end of transmission (%s), silent until the next video record\n",
                           g_call_len ? "identified" : "no callsign set");
                }
                host_status_publish();
            }
            seq++; t += SLOT_US;
            uint64_t now = time_us_64();
            while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
            continue;
        }
        if ((s16 % CMD_PERIOD) == 0) {  // command slot: listen instead of sending
            if (chan_hz(seq) != cur_hz) { cur_hz = chan_hz(seq); radio_set_freq(cur_hz); }  // (hopping only)
            lr20xx_radio_fifo_clear_rx(radio);
            lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
            radio_set_payload_len(CMD_LEN);
            lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
            slot_report(seq);
            uint32_t irq = 0;
            bool got = false;
            cmd_slots++;
            while (time_us_64() + 700 < t + SLOT_US) {
                if (radio_hal_irq_pin()) {
                    cmd_arr = (uint16_t)((time_us_64() - (t - 2000)) >> 4);
                    irq = radio_hal_read_clear_irq_fast();
                    got = true;
                    break;
                }
                poll_host();
            }
            if (!got) lr20xx_system_set_fs_mode(radio);  // nothing came: leave RX
            else {
                cmd_irqs++;
                cmd_flags = (irq & LR20XX_SYSTEM_IRQ_RX_DONE ? 1 : 0) | (irq & LR20XX_SYSTEM_IRQ_CRC_ERROR ? 2 : 0) |
                            (irq & LR20XX_SYSTEM_IRQ_LEN_ERROR ? 4 : 0) |
                            (irq & ~(LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR) ? 8 : 0);
            }
            if (got && (irq & LR20XX_SYSTEM_IRQ_RX_DONE) && !(irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR))) {
                uint16_t level = 0;
                lr20xx_radio_fifo_get_rx_level(radio, &level);
                vcmd_t c;
                if (level != CMD_LEN) cmd_flags |= 16;
                if (level == CMD_LEN) {
                    lr20xx_radio_fifo_read_rx(radio, pkt, CMD_LEN);
                    bool dec = vcmd_decode(pkt, false, &c) || vcall_decode(pkt, false, &c);
                    cmd_flags |= dec ? 32 : 64;
                    if (!act_valid && dec) {
                        if (c.flags == CMD_F_CALL) call_stage_part(&c);
                        act = c;
                        act.eff = (uint16_t)(s16 + CMD_EFF);
                        ev_count++;
                        act_valid = true;
                        ack_first = (uint16_t)(s16 + 2);
                        printf("C cmd id=%u flags=%u freq_hz=%lu pwr=%d eff=%u\n", act.id, act.flags,
                               (unsigned long)act.hz, act.pwr, act.eff);
                    }
                }
            }
            lr20xx_radio_fifo_clear_rx(radio);
            lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
            lr20xx_system_set_fs_mode(radio);
            radio_set_payload_len(PAYLOAD_LEN);
            seq++; t += SLOT_US;
            uint64_t now = time_us_64();
            while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
            continue;
        }
        if (act_valid && sdiff16(s16, ack_first) >= 0 && sdiff16(s16, (uint16_t)(ack_first + CMD_ACKS)) < 0) {
            if (act.flags == CMD_F_CALL) vcall_encode(pkt, s16, &act, true);  // acknowledge the command instead of video
            else vcmd_encode(pkt, s16, &act, true);
        } else if (ann_left || (s16 % ANN_PERIOD) == 1u ||
                   (g_hop.n && seq - last_plan_ann >= PLAN_ANN_SLOTS && chan_hz(seq) == g_freq_hz)) {
            // announce our power (downlink only, id 0); while hopping also the plan, and that regularly on the centre
            // channel, where a receiver that does not know the plan waits for it
            vcmd_t a = {.id = 0, .flags = CMD_F_PWR, .hz = g_freq_hz, .pwr = pwr_out(pwr), .eff = s16};
            if (g_hop.n) { a.flags |= CMD_F_HOP; a.hn = g_hop.n; a.hstep = g_hop.step_khz; }
            vcmd_encode(pkt, s16, &a, true);
            if (ann_left) ann_left--;
            last_plan_ann = seq;
        } else {
            pkt[0] = (uint8_t)seq; pkt[1] = (uint8_t)(seq >> 8);
            if (lq_head != lq_tail) {
                memcpy(pkt + 2, lq[lq_tail % LQ_CAP], ENTRY);
                if (pkt[2] != 0xFF) link_key_entry(pkt + 2);  // the host builds v4 entries: keyed here
                lq_tail++;
                data++, tot_data++;
            } else {
                memset(pkt + 2, 0, ENTRY);
                pkt[2] = pkt[3] = pkt[4] = 0xFF;
                air_status(pkt + 2, ident_quiet || g_ident_req == 2);
                idle++, tot_idle++;
            }
        }
        if (chan_hz(seq) != cur_hz) { cur_hz = chan_hz(seq); radio_set_freq(cur_hz); }  // hop (fixed: never)
        lr20xx_radio_fifo_write_tx(radio, pkt, PAYLOAD_LEN);
        while (time_us_64() + HOST_QUIET_US < t) poll_host();
        while (time_us_64() < t) tight_loop_contents();
        uint32_t t0 = time_us_32();
        lr20xx_radio_common_set_tx(radio, 0);
        // Status goes out while the packet is on air.
        slot_report(seq);
        tx_usb_poll();
        if (t0 - last_t >= 5000000u) {
            last_t = t0;
            printf("T seq=%lu sent=%lu fail=%lu late=%lu data=%lu idle=%lu bad=%lu air_us=%lu..%lu idents=%lu\n", (unsigned long)seq,
                   (unsigned long)sent, (unsigned long)fail, (unsigned long)late, (unsigned long)data,
                   (unsigned long)idle, (unsigned long)rec_bad, (unsigned long)air_min, (unsigned long)air_max,
                   (unsigned long)g_idents);
            // The host link over the same 5 s (a stalled Pi sender: which side stopped): bytes the host sent, records
            // taken and the bad-record total, a record half received, bytes waiting in the ring, the queue, whether
            // the two SPI DMA channels (MOSI ring, MISO status frame) still run, and when the host last sent a byte.
            {
                static uint32_t prev_bytes, prev_ok;
                unsigned wr = (unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
                printf("H bytes=%lu records=%lu bad_total=%lu partial=%u ring=%u queue=%lu rx_dma=%d tx_dma=%d host_ago_ms=%lu\n",
                       (unsigned long)(g_host_bytes - prev_bytes), (unsigned long)(rec_ok - prev_ok), (unsigned long)rec_bad,
                       rec_n, (wr - host_rd) & (HOST_RING_SIZE - 1), (unsigned long)(lq_head - lq_tail),
                       (int)dma_channel_is_busy(host_dma), (int)dma_channel_is_busy(host_tx_dma),
                       (unsigned long)(g_host_seen_us ? (time_us_64() - g_host_seen_us) / 1000u : 0u));
                prev_bytes = g_host_bytes; prev_ok = rec_ok;
            }
            if (g_hop.n) printf("C hop freq_hz=%lu n=%u spacing_hz=%lu\n", (unsigned long)g_freq_hz, g_hop.n,
                                (unsigned long)g_hop.step_khz * 1000u);
            sent = fail = data = idle = 0;
            air_min = 0xFFFFFFFFu; air_max = 0;
        }
        uint32_t irq = 0;
        for (;;) {
            if (radio_hal_irq_pin()) { irq = radio_hal_read_clear_irq_fast(); break; }
            if (time_us_32() - t0 > SLOT_US + 20000) { lr20xx_system_get_and_clear_irq_status(radio, &irq); break; }
            poll_host();
        }
        if (irq & LR20XX_SYSTEM_IRQ_TX_DONE) {
            uint32_t air = time_us_32() - t0;
            if (air < air_min) air_min = air;
            if (air > air_max) air_max = air;
            sent++, tot_sent++;
        }
        else { fail++, tot_fail++; recover(); }
        // Next slot; if we overran, skip whole slots so seq stays on schedule.
        seq++; t += SLOT_US;
        uint64_t now = time_us_64();
        while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
    }
}
#endif

// ---------------------------------------------------------------------------
// RX (B)
// ---------------------------------------------------------------------------
static const char HEX[] = "0123456789abcdef";

// USB output is buffered and written while the radio is listening.
// P (packet) line: P seq ch mode rssi_pkt rssi_sig snr foff dt rssi_pre hex
//   rssi_* in 0.5 dB units, snr in 0.25 dB units, foff in Hz, dt = arrival
//   minus expected end (us, 0 in acquisition), rssi_pre = channel RSSI when the
//   RX window opened (0.5 dB units, 9999 = not measured), mode A=acquire T=track.
// M (miss) line: M seq ch reason rssi_pre   reason: T timeout, C crc, L length,
//   W wrong seq, K skipped (too late to open the window), O other IRQ.
static char outbuf[8192];
static unsigned outlen;
static uint32_t out_dropped;

static void out_append(const char* s, unsigned n) {
    if (outlen + n >= sizeof outbuf) { out_dropped++; return; }
    memcpy(outbuf + outlen, s, n);
    outlen += n;
}
static void flush_out(void) {
    if (outlen) { fwrite(outbuf, 1, outlen, stdout); fflush(stdout); outlen = 0; }
}

typedef struct { uint32_t ok, crc, lvl, miss, wrong, acq, late; int rssi2, rsig2, snr, foff; } rx_stats_t;
static rx_stats_t st;
static uint32_t last_report;


#ifdef VLIVE
// ---- over-the-air control, receiver side (see the protocol description above) ----
// Host -> RP2350 text lines:  F <hz>  set TX+RX frequency (hopping: the centre)   P <half_dbm>  set TX power
//                             L <hz>  retune only this receiver (no link: use it to find a TX that moved)
//                             H <centre_hz> <n> <spacing_hz>  hop plan for TX+RX (n = 0: fixed frequency);
//                               without a link: this receiver only
//                             I <hex blob>  set the transmitter's callsign (sent in parts of 7 bytes, see "Identification")
//                             K  read the pairing profile;  K <centre_hz> <n> <spacing_hz> <pwr_half_dbm> <link>  save
//                               it (flash) and retune to it now (see "Pairing profile")
// RP2350 -> host:  "C ..." lines (cfg once per second, plus one line per step of a change);
//                  "I seq part parts <20 hex digits>" for every ident packet heard (the TX's identification).
enum { RQ_IDLE, RQ_WAIT, RQ_SENT, RQ_ACKED };
static struct { uint8_t state, tries; vcmd_t c; uint16_t sent_seq; } rq;
static uint8_t rq_id;
static int8_t g_pwr = TX_POWER_HALF_DBM;       // last TX power the link agreed on
static uint32_t g_alt_hz;                      // previous / pending frequency, also tried while searching
static hop_t g_alt_hop;                        // previous / pending hop plan, also tried while searching
static bool g_locked;
static bool g_local_retune;                    // acquisition: frequency was changed by an L / F command
static bool g_hop_adopted;                     // plan taken from a TX announcement: the radio was retuned
static vcmd_t g_ovh;                           // hop change overheard (another receiver's command, ACKed by the TX)
static bool g_ovh_valid;
#define SCAN_DWELL_US 250000u
static uint8_t g_up[CALL_MAX];                 // callsign being sent to the TX (I command), in parts of 7 bytes
static unsigned g_up_n, g_up_part;             // parts, next part (g_up_n = 0: nothing to send)

static void c_msg(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void c_msg(const char* fmt, ...) {
    char l[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(l, sizeof l, fmt, ap);
    va_end(ap);
    if (n > 0) out_append(l, (unsigned)(n < (int)sizeof l ? n : (int)sizeof l - 1));
}

static void rx_queue_call_part(void) {
    memset(&rq.c, 0, sizeof rq.c);
    rq.tries = 0;
    rq.c.flags = CMD_F_CALL;
    rq.c.cpn = (uint8_t)(g_up_part << 4 | (g_up_n - 1u));
    memcpy(rq.c.cd, g_up + g_up_part * 7u, 7);
    rq.c.id = ++rq_id;
    while (rq.c.id == 0 || rq.c.id == 0xFF) rq.c.id = ++rq_id;
    rq.state = RQ_WAIT;
    c_msg("C queued call part %u/%u (sent in the next command slot)\n", g_up_part + 1u, g_up_n);
}

// ---- Pairing profile (Horizon Dev): the link this receiver starts on, kept in flash.
// The setup site writes the same profile to the receiver ("K ..." below) and to the transmitter (Luckfox
// horizon_vtx.conf, sent to its RP2350 at every start), so both always start on the same frequency / hop plan, and a
// restart of either brings the pair back together. Over-the-air changes (F / H / P) stay temporary: not saved.
// Layout (second-to-last flash sector): "HZPR" ver=3 hz[4] n step_khz[2] pwr link[2] xor (all bytes from ver). Older
// versions are not read: the receiver starts on the build's default until it is paired again.
static bool g_prof_saved;
// At boot: the saved profile replaces the build's default frequency (and the TX power the receiver reports).
static void prof_load(void) {
    const uint8_t* f = (const uint8_t*)(XIP_BASE + PROF_FLASH_OFF);
    link_set(link_of(LINK_DEFAULT_BLOB, sizeof LINK_DEFAULT_BLOB));
    if (f[0] != 'H' || f[1] != 'Z' || f[2] != 'P' || f[3] != 'R' || f[4] != 3) return;
    uint8_t x = 0x5A;
    for (unsigned i = 4; i < 15; i++) x ^= f[i];
    if (x != f[15]) return;
    uint16_t link = (uint16_t)(f[13] | f[14] << 8);
    link_set(link ? link : 1u);
    uint32_t hz = (uint32_t)f[5] | (uint32_t)f[6] << 8 | (uint32_t)f[7] << 16 | (uint32_t)f[8] << 24;
    hop_t h = {f[9], (uint16_t)(f[10] | f[11] << 8)};
    int8_t pwr = (int8_t)f[12];
    if (!hop_valid(hz, &h) || pwr < VPWR_MIN || pwr > VPWR_MAX) return;
    g_freq_hz = hz; hop_set(&h); g_pwr = pwr;
    g_prof_saved = true;
}
static void prof_print(const char* what) {
    c_msg("K %s freq_hz=%lu hop=%u,%lu pwr=%d saved=%d link=%u\n", what, (unsigned long)g_freq_hz, g_hop.n,
          (unsigned long)g_hop.step_khz * 1000u, g_pwr, g_prof_saved, g_link);
}
// "K" reads the profile; "K <centre_hz> <n> <spacing_hz> <pwr_half_dbm> [<link>]" saves it and retunes to it now (the
// link: see "Link ID"; without it the current one is kept).
static void prof_command(const char* s) {
    while (*s == ' ') s++;
    if (!*s) { prof_print("profile"); return; }
    unsigned long hz = 0, n = 0, step = 0;
    long pwr = 0;
    unsigned long link = g_link;
    char plan[64];
    int got = sscanf(s, "%lu %lu %lu %ld %lu", &hz, &n, &step, &pwr, &link);
    if (got < 4 || link < 1 || link > 65535) {
        c_msg("C fail profile (K <centre_hz> <n 0|2..%u> <spacing_hz> <pwr_half_dbm> [<link 1..65535>])\n", HOP_MAX);
        return;
    }
    uint32_t centre; hop_t h;
    snprintf(plan, sizeof plan, "%lu %lu %lu", hz, n, step);
    if (!hop_parse(plan, &centre, &h) || pwr < VPWR_MIN || pwr > VPWR_MAX) { c_msg("C fail profile range\n"); return; }
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    prof_encode(page, centre, &h, (int8_t)pwr, (uint16_t)link);
    uint32_t ir = save_and_disable_interrupts();   // ~50 ms: a few slots are missed, nothing else
    flash_range_erase(PROF_FLASH_OFF, FLASH_SECTOR_SIZE);
    flash_range_program(PROF_FLASH_OFF, page, FLASH_PAGE_SIZE);
    restore_interrupts(ir);
    if (memcmp((const void*)(XIP_BASE + PROF_FLASH_OFF), page, PROF_LEN)) { c_msg("C fail profile (flash write)\n"); return; }
    g_prof_saved = true;
    g_alt_hz = g_freq_hz; g_alt_hop = g_hop;        // searching still tries the old plan
    g_freq_hz = centre; hop_set(&h); g_pwr = (int8_t)pwr;
    link_set((uint16_t)link);
    hop_retune();
    g_local_retune = true;
    prof_print("saved");
}

static void host_command(char* line) {
    unsigned long v = 0;
    long sv = 0;
    char c = line[0];
    if (c == 'K') { prof_command(line + 1); return; }  // pairing profile (see above)
    if (c == 'W') {  // who am I (see "Identity")
        char l[160];
        if (who_line(l, sizeof l) > 0) c_msg("%s", l);
        return;
    }
    if (c == 'I') {  // I <hex blob>: the transmitter's callsign, over the air
        const char* h = line + 1;
        while (*h == ' ') h++;
        uint8_t b[CALL_MAX];
        unsigned n = 0;
        while (h[0] && h[1] && n < CALL_MAX && hexval(h[0]) >= 0 && hexval(h[1]) >= 0) { b[n++] = (uint8_t)(hexval(h[0]) << 4 | hexval(h[1])); h += 2; }
        if (!n || *h || blob_len(b, n) != n) { c_msg("C fail call (I <hex blob>, as swfec.py encodes it)\n"); return; }
        if (!g_locked) { c_msg("C fail nolink\n"); return; }
        if (rq.state != RQ_IDLE) { c_msg("C busy\n"); return; }
        memset(g_up, 0, sizeof g_up);
        memcpy(g_up, b, n);
        g_up_n = (n + 6u) / 7u;
        g_up_part = 0;
        rx_queue_call_part();
        return;
    }
    if (c == 'H') {
        uint32_t hz; hop_t h;
        if (!hop_parse(line + 1, &hz, &h)) {
            c_msg("C fail hop (H <centre_hz> <n 0|2..%u> <spacing_hz, kHz steps>, channels %lu..%lu Hz)\n",
                  HOP_MAX, (unsigned long)VFREQ_MIN_HZ, (unsigned long)VFREQ_MAX_HZ);
            return;
        }
        if (!g_locked) {  // no link: this receiver only (e.g. to join a TX that already hops)
            g_alt_hz = g_freq_hz; g_alt_hop = g_hop;
            g_freq_hz = hz; hop_set(&h);
            hop_retune();
            g_local_retune = true;
            c_msg("C local hop freq_hz=%lu n=%u spacing_hz=%lu (receiver only, no link)\n", (unsigned long)hz,
                  h.n, (unsigned long)h.step_khz * 1000u);
            return;
        }
        if (rq.state == RQ_SENT || rq.state == RQ_ACKED) { c_msg("C busy\n"); return; }
        if (rq.state == RQ_WAIT && (rq.c.flags & (CMD_F_PWR | CMD_F_CALL))) { c_msg("C busy (change pending)\n"); return; }
        rq.tries = 0;
        rq.c.flags = CMD_F_HOP; rq.c.hz = hz; rq.c.pwr = 0; rq.c.eff = 0;
        rq.c.hn = h.n; rq.c.hstep = h.step_khz;
    } else if ((c == 'F' || c == 'L') && sscanf(line + 1, "%lu", &v) == 1) {
        if (!vfreq_ok(v)) { c_msg("C fail range freq_hz=%lu\n", v); return; }
        if (g_hop.n && !hop_valid((uint32_t)v, &g_hop)) { c_msg("C fail range (hop channels outside %lu..%lu Hz)\n",
                                                                 (unsigned long)VFREQ_MIN_HZ, (unsigned long)VFREQ_MAX_HZ); return; }
        if (c == 'L' || !g_locked) {  // no link (or explicitly local): retune just this receiver
            if (g_locked) { c_msg("C fail L only without link\n"); return; }
            g_alt_hz = g_freq_hz;
            g_freq_hz = (uint32_t)v;
            hop_retune();
            g_local_retune = true;
            c_msg("C local freq_hz=%lu (receiver only, no link)\n", v);
            return;
        }
        if (rq.state == RQ_SENT || rq.state == RQ_ACKED || (rq.state == RQ_WAIT && rq.c.flags == CMD_F_CALL)) { c_msg("C busy\n"); return; }
        if (rq.state != RQ_WAIT) { rq.c.flags = 0; rq.tries = 0; }
        if (rq.c.flags & CMD_F_HOP) rq.c.hz = (uint32_t)v;  // pending plan change: just move its centre
        else { rq.c.flags |= CMD_F_FREQ; rq.c.hz = (uint32_t)v; }
    } else if (c == 'P' && sscanf(line + 1, "%ld", &sv) == 1) {
        if (sv < VPWR_MIN || sv > VPWR_MAX) { c_msg("C fail range pwr=%ld\n", sv); return; }
        if (!g_locked) { c_msg("C fail nolink\n"); return; }
        if (rq.state == RQ_SENT || rq.state == RQ_ACKED) { c_msg("C busy\n"); return; }
        if (rq.state == RQ_WAIT && (rq.c.flags & (CMD_F_HOP | CMD_F_CALL))) { c_msg("C busy (change pending)\n"); return; }
        if (rq.state != RQ_WAIT) { rq.c.flags = 0; rq.tries = 0; }
        rq.c.flags |= CMD_F_PWR; rq.c.pwr = (int8_t)sv;
    } else {
        c_msg("C fail bad command\n");
        return;
    }
    rq.c.id = ++rq_id;
    while (rq.c.id == 0 || rq.c.id == 0xFF) rq.c.id = ++rq_id;  // id 0 = the TX's announcements, 0xFF = TX-side change
    rq.state = RQ_WAIT;
    c_msg("C queued flags=%u (sent in the next command slot, within %u ms)\n", rq.c.flags,
          (unsigned)(CMD_PERIOD * SLOT_US / 1000));
}

// Non-blocking read of host command lines (a few us when idle).
static void host_poll(void) {
    static char buf[256];
    static unsigned n;
    uint8_t tmp[32];
    int got = stdio_get_until((char*)tmp, sizeof tmp, get_absolute_time());
    for (int k = 0; k < got; k++) {
        if (tmp[k] == '\n' || tmp[k] == '\r') {
            if (n) { buf[n] = 0; host_command(buf); n = 0; }
        } else if (n < sizeof buf - 1) buf[n++] = (char)tmp[k];
        else n = 0;
    }
}

// Sends the pending command in command slot `seq`, whose video packet would end at `end`.
// The TX has been listening since its previous packet ended (about 1.3 ms before the slot starts) and
// stops listening 0.7 ms before the slot ends, so a packet starting at the nominal slot start fits.
// The command is started CMD_EARLY_US before the nominal slot start when there is time for that: the TX listens
// from before the slot starts, but stops listening shortly before the slot ends, and the packet reaches it late by
// the receive delay of both radios, so a nominal start leaves only a few hundred us of margin at the end.
#define CMD_EARLY_US 400u
static bool rx_send_cmd(uint64_t end, uint16_t seq) {
    // 1 ms after the command slot's nominal packet start: the TX listens from right after its previous packet until
    // 0.7 ms before the slot ends, and a 16-byte command takes ~2.2 ms on air (ends ~3.2 ms into the slot)
    uint64_t start = end - AIR_US + 1000;
    int32_t lead = (int32_t)(start - time_us_64());
    if (time_us_64() + 200 > start) return false;  // too late for this slot: try the next command slot
    uint8_t p[PAYLOAD_LEN];
    if (rq.c.flags == CMD_F_CALL) vcall_encode(p, seq, &rq.c, false);
    else vcmd_encode(p, seq, &rq.c, false);
    lr20xx_system_set_fs_mode(radio);
    if (g_hop.n) radio_set_freq(chan_hz(seq));  // the TX listens on this slot's hop channel
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    radio_set_payload_len(CMD_LEN);
    lr20xx_radio_fifo_clear_tx(radio);
    lr20xx_radio_fifo_write_tx(radio, p, CMD_LEN);
    while (time_us_64() < start) tight_loop_contents();
    uint32_t t0 = time_us_32();
    lr20xx_radio_common_set_tx(radio, 0);
    uint32_t irq = 0;
    for (;;) {
        if (radio_hal_irq_pin()) { irq = radio_hal_read_clear_irq_fast(); break; }
        if (time_us_32() - t0 > AIR_US + 5000) { lr20xx_system_get_and_clear_irq_status(radio, &irq); break; }
    }
    radio_set_payload_len(PAYLOAD_LEN);
    if (!(irq & LR20XX_SYSTEM_IRQ_TX_DONE)) { recover(); return false; }
    rq.state = RQ_SENT; rq.sent_seq = seq; rq.tries++;
    if (rq.c.flags & (CMD_F_FREQ | CMD_F_HOP)) g_alt_hz = rq.c.hz;  // if only the ACKs get lost, searching still finds the TX
    if (rq.c.flags & CMD_F_HOP) { g_alt_hop.n = rq.c.hn; g_alt_hop.step_khz = rq.c.hstep; }
    c_msg("C sent try=%u early_us=%ld lead_us=%ld\n", rq.tries, (long)(end - AIR_US - start), (long)lead);
    return true;
}

static void rx_handle_ack(const uint8_t* p) {
    vcmd_t a;
    if (p[6] == CMD_F_CALL) {  // ACK of a callsign part (ours, or another receiver's: ignored)
        if (!vcall_decode(p, true, &a) || rq.state != RQ_SENT || rq.c.flags != CMD_F_CALL || a.id != rq.c.id ||
            a.cpn != rq.c.cpn || memcmp(a.cd, rq.c.cd, 7) || a.eff != (uint16_t)(rq.sent_seq + CMD_EFF)) return;
        rq.c.eff = a.eff;
        rq.state = RQ_ACKED;
        c_msg("C ack eff=%u\n", a.eff);
        return;
    }
    if (!vcmd_decode(p, true, &a)) return;
    if (a.id == 0) {  // the TX announcing its power (changed on the TX side, at its boot, or periodically)
        if ((a.flags & CMD_F_PWR) && a.pwr != g_pwr) {
            g_pwr = a.pwr;
            c_msg("C txpwr pwr=%d (reported by the transmitter)\n", g_pwr);
        }
        // ... and, while it hops, its plan (an announcement without the plan = the TX is on a fixed frequency)
        hop_t h = {a.hn, a.hstep};
        if (!(a.flags & CMD_F_HOP)) h.n = 0;
        if (!hop_same(&h, &g_hop) || (h.n && a.hz != g_freq_hz)) {
            g_alt_hz = g_freq_hz; g_alt_hop = g_hop;
            if (h.n) g_freq_hz = a.hz;
            hop_set(&h);
            hop_retune();
            g_hop_adopted = true;
            c_msg("C txhop freq_hz=%lu n=%u spacing_hz=%lu (reported by the transmitter)\n",
                  (unsigned long)g_freq_hz, g_hop.n, (unsigned long)g_hop.step_khz * 1000u);
        }
        return;
    }
    if (a.id == rq.c.id && rq.state == RQ_ACKED) return;  // another copy of the ACK for our own command
    if (rq.state != RQ_SENT || a.id != rq.c.id) {
        // Another receiver's command (diversity): a hop plan change, or a centre change while hopping, is applied
        // here too, in the same agreed slot as the TX, so this receiver never loses the link (receive only).
        if (((a.flags & CMD_F_HOP) || (g_hop.n && (a.flags & CMD_F_FREQ))) && !(g_ovh_valid && g_ovh.id == a.id)) {
            g_ovh = a; g_ovh_valid = true;
            c_msg("C overheard hop change id=%u eff=%u\n", a.id, a.eff);
        }
        return;
    }
    if (a.id != rq.c.id || a.flags != rq.c.flags || a.hz != rq.c.hz || a.pwr != rq.c.pwr ||
        a.eff != (uint16_t)(rq.sent_seq + CMD_EFF)) return;
    if ((a.flags & CMD_F_HOP) && (a.hn != rq.c.hn || a.hstep != rq.c.hstep)) return;
    rq.c.eff = a.eff;
    rq.state = RQ_ACKED;
    c_msg("C ack eff=%u\n", a.eff);
}

// Takes effect at the agreed slot. Returns the number of slots to skip (quiet time).
static uint16_t rx_apply(uint16_t n16) {
    uint16_t skip = 0;
    if (rq.c.flags == CMD_F_CALL) {  // the TX has this part: next one, or done
        rq.state = RQ_IDLE;
        if (++g_up_part < g_up_n) { c_msg("C applied call part %u/%u\n", g_up_part, g_up_n); rx_queue_call_part(); }
        else { c_msg("C applied call (%u parts): the transmitter stored the callsign\n", g_up_n); g_up_n = 0; }
        return 0;
    }
    if (rq.c.flags & CMD_F_PWR) g_pwr = rq.c.pwr;
    if (rq.c.flags & CMD_F_HOP) {
        g_alt_hz = g_freq_hz; g_alt_hop = g_hop;
        hop_t h = {rq.c.hn, rq.c.hstep};
        hop_set(&h);
        g_freq_hz = rq.c.hz;
        bool ok = hop_retune();
        int16_t d = sdiff16((uint16_t)(rq.c.eff + CMD_QUIET), n16);
        if (d > 0) skip = (uint16_t)d;
        c_msg("C applied hop freq_hz=%lu n=%u spacing_hz=%lu pwr=%d cal=%d\n", (unsigned long)g_freq_hz, g_hop.n,
              (unsigned long)g_hop.step_khz * 1000u, g_pwr, ok);
    } else if ((rq.c.flags & CMD_F_FREQ) && rq.c.hz != g_freq_hz) {
        g_alt_hz = g_freq_hz;
        g_freq_hz = rq.c.hz;
        bool ok = hop_retune();
        int16_t d = sdiff16((uint16_t)(rq.c.eff + CMD_QUIET), n16);
        if (d > 0) skip = (uint16_t)d;
        c_msg("C applied freq_hz=%lu pwr=%d cal=%d\n", (unsigned long)g_freq_hz, g_pwr, ok);
    } else {
        c_msg("C applied freq_hz=%lu pwr=%d\n", (unsigned long)g_freq_hz, g_pwr);
    }
    rq.state = RQ_IDLE;
    return skip;
}
#endif

// kind 'P' = packet that passed the CRC, 'E' = packet that FAILED the CRC (vlive: forwarded anyway, the host repairs
// it with the inner Reed-Solomon code, or combines copies from several receivers). Same fields.
static void queue_packet_kind(char kind, uint32_t seq, char mode, int dt, int pre2, const uint8_t* p) {
    char l[2 * PAYLOAD_LEN + 96];
    int n = snprintf(l, sizeof l, "%c %lu %u %c %d %d %d %d %d %d ", kind, (unsigned long)seq, (unsigned)chan_idx(seq), mode,
                     st.rssi2, st.rsig2, st.snr, st.foff, dt, pre2);
    for (unsigned i = 0; i < PAYLOAD_LEN; i++) {
        l[n++] = HEX[p[i] >> 4];
        l[n++] = HEX[p[i] & 15];
    }
    l[n++] = '\n';
    out_append(l, n);
}
static void queue_packet(uint32_t seq, char mode, int dt, int pre2, const uint8_t* p) {
    queue_packet_kind('P', seq, mode, dt, pre2, p);
}
static void queue_miss(uint32_t seq, char reason, int pre2) {
    char l[48];
    int n = snprintf(l, sizeof l, "M %lu %u %c %d\n", (unsigned long)seq, (unsigned)chan_idx(seq), reason, pre2);
    out_append(l, n);
}

static void report(bool locked) {
    if (time_us_32() - last_report < 1000000) return;
    last_report = time_us_32();
    char l[200];
    int n = snprintf(l, sizeof l, "S ok=%lu crc=%lu lvl=%lu miss=%lu wrong=%lu late=%lu acq=%lu locked=%d out_dropped=%lu\n",
                     (unsigned long)st.ok, (unsigned long)st.crc, (unsigned long)st.lvl, (unsigned long)st.miss,
                     (unsigned long)st.wrong, (unsigned long)st.late, (unsigned long)st.acq, locked,
                     (unsigned long)out_dropped);
    out_append(l, n);
#ifdef VLIVE
    if (g_hop.n)  // hopping: plan as hop=n,spacing_hz (fixed: no hop=)
        n = snprintf(l, sizeof l, "C cfg freq_hz=%lu pwr=%d locked=%d hop=%u,%lu link=%u\n", (unsigned long)g_freq_hz,
                     g_pwr, locked, g_hop.n, (unsigned long)g_hop.step_khz * 1000u, g_link);
    else
        n = snprintf(l, sizeof l, "C cfg freq_hz=%lu pwr=%d locked=%d link=%u\n", (unsigned long)g_freq_hz, g_pwr, locked,
                     g_link);
    out_append(l, n);
#endif
}

static int rssi_now2(void) {
    int16_t r = 0;
    uint8_t h = 0;
    if (lr20xx_radio_common_get_rssi_inst(radio, &r, &h) != LR20XX_STATUS_OK) return 9999;
    return 2 * r - h;
}

// Reads a received packet. Returns 0 if it passed CRC and length checks,
// otherwise the miss reason character.
static char read_packet(uint32_t irq, uint8_t* p) {
    if (!(irq & LR20XX_SYSTEM_IRQ_RX_DONE)) return 'O';
    if (irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR)) {
        st.crc++;
#ifdef VLIVE
        if (!(irq & LR20XX_SYSTEM_IRQ_LEN_ERROR)) {  // CRC failed but full length: keep the bytes for the host
            uint16_t lv = 0;
            lr20xx_radio_fifo_get_rx_level(radio, &lv);
            if (lv == PAYLOAD_LEN) {
                lr20xx_radio_fifo_read_rx(radio, p, PAYLOAD_LEN);
                lr20xx_radio_lora_packet_status_t ps;
                if (lr20xx_radio_lora_get_packet_status(radio, &ps) == LR20XX_STATUS_OK) {
                    st.rssi2 = 2 * ps.rssi_pkt_in_dbm - ps.rssi_pkt_half_dbm_count;
                    st.rsig2 = 2 * ps.rssi_signal_pkt_in_dbm - ps.rssi_signal_pkt_half_dbm_count;
                    st.snr = ps.snr_pkt_raw;
                    st.foff = ps.freq_offset_hz;
                }
                return 'E';
            }
        }
#endif
        lr20xx_radio_fifo_clear_rx(radio);
        return 'C';
    }
    uint16_t level = 0;
    lr20xx_radio_fifo_get_rx_level(radio, &level);
    if (level != PAYLOAD_LEN) {
        st.lvl++;
        lr20xx_radio_fifo_clear_rx(radio);
        return 'L';
    }
    lr20xx_radio_fifo_read_rx(radio, p, PAYLOAD_LEN);
    lr20xx_radio_lora_packet_status_t ps;
    if (lr20xx_radio_lora_get_packet_status(radio, &ps) == LR20XX_STATUS_OK) {
        st.rssi2 = 2 * ps.rssi_pkt_in_dbm - ps.rssi_pkt_half_dbm_count;
        st.rsig2 = 2 * ps.rssi_signal_pkt_in_dbm - ps.rssi_signal_pkt_half_dbm_count;
        st.snr = ps.snr_pkt_raw;
        st.foff = ps.freq_offset_hz;
    }
    return 0;
}

static void run_rx(void) {
    uint8_t p[PAYLOAD_LEN];
#ifdef VLIVE
    const uint32_t boot_hz = g_freq_hz;
#endif
    for (;;) {
        // ---- acquisition: park on one channel until any valid packet arrives ----
        st.acq++;
#ifdef VLIVE
        g_locked = false;
        if (rq.state != RQ_IDLE) {  // link lost during a change: finished if we already sit on the new frequency
            if (rq.c.flags == CMD_F_FREQ && rq.c.hz == g_freq_hz) { c_msg("C applied freq_hz=%lu pwr=%d\n", (unsigned long)g_freq_hz, g_pwr); rq.state = RQ_IDLE; }
            else if (rq.state != RQ_WAIT) { rq.state = RQ_WAIT; }
        }
        // (frequency, plan) pairs to try, in turn, while searching: current, previous/pending frequency and plan,
        // boot default (fixed). The receiver parks on the centre frequency, which is a channel of every plan.
        // With only one distinct value this is the original "park on one channel" behaviour.
        const hop_t fixed = {0};
        struct { uint32_t hz; hop_t h; } scan[5] = {{g_freq_hz, g_hop}, {g_alt_hz, g_hop}, {g_freq_hz, g_alt_hop},
                                                    {boot_hz, fixed}, {g_freq_hz, fixed}};
        unsigned nscan = 1, si = 0;
        for (unsigned i = 1; i < 5; i++) {
            bool dup = scan[i].hz == 0 || !hop_valid(scan[i].hz, &scan[i].h);
            for (unsigned j = 0; j < nscan; j++) dup |= scan[j].hz == scan[i].hz && hop_same(&scan[j].h, &scan[i].h);
            if (!dup) scan[nscan++] = scan[i];
        }
        uint32_t dwell_t0 = time_us_32();
        const uint32_t f_start = g_freq_hz;
        const hop_t h_start = g_hop;
#endif
        recover();
        lr20xx_system_set_fs_mode(radio);
#ifdef VLIVE
        radio_set_freq(g_freq_hz);  // the centre (fixed: the frequency, as before)
#else
        radio_set_freq(chan_hz(0));
#endif
        uint32_t base_seq = 0;
        uint64_t base_end = 0;
        // Single-packet RX, re-armed with an empty FIFO after every event
        // (continuous RX let FIFO contents accumulate across packets).
        bool armed = false;
        for (;;) {
#ifdef VLIVE
            host_poll();
            if (g_local_retune || g_hop_adopted) {  // L / H command or plan from an announcement: list is stale
                g_local_retune = g_hop_adopted = false;
                nscan = 1; si = 0; scan[0].hz = g_freq_hz; scan[0].h = g_hop;
                dwell_t0 = time_us_32();
                armed = false;
            } else if (nscan > 1 && time_us_32() - dwell_t0 >
                       SCAN_DWELL_US + (g_hop.n ? 2u * g_hop.n * SLOT_US + PLAN_ANN_SLOTS * SLOT_US : 0u)) {
                // (hopping: long enough for the TX to visit the centre twice and announce its plan there)
                si = (si + 1) % nscan;
                g_freq_hz = scan[si].hz;
                hop_set(&scan[si].h);
                dwell_t0 = time_us_32();
                lr20xx_system_set_fs_mode(radio);
                radio_set_freq(g_freq_hz);
                armed = false;
            }
#endif
            if (!armed) {
                lr20xx_system_set_fs_mode(radio);
                lr20xx_radio_fifo_clear_rx(radio);
                lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
                lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
                armed = true;
            }
            flush_out();
            report(false);
            if (!radio_hal_irq_pin()) continue;
            uint64_t now = time_us_64();
            uint32_t irq = radio_hal_read_clear_irq_fast();
            armed = false;
            if (read_packet(irq, p)) continue;
            uint16_t s16 = (uint16_t)(p[0] | p[1] << 8);
#ifdef VLIVE
            // a TX announcement heard while parked: it carries the TX's plan (hopping or fixed), take it
            if (p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE) { rx_handle_ack(p); g_hop_adopted = false; }
            if (chan_hz(s16) != g_freq_hz) { st.wrong++; continue; }  // (this plan would not be on the centre now)
#else
            if (chan_hz(s16) != chan_hz(0)) { st.wrong++; continue; }
#endif
            base_seq = s16;
            base_end = now;
            st.ok++;
            queue_packet(base_seq, 'A', 0, 9999, p);
            break;
        }
#ifdef VLIVE
        if (g_freq_hz != f_start && g_alt_hz == 0) g_alt_hz = f_start;
        if (g_hop.n && (g_freq_hz != f_start || !hop_same(&g_hop, &h_start))) hop_retune();  // calibrate for the band
        if (g_hop.n) c_msg("C found freq_hz=%lu hop n=%u spacing_hz=%lu\n", (unsigned long)g_freq_hz, g_hop.n,
                           (unsigned long)g_hop.step_khz * 1000u);
        else if (g_freq_hz != f_start) c_msg("C found freq_hz=%lu\n", (unsigned long)g_freq_hz);
        g_locked = true;
#endif
        // ---- tracking: one RX window per slot on the predicted channel ----
        // Format v3 slots leave only ~0.4 ms between packets, so after a packet the receiver is re-rx_armed at once
        // (status + FIFO read, then SetRx: ~0.15 ms) and everything slow (hex formatting, USB) happens while the
        // next packet is on air. After a missed slot the radio simply stays in RX for the next one.
        uint32_t next = base_seq + 1, misses = 0;
        bool rx_armed = false;
        uint32_t last_hz = 0;
        // the previous slot's result, formatted after re-arming: kind 0 none, 'P' packet, 'E' failed CRC, else miss
        char pend_kind = 0, pend_mode = 'T';
        uint32_t pend_seq = 0;
        int pend_dt = 0, pend_pre = 9999;
        static uint8_t pend_p[PAYLOAD_LEN];
#ifdef VLIVE
        // hopping only: link check over windows of 128 slots (a fixed-mode link keeps the original rule only)
        unsigned hw_n = 0, hw_ok_c = 0, hw_ok_o = 0;
        bool hop_lost = false;
#endif
        while (misses < LOST_MISSES) {
#ifdef VLIVE
            if (g_hop_adopted) {  // plan taken from an announcement (radio recalibrated): retune before the next window
                g_hop_adopted = false;
                rx_armed = false; last_hz = 0;
                hw_n = hw_ok_c = hw_ok_o = 0;
            }
            if (g_hop.n && hw_n >= 128u) {
                if (hw_ok_o == 0 && hw_ok_c >= 2) {
                    // only centre-channel slots got through: the TX is on a fixed frequency (e.g. it rebooted)
                    hop_t f = {0};
                    g_alt_hop = g_hop;
                    hop_set(&f);
                    hop_retune();
                    rx_armed = false; last_hz = 0;
                    c_msg("C hop off: only the centre channel is heard, the TX is on a fixed frequency\n");
                } else if ((hw_ok_c + hw_ok_o) * 4u < hw_n) {
                    hop_lost = true;  // under 25 %: wrong plan or no link, search again (TX plan announcements)
                    break;
                }
                hw_n = hw_ok_c = hw_ok_o = 0;
            }
#endif
            uint64_t end = base_end + (uint64_t)(next - base_seq) * SLOT_US;
#ifdef VLIVE
            uint16_t n16 = (uint16_t)next;
            if (rq.state == RQ_ACKED && sdiff16(n16, rq.c.eff) >= 0) {  // agreed slot: apply together with the TX
                next += rx_apply(n16);
                rx_armed = false;
                continue;
            }
            if (g_ovh_valid && sdiff16(n16, g_ovh.eff) >= 0) {  // overheard hop change: same slot as the TX
                g_ovh_valid = false;
                if (sdiff16(n16, g_ovh.eff) < 64) {
                    vcmd_t save = rq.c; uint8_t st_save = rq.state;
                    rq.c = g_ovh; rq.c.flags &= (uint8_t)~CMD_F_PWR;  // (power: the TX announces it anyway)
                    next += rx_apply(n16);
                    rq.c = save; rq.state = st_save;
                    rx_armed = false; last_hz = 0;
                    hw_n = hw_ok_c = hw_ok_o = 0;
                    continue;
                }
            }
            if (rq.state == RQ_SENT && sdiff16(n16, (uint16_t)(rq.sent_seq + CMD_EFF - 1)) >= 0) {  // no ACK came
                if (rq.tries < 3) { rq.state = RQ_WAIT; c_msg("C retry (no ACK)\n"); }
                else { rq.state = RQ_IDLE; g_up_n = 0; c_msg("C fail no ACK from the transmitter\n"); }
            }
            if ((n16 % CMD_PERIOD) == 0) {  // command slot: the TX listens (or identifies), it sends no video
                lr20xx_system_set_fs_mode(radio);
                rx_armed = false;
                if (g_hop.n) last_hz = 0;  // a command goes out on this slot's channel: retune for the next window
                bool sent = rq.state == RQ_WAIT && rx_send_cmd(end, n16);  // first: the command must start within ~0.3 ms
                if (!sent) {  // listen for the TX's identification (a 16-byte packet IDENT_DELAY_US into the slot)
                    if (g_hop.n) radio_set_freq(chan_hz(n16));
                    radio_set_payload_len(CMD_LEN);
                    lr20xx_radio_fifo_clear_rx(radio);
                    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
                    lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
                }
                if (pend_kind) {
                    if (pend_kind == 'P') queue_packet(pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                    else if (pend_kind == 'E') queue_packet_kind('E', pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                    else queue_miss(pend_seq, pend_kind, pend_pre);
                    pend_kind = 0;
                }
                flush_out();
                report(true);
                if (!sent) {
                    // the ident ends ~3.2 ms into the slot; give up at 6 ms (the next packet starts at the slot's end)
                    uint64_t until = end - AIR_US + 6000u;
                    bool got = false;
                    uint32_t irq = 0;
                    while (time_us_64() < until) {
                        if (radio_hal_irq_pin()) { irq = radio_hal_read_clear_irq_fast(); got = true; break; }
                    }
                    uint16_t lv = 0;
                    if (got && (irq & LR20XX_SYSTEM_IRQ_RX_DONE) &&
                        !(irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR)) &&
                        lr20xx_radio_fifo_get_rx_level(radio, &lv) == LR20XX_STATUS_OK && lv == CMD_LEN) {
                        uint8_t ip[CMD_LEN];
                        lr20xx_radio_fifo_read_rx(radio, ip, CMD_LEN);
                        if (ident_check(ip)) {
                            char l[80];
                            int n = snprintf(l, sizeof l, "I %lu %u %u ", (unsigned long)next, ip[4] >> 4, (ip[4] & 15u) + 1u);
                            for (unsigned i = 0; i < IDENT_PART; i++) { l[n++] = HEX[ip[5 + i] >> 4]; l[n++] = HEX[ip[5 + i] & 15]; }
                            l[n++] = '\n';
                            out_append(l, (unsigned)n);
                        }
                    }
                    lr20xx_system_set_fs_mode(radio);
                    lr20xx_radio_fifo_clear_rx(radio);
                    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
                    radio_set_payload_len(PAYLOAD_LEN);
                    flush_out();
                }
                next++;
                continue;
            }
#endif
            if (!rx_armed) {
                uint64_t open = end - AIR_US - RX_PRE_US;
                uint64_t now = time_us_64();
                if (now > open + RX_PRE_US) {  // too late to catch this preamble: skip slot
                    st.late++; st.miss++; misses++;
                    queue_miss(next, 'K', 9999);
                    next++;
                    continue;
                }
                lr20xx_system_set_fs_mode(radio);
                lr20xx_radio_fifo_clear_rx(radio);
                lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
                uint32_t hz = chan_hz(next);
                if (hz != last_hz) { radio_set_freq(hz); last_hz = hz; }
                while (time_us_64() < open) tight_loop_contents();
                lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);  // single, no timeout
                rx_armed = true;
            }
            // the previous slot's line, now that the radio listens again
            if (pend_kind) {
                if (pend_kind == 'P') queue_packet(pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                else if (pend_kind == 'E') queue_packet_kind('E', pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                else queue_miss(pend_seq, pend_kind, pend_pre);
                pend_kind = 0;
            }
            flush_out();  // USB output happens while the radio is listening
            report(true);
#ifdef VLIVE
            host_poll();
#endif
            int pre2 = rssi_now2();  // channel energy before the packet's preamble
            char reason = 'T';
            uint64_t t_irq = 0;
#ifdef VLIVE
            // hopping: close the window early after a miss (packets end within a few us of `end`), so there is time
            // to retune before the next packet starts ~0.33 ms after this one ends
            const uint64_t post = g_hop.n ? 120u : RX_POST_US;
#else
            const uint64_t post = RX_POST_US;
#endif
            while (time_us_64() < end + post) {
                if (!radio_hal_irq_pin()) continue;
                t_irq = time_us_64();
                uint32_t irq = radio_hal_read_clear_irq_fast();
                if (t_irq + SLOT_US / 2 < end) {  // far too early: not this slot's packet (e.g. another receiver's
                    lr20xx_radio_fifo_clear_rx(radio);  // command in the command slot): listen on
                    lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
                    continue;
                }
                reason = read_packet(irq, p);
                if (irq & LR20XX_SYSTEM_IRQ_RX_DONE) {  // a packet ended: re-arm first
                    if (reason == 'C' || reason == 'L') lr20xx_radio_fifo_clear_rx(radio);
#ifdef VLIVE
                    if (g_hop.n) {  // hop to the next slot's channel first (fallback mode after RX is FS)
                        uint32_t hz = chan_hz(next + 1);
                        if (hz != last_hz) { radio_set_freq(hz); last_hz = hz; }
                    }
#endif
                    lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
                    rx_armed = true;
                } else {
                    rx_armed = false;
                }
                if (!reason) {
                    uint16_t s16 = (uint16_t)(p[0] | p[1] << 8);
                    if (s16 == (uint16_t)next) {
                        base_seq = next;
                        base_end = t_irq;
                        st.ok++;
#ifdef VLIVE
                        if (p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE) rx_handle_ack(p);
#endif
                    } else {
                        st.wrong++;
                        reason = 'W';
                    }
                }
                break;
            }
            // (no event by the end of the window: the radio stays in RX and catches the next slot's packet)
#ifdef VLIVE
            if (g_hop.n && rx_armed && chan_hz(next + 1) != last_hz) {  // hopping: not on this channel, though
                lr20xx_system_set_fs_mode(radio);
                uint32_t hz = chan_hz(next + 1);
                radio_set_freq(hz); last_hz = hz;
                lr20xx_radio_fifo_clear_rx(radio);
                lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
                lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(radio, 0);
            }
            if (g_hop.n) {
                hw_n++;
                if (!reason) { if (chan_hz(next) == g_freq_hz) hw_ok_c++; else hw_ok_o++; }
            }
#endif
            pend_seq = next; pend_pre = pre2; pend_mode = 'T';
            pend_dt = t_irq ? (int)((int64_t)t_irq - (int64_t)end) : 0;
            if (!reason) {
                misses = 0;
                pend_kind = 'P';
                memcpy(pend_p, p, PAYLOAD_LEN);
            } else if (reason == 'E') {  // failed CRC: forward the bytes (the slot still counts as a miss here)
                st.miss++; misses++;
                pend_kind = 'E';
                memcpy(pend_p, p, PAYLOAD_LEN);
            } else {
                st.miss++; misses++;
                pend_kind = reason;
            }
            next++;
        }
        if (pend_kind && pend_kind != 'P' && pend_kind != 'E') queue_miss(pend_seq, pend_kind, pend_pre);
#ifdef VLIVE
        if (hop_lost) c_msg("C hop link lost (under 25 %% of the slots received): searching\n");
#endif
    }
}

int main(void) {
    stdio_init_all();
#ifdef PIN_LED
    gpio_init(PIN_LED);  // the status LED (led_tick()): off until the first poll_host()
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);
#endif
    pico_get_unique_board_id_string(g_id, sizeof g_id);
    make_perm();
    sleep_ms(1500);
    int rc;
    who_print();
#if defined(LIVE_TX)
    // The host link first: the host (Luckfox) can then identify this board and read the radio state even when the
    // radio does not start (no module, no power, SPI wiring), instead of seeing nothing at all.
    host_spi_init();
    host_status_publish();
#endif
    while ((rc = radio_init()) != 0) {
        printf("radio_init failed (%d)\n", rc);
        g_radio_rc = rc;
        g_radio_state = 2;
#if defined(LIVE_TX)
        host_status_publish();
        uint64_t until = time_us_64() + 2000000u;
        while (time_us_64() < until) poll_host();  // keep answering the host (identity, records) while retrying
#else
        sleep_ms(2000);
#endif
    }
    g_radio_state = 1;
    g_radio_rc = 0;
#if defined(LIVE_TX)
#ifdef VLIVE_HOST_PIO
    printf("vlive TX host=luckfox-pio spi_mode=0 sck=%u rx=%u tx=%u cs=%u\n", PIN_HOST_SCK, PIN_HOST_RX, PIN_HOST_TX,
           PIN_HOST_CS);
#endif
    run_live_tx();
#elif defined(VIDEO_TX)
    printf("vlink TX id=%s\n", g_id);
    run_tx();
#elif defined(VLIVE)
    prof_load();  // the paired profile, if one was saved (else the build's default frequency)
    // onto the profile's plan now: the radio started on the build's default (sub-GHz), and a 2.4 GHz profile needs the
    // HF path (radio.c switches band in a retune; setting the frequency alone does not)
    if (!hop_retune()) printf("radio: retune to the profile's plan failed\n");
    printf("vlive RX id=%s freq_hz=%lu slot_us=%u profile=%s\n", g_id, (unsigned long)g_freq_hz, SLOT_US,
           g_prof_saved ? "saved" : "default");
    run_rx();
#else
    printf("vlink RX id=%s\n", g_id);
    run_rx();
#endif
}
