// The shared part of the radio firmware (common.h).
#include "common.h"

char g_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
// Reported in the identity frame, so a radio that does not start does not look like a dead SPI link.
// 0 = not tried yet, 1 = running, 2 = failing.
volatile uint8_t g_radio_state;
volatile int g_radio_rc;
const char* radio_state_name(void) {
    return g_radio_state == 1 ? "ok" : g_radio_state == 2 ? radio_fail_cause(g_radio_rc) : "starting";
}
char g_reset[48];
void __used hz_crash(const uint32_t* frame) {  // frame: the registers the exception stacked (r0-r3 r12 lr pc xpsr)
    watchdog_hw->scratch[0] = HZ_CRASH_MAGIC;
    watchdog_hw->scratch[1] = frame[6];
    watchdog_hw->scratch[2] = frame[5];
    watchdog_reboot(0, 0, 0);
    for (;;) {}
}
static void __attribute__((naked)) hz_hardfault(void) {  // hz_crash(the stack the fault happened on)
    __asm volatile("movs r0, #4\n mov r1, lr\n tst r0, r1\n beq 1f\n mrs r0, psp\n b hz_crash\n"
                   "1: mrs r0, msp\n b hz_crash\n");
}
static bool hz_feed(repeating_timer_t* t) {
    (void)t;
    watchdog_update();
    return true;
}
void hz_guard_start(void) {
    if (watchdog_hw->scratch[0] == HZ_CRASH_MAGIC)
        snprintf(g_reset, sizeof g_reset, " reset=crash pc=0x%08lx lr=0x%08lx", (unsigned long)watchdog_hw->scratch[1],
                 (unsigned long)watchdog_hw->scratch[2]);
    else if (watchdog_enable_caused_reboot())
        snprintf(g_reset, sizeof g_reset, " reset=watchdog");
    watchdog_hw->scratch[0] = 0;
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hz_hardfault);
    static repeating_timer_t feed;
    add_repeating_timer_ms(250, hz_feed, NULL, &feed);
    watchdog_enable(2000, true);  // paused while a debugger halts the chip
}
void hz_guard_stop(void) {  // before a reboot on purpose, or a long time with interrupts off (firmware update)
    watchdog_disable();
    watchdog_hw->scratch[4] = 0;  // the next boot is not a watchdog reset
}
int who_line(char* out, unsigned sz) {
    return snprintf(out, sz, "W board=%s board_id=%u fw=%s git=%s fmt=%u uid=%s radio=%s%s\n", HZ_BOARD_NAME,
                    (unsigned)HZ_BOARD, HZ_FW_TARGET, HZ_FW_GIT, HZ_OTA_FORMAT, g_id, radio_state_name(), g_reset);
}
void who_print(void) {
    char l[208];
    if (who_line(l, sizeof l) > 0) printf("%s", l);
}
uint32_t g_freq_hz = FIXED_FREQ_HZ;  // boot default; changed over the air
hop_t g_hop;                 // current plan
uint8_t hop_tab[HOP_MAX];
void hop_set(const hop_t* h) {
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
bool hop_same(const hop_t* a, const hop_t* b) {
    return a->n == b->n && (!a->n || a->step_khz == b->step_khz);
}
uint32_t chan_hz(uint32_t seq) { return g_hop.n ? (uint32_t)hop_hz(g_freq_hz, &g_hop, hop_k(seq)) : g_freq_hz; }
uint32_t wait_irq(uint32_t timeout_us) {
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
void recover(void) {
    lr20xx_system_set_standby_mode(radio, LR20XX_SYSTEM_STANDBY_MODE_XOSC);
    lr20xx_system_clear_irq_status(radio, LR20XX_SYSTEM_IRQ_ALL_MASK);
    lr20xx_radio_fifo_clear_rx(radio);
    lr20xx_radio_fifo_clear_tx(radio);
}
// A carrier the radio may use: the sub-GHz range or the 2.4 GHz band.
bool vfreq_ok(int64_t hz) {
    return (hz >= VFREQ_MIN_HZ && hz <= VFREQ_MAX_HZ) || (hz >= VFREQ_HF_MIN_HZ && hz <= VFREQ_HF_MAX_HZ);
}
// All hop channels inside one band: the LR2021 sub-GHz range or the 2.4 GHz band (the band plan is the user's to set).
bool hop_valid(uint32_t centre, const hop_t* h) {
    if (!vfreq_ok(centre)) return false;
    if (!h->n) return true;
    if (h->n < 2u || h->n > HOP_MAX || !h->step_khz) return false;
    int64_t lo = hop_hz(centre, h, 0), hi = hop_hz(centre, h, h->n - 1u);
    return vfreq_ok(lo) && vfreq_ok(hi) && (lo >= VFREQ_HF_MIN_HZ) == (hi >= VFREQ_HF_MIN_HZ);
}
bool hop_retune(void) {
    if (!g_hop.n) return radio_retune(g_freq_hz);
    return radio_retune_span(g_freq_hz, (uint32_t)hop_hz(g_freq_hz, &g_hop, 0), (uint32_t)hop_hz(g_freq_hz, &g_hop, g_hop.n - 1u));
}
bool vcmd_valid(const vcmd_t* c) {
    if (!(c->flags & (CMD_F_FREQ | CMD_F_PWR | CMD_F_HOP)) || (c->flags & ~(CMD_F_FREQ | CMD_F_PWR | CMD_F_HOP))) return false;
    if ((c->flags & CMD_F_FREQ) && !vfreq_ok(c->hz)) return false;
    // A 2.4 GHz TX announces RADIO_HF_POWER, below the sub-GHz minimum.
    if ((c->flags & CMD_F_PWR) && (c->pwr < RADIO_HF_POWER || c->pwr > VPWR_MAX)) return false;
    if (c->flags & CMD_F_HOP) {
        hop_t h = {c->hn, c->hstep};
        if (!hop_valid(c->hz, &h)) return false;
    }
    return true;
}
#include "../../format/packet.h"  // CRC-16 and RS(253,237) of the entries (the hosts' code)
const uint8_t LINK_DEFAULT_BLOB[] = {0x86, 0xBB, 0x5B, 0x2C, 0x08};  // N0CALL: the default on both sides
uint16_t g_link;
uint8_t g_link_par[HZ_SW_RS_PAR];
uint16_t link_of(const uint8_t* b, unsigned n) {
    uint32_t h = 2166136261u;
    for (unsigned i = 0; i < n; i++) h = (h ^ b[i]) * 16777619u;
    uint16_t l = (uint16_t)(h ^ (h >> 16));
    return l ? l : 1u;
}
void link_set(uint16_t l) {
    static bool ready;
    if (!ready) { hz_sw_init(); ready = true; }
    g_link = l;
    uint8_t v[HZ_SW_RS_DATA];
    memset(v, 0, sizeof v);
    v[HZ_SW_CRC_AT] = (uint8_t)l; v[HZ_SW_CRC_AT + 1] = (uint8_t)(l >> 8);
    hz_rs_encode(v, HZ_SW_RS_DATA, g_link_par);
}
void link_key_entry(uint8_t* e) {  // e: an entry (packet byte 2 on)
    e[HZ_SW_CRC_AT] ^= (uint8_t)g_link; e[HZ_SW_CRC_AT + 1] ^= (uint8_t)(g_link >> 8);
    for (unsigned i = 0; i < HZ_SW_RS_PAR; i++) e[HZ_SW_RS_DATA + i] ^= g_link_par[i];
}
bool link_ack_ok(const uint8_t* p) { return p[20] == (uint8_t)g_link && p[21] == (uint8_t)(g_link >> 8); }
// CMD:  seq16 | C3 3C A1 | id flags hz[4] pwr eff16 chk | zeros     ACK: seq16 | FF FF FE | same body | zeros
// With CMD_F_HOP (hopping plan, hz = centre; older firmware rejects the unknown flag and ignores the packet):
//   CMD (16 bytes): byte 11 = n, 12..13 = spacing kHz (instead of pwr / eff)
//   ACK / announcement: the body as above (pwr, eff), then 15 = n, 16..17 = spacing kHz, 18 = chk2
void vcmd_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack) {
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
bool vcmd_decode(const uint8_t* p, bool ack, vcmd_t* c) {
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
bool hop_parse(const char* s, uint32_t* centre, hop_t* h) {
    unsigned long hz = 0, n = 0, step = 0;
    int got = sscanf(s, "%lu %lu %lu", &hz, &n, &step);
    if (got < 2) return false;
    if (n == 0) step = 0;
    else if (got < 3 || n > HOP_MAX || step % 1000u || step / 1000u > 65535u) return false;
    *centre = (uint32_t)hz;
    h->n = (uint8_t)n; h->step_khz = (uint16_t)(step / 1000u);
    return hop_valid(*centre, h);
}
void prof_encode(uint8_t* p, uint32_t hz, const hop_t* h, int8_t pwr, uint16_t link) {
    p[0] = 'H'; p[1] = 'Z'; p[2] = 'P'; p[3] = 'R'; p[4] = 3;
    p[5] = (uint8_t)hz; p[6] = (uint8_t)(hz >> 8); p[7] = (uint8_t)(hz >> 16); p[8] = (uint8_t)(hz >> 24);
    p[9] = h->n; p[10] = (uint8_t)h->step_khz; p[11] = (uint8_t)(h->step_khz >> 8); p[12] = (uint8_t)pwr;
    p[13] = (uint8_t)link; p[14] = (uint8_t)(link >> 8);
    uint8_t x = 0x5A;
    for (unsigned i = 4; i < 15; i++) x ^= p[i];
    p[15] = x;
}
uint64_t g_host_rec_us;     // the host's last video record (0: none yet)
uint64_t g_host_seen_us;    // the host's last byte on SPI / USB, polls included (0: never)
volatile uint8_t g_host_err;  // HOST_STAT_SYNC code, cleared by the next video record
int hexval(int c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; }
// Bytes the blob really uses (from its header), 0 if it does not fit in max.
unsigned blob_len(const uint8_t* b, unsigned max) {
    if (!max) return 0;
    unsigned n = b[0] & 0x7Fu;
    unsigned len = (b[0] & 0x80u) ? 1u + 2u * ((n + 2u) / 3u) : 1u + n;
    return len <= max && len <= CALL_MAX ? len : 0;
}
void ident_encode(uint8_t* p, uint16_t seq, const uint8_t* blob, unsigned len, unsigned part) {
    unsigned parts = (len + IDENT_PART - 1u) / IDENT_PART;
    memset(p, 0, CMD_LEN);
    p[0] = (uint8_t)seq; p[1] = (uint8_t)(seq >> 8);
    p[2] = 0x1D; p[3] = 0xC5; p[4] = (uint8_t)(part << 4 | (parts - 1u));
    for (unsigned i = 0; i < IDENT_PART && part * IDENT_PART + i < len; i++) p[5 + i] = blob[part * IDENT_PART + i];
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 14; i++) x ^= p[i];
    p[15] = x;
}
bool ident_check(const uint8_t* p) {
    if (p[2] != 0x1D || p[3] != 0xC5) return false;
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 14; i++) x ^= p[i];
    return x == p[15] && (p[4] >> 4) <= (p[4] & 15u);
}
void vcall_encode(uint8_t* p, uint16_t seq, const vcmd_t* c, bool ack) {
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
bool vcall_decode(const uint8_t* p, bool ack, vcmd_t* c) {
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

// Retries every 2 s; meanwhile on_fail() after each failed try and poll() over and over, so the board still answers.
void radio_start(void (*on_fail)(void), void (*poll)(void)) {
    int rc;
    while ((rc = radio_init()) != 0) {
        printf("radio_init failed (%d)\n", rc);
        printf("ERROR radio_%s step=%d\n", radio_fail_cause(rc), -rc);
        g_radio_rc = rc;
        g_radio_state = 2;
        if (on_fail) on_fail();
        uint64_t until = time_us_64() + 2000000u;
        while (time_us_64() < until) poll();
    }
    g_radio_state = 1;
    g_radio_rc = 0;
}
