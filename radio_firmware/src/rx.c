// The receiver role (hat_rx, ground_unit): acquisition and tracking, packets to the host over USB, the pairing
// profile, commands to the transmitter.
#include "common.h"

#ifndef LIVE_TX
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


// Host -> RP2350 text lines:  F <hz>  set TX+RX frequency (hopping: the centre)   P <half_dbm>  set TX power
//                             L <hz>  retune only this receiver (no link: use it to find a TX that moved)
//                             H <centre_hz> <n> <spacing_hz>  hop plan for TX+RX (n = 0: fixed frequency);
//                               without a link: this receiver only
//                             I <hex blob>  set the transmitter's callsign (sent in parts of 7 bytes)
//                             K  read the pairing profile;  K <centre_hz> <n> <spacing_hz> <pwr_half_dbm> <link>  save
//                               it (flash) and retune to it now
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
    char l[208];
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

// Pairing profile (layout in common.h): the setup site writes the same one to the receiver and the transmitter, so a
// restart of either brings the pair back together. Over-the-air changes (F / H / P) are not saved. Older profile
// versions are not read: the receiver starts on the build's default until it is paired again.
static bool g_prof_saved;
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
// "K" reads the profile; "K <centre_hz> <n> <spacing_hz> <pwr_half_dbm> [<link>]" saves it and retunes to it now
// (without a link the current one is kept).
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
    if (c == 'K') { prof_command(line + 1); return; }
    if (c == 'W') {
        char l[208];
        if (who_line(l, sizeof l) > 0) c_msg("%s", l);
        return;
    }
    if (c == 'I') {  // I <hex blob>: the transmitter's callsign, over the air
        const char* h = line + 1;
        while (*h == ' ') h++;
        uint8_t b[CALL_MAX];
        unsigned n = 0;
        while (h[0] && h[1] && n < CALL_MAX && hexval(h[0]) >= 0 && hexval(h[1]) >= 0) { b[n++] = (uint8_t)(hexval(h[0]) << 4 | hexval(h[1])); h += 2; }
        if (!n || *h || blob_len(b, n) != n) { c_msg("C fail call (I <hex blob>, as format/packet.py encodes it)\n"); return; }
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

// Non-blocking (a few us when idle).
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
static bool rx_send_cmd(uint64_t end, uint16_t seq) {
    // 1 ms after the nominal packet start: the TX listens from right after its previous packet until 0.7 ms before
    // the slot ends, and a 16-byte command takes ~2.2 ms on air.
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
        // Another receiver's hop plan or hopping-centre change: applied here too, in the same slot as the TX.
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

// kind 'P' = passed the CRC, 'E' = failed it (forwarded anyway: the host repairs it or combines receivers' copies).
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
    if (g_hop.n)  // hopping: plan as hop=n,spacing_hz (fixed: no hop=)
        n = snprintf(l, sizeof l, "C cfg freq_hz=%lu pwr=%d locked=%d hop=%u,%lu link=%u\n", (unsigned long)g_freq_hz,
                     g_pwr, locked, g_hop.n, (unsigned long)g_hop.step_khz * 1000u, g_link);
    else
        n = snprintf(l, sizeof l, "C cfg freq_hz=%lu pwr=%d locked=%d link=%u\n", (unsigned long)g_freq_hz, g_pwr, locked,
                     g_link);
    out_append(l, n);
}

static int rssi_now2(void) {
    int16_t r = 0;
    uint8_t h = 0;
    if (lr20xx_radio_common_get_rssi_inst(radio, &r, &h) != LR20XX_STATUS_OK) return 9999;
    return 2 * r - h;
}

// Returns 0 if the packet passed CRC and length checks, otherwise the miss reason character.
static char read_packet(uint32_t irq, uint8_t* p) {
    if (!(irq & LR20XX_SYSTEM_IRQ_RX_DONE)) return 'O';
    if (irq & (LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR)) {
        st.crc++;
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
    const uint32_t boot_hz = g_freq_hz;
    for (;;) {
        // Acquisition: park on one channel until any valid packet arrives.
        st.acq++;
        g_locked = false;
        if (rq.state != RQ_IDLE) {  // link lost during a change: finished if we already sit on the new frequency
            if (rq.c.flags == CMD_F_FREQ && rq.c.hz == g_freq_hz) { c_msg("C applied freq_hz=%lu pwr=%d\n", (unsigned long)g_freq_hz, g_pwr); rq.state = RQ_IDLE; }
            else if (rq.state != RQ_WAIT) { rq.state = RQ_WAIT; }
        }
        // (frequency, plan) pairs tried in turn: current, previous/pending, boot default. Parking is on the centre,
        // which is a channel of every plan.
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
        recover();
        lr20xx_system_set_fs_mode(radio);
        radio_set_freq(g_freq_hz);  // the centre (fixed: the frequency)
        uint32_t base_seq = 0;
        uint64_t base_end = 0;
        // Single-packet RX, re-armed with an empty FIFO after every event: continuous RX accumulates FIFO contents.
        bool armed = false;
        for (;;) {
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
            // a TX announcement heard while parked: it carries the TX's plan (hopping or fixed), take it
            if (p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE) { rx_handle_ack(p); g_hop_adopted = false; }
            if (chan_hz(s16) != g_freq_hz) { st.wrong++; continue; }  // (this plan would not be on the centre now)
            base_seq = s16;
            base_end = now;
            st.ok++;
            queue_packet(base_seq, 'A', 0, 9999, p);
            break;
        }
        if (g_freq_hz != f_start && g_alt_hz == 0) g_alt_hz = f_start;
        if (g_hop.n && (g_freq_hz != f_start || !hop_same(&g_hop, &h_start))) hop_retune();  // calibrate for the band
        if (g_hop.n) c_msg("C found freq_hz=%lu hop n=%u spacing_hz=%lu\n", (unsigned long)g_freq_hz, g_hop.n,
                           (unsigned long)g_hop.step_khz * 1000u);
        else if (g_freq_hz != f_start) c_msg("C found freq_hz=%lu\n", (unsigned long)g_freq_hz);
        g_locked = true;
        // Tracking: one RX window per slot on the predicted channel. Only ~0.3 ms between packets, so after a packet
        // the radio is re-armed at once (~0.15 ms) and everything slow (hex, USB) happens while the next is on air.
        uint32_t next = base_seq + 1, misses = 0;
        bool rx_armed = false;
        uint32_t last_hz = 0;
        // the previous slot's result, formatted after re-arming: kind 0 none, 'P' packet, 'E' failed CRC, else miss
        char pend_kind = 0, pend_mode = 'T';
        uint32_t pend_seq = 0;
        int pend_dt = 0, pend_pre = 9999;
        static uint8_t pend_p[PAYLOAD_LEN];
        // Hopping only: link check over windows of 128 slots.
        unsigned hw_n = 0, hw_ok_c = 0, hw_ok_o = 0;
        bool hop_lost = false;
        while (misses < LOST_MISSES) {
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
            uint64_t end = base_end + (uint64_t)(next - base_seq) * SLOT_US;
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
            if (pend_kind) {
                if (pend_kind == 'P') queue_packet(pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                else if (pend_kind == 'E') queue_packet_kind('E', pend_seq, pend_mode, pend_dt, pend_pre, pend_p);
                else queue_miss(pend_seq, pend_kind, pend_pre);
                pend_kind = 0;
            }
            flush_out();  // USB output happens while the radio is listening
            report(true);
            host_poll();
            int pre2 = rssi_now2();  // channel energy before the packet's preamble
            char reason = 'T';
            uint64_t t_irq = 0;
            // hopping: close the window early after a miss (packets end within a few us of `end`), so there is time
            // to retune before the next packet starts ~0.33 ms after this one ends
            const uint64_t post = g_hop.n ? 120u : RX_POST_US;
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
                    if (g_hop.n) {  // hop to the next slot's channel first (fallback mode after RX is FS)
                        uint32_t hz = chan_hz(next + 1);
                        if (hz != last_hz) { radio_set_freq(hz); last_hz = hz; }
                    }
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
                        if (p[2] == 0xFF && p[3] == 0xFF && p[4] == 0xFE) rx_handle_ack(p);
                    } else {
                        st.wrong++;
                        reason = 'W';
                    }
                }
                break;
            }
            // (no event by the end of the window: the radio stays in RX and catches the next slot's packet)
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
        if (hop_lost) c_msg("C hop link lost (under 25 %% of the slots received): searching\n");
    }
}

// While the radio does not start: answer "W" with the failure cause, so the setup site can tell a missing module
// from a dead board.
static void boot_poll(void) {
    static char buf[32];
    static unsigned n;
    int c = getchar_timeout_us(1000);
    if (c >= 0) {
        if (c == '\n' || c == '\r') {
            if (n == 1 && buf[0] == 'W') who_print();
            n = 0;
        } else if (n < sizeof buf - 1) buf[n++] = (char)c;
        else n = 0;
    }
#ifdef PIN_LED
    gpio_put(PIN_LED, (time_us_32() / 100000u) & 1u);
#endif
}


void rx_main(void) {
    radio_start(NULL, boot_poll);
#ifdef PIN_LED
    gpio_put(PIN_LED, 1);  // LED on = radio running (fast blink while it does not start)
#endif
    prof_load();  // the paired profile, if one was saved (else the build's default frequency)
    // A 2.4 GHz profile needs the HF path: radio.c switches band in a retune, setting the frequency alone does not.
    if (!hop_retune()) printf("radio: retune to the profile's plan failed\n");
    printf("RX id=%s freq_hz=%lu slot_us=%u profile=%s\n", g_id, (unsigned long)g_freq_hz, SLOT_US,
           g_prof_saved ? "saved" : "default");
    run_rx();
}
#endif
