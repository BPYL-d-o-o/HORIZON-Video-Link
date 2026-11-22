/* Luckfox <-> TX RP2350 link over SPI: C port of pi5_vtx/live_sender.py `Link` and its feeder.
 *
 * Video records go out on MOSI; the RP2350 (vlive_tx_luckfox) repeats a 32-byte status frame on MISO that
 * every transfer also reads (queue depth, records accepted, counters, TX power, frequency, slot seq16):
 *  0 C3  1 3C  2 queue depth  3 event counter  4..7 records accepted (u32)  8..9 sent  10..11 fail
 *  12..13 late  14..15 data  16..17 idle  18 bad  19 TX power (half dBm, i8)  20..23 freq_hz (u32)
 *  24..25 seq16  26 command slots  27 command-slot IRQs  28 last command-slot flags (bit 7: flips after every
 *  identification the RP2350 sent)  29..30 arrival  31 xor(bytes 2..30) ^ 0x5A
 * The slot period is learned from seq16 (the RP2350 publishes it when a packet starts), and each record is
 * handed over `lead_ms` before the RP2350 takes it, so it carries the freshest bytes. Frequency / power
 * changes are carried out by the two RP2350 firmwares alone; this side only reports them. */
#pragma once
#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "hz_sched.h"

#define HZ_SLOT_S 0.01754         /* firmware SLOT_US (format v4): only the starting guess, the real slot is measured */
#define HZ_SLOT_MARGIN_S 0.0013   /* feeder timing reference: air_s = slot - this (as before) */
#define HZ_AIR_S (HZ_SLOT_S - HZ_SLOT_MARGIN_S)
#define HZ_POLL_LEN 72            /* 8 stale prefetched bytes + 2 full status frames */
#define HZ_DTS 64

typedef struct {
    int fd;
    uint32_t hz;
    pthread_mutex_t spi_lock, lock;
    /* status (under lock) */
    int have_status;
    int depth;
    uint32_t accepted;
    int64_t written;              /* records handed over (in the RP2350's accepted-count space) */
    double acc_change_t, write_t;
    int evcount;                  /* -1 = none yet */
    uint32_t freq_hz;
    int pwr;
    uint8_t cmd[4], cmd_prev[4];
    int ident_toggle;             /* status byte 28 bit 7; -1 = none yet */
    int have_cmd_prev;
    int seq;                      /* -1 = none yet */
    int64_t seq32;                /* seq unwrapped to a running count */
    double slot_t;                /* monotonic time the RP2350 was last seen starting a slot; <0 = unknown */
    double slot_s, air_s;
    int slot_known;
    double dts[HZ_DTS];
    unsigned ndts, dts_head;
    uint16_t prev[5];
    uint8_t prev_bad;
    int have_prev;
    double last_stat_t;
    char last_t[200];
    char events[4][120];
    unsigned nevents;
    uint64_t status_frames, spi_errors;
} hz_link_t;

static __thread double hz_xfer_mid;   /* middle of this thread's last transfer */

static inline void hz_sleep(double s) {
    if (s <= 0) return;
    struct timespec ts = {(time_t)s, (long)((s - (time_t)s) * 1e9)};
    while (nanosleep(&ts, &ts) && errno == EINTR) {}
}

static inline void hz_link_event(hz_link_t *l, const char *msg) {
    if (l->nevents == 4) {  /* keep the newest four */
        memmove(l->events[0], l->events[1], sizeof l->events[0] * 3);
        l->nevents = 3;
    }
    snprintf(l->events[l->nevents], sizeof l->events[0], "%s", msg);
    l->nevents++;
}

static inline int hz_link_open(hz_link_t *l, const char *dev, uint32_t hz, uint8_t mode) {
    memset(l, 0, sizeof *l);
    l->fd = open(dev, O_RDWR);
    if (l->fd < 0) return -1;
    uint8_t bits = 8;
    if (ioctl(l->fd, SPI_IOC_WR_MODE, &mode) || ioctl(l->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) ||
        ioctl(l->fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz)) { close(l->fd); return -1; }
    l->hz = hz;
    pthread_mutex_init(&l->spi_lock, NULL);
    pthread_mutex_init(&l->lock, NULL);
    l->depth = -1; l->written = -1; l->evcount = -1; l->seq = -1; l->slot_t = -1; l->ident_toggle = -1;
    l->slot_s = HZ_SLOT_S; l->air_s = HZ_AIR_S;
    l->last_stat_t = l->acc_change_t = hz_now();
    return 0;
}

/* Full-duplex transfer, CS low for its whole length. */
static inline int hz_xfer(hz_link_t *l, const uint8_t *tx, uint8_t *rx, unsigned n) {
    struct spi_ioc_transfer t;
    memset(&t, 0, sizeof t);
    t.tx_buf = (uintptr_t)tx; t.rx_buf = (uintptr_t)rx; t.len = n; t.speed_hz = l->hz; t.bits_per_word = 8;
    pthread_mutex_lock(&l->spi_lock);
    hz_xfer_mid = hz_now() + n * 4.0 / l->hz;
    int r = ioctl(l->fd, SPI_IOC_MESSAGE(1), &t);
    pthread_mutex_unlock(&l->spi_lock);
    return r < 0 ? -1 : 0;
}

static inline const char *hz_cmd_flags(uint8_t b, char *out, size_t sz) {
    static const struct { uint8_t m; const char *n; } names[] = {{1, "rx_done"}, {2, "crc_err"}, {4, "len_err"},
        {8, "other_irq"}, {16, "bad_len"}, {32, "decoded"}, {64, "not_a_command"}};
    out[0] = 0;
    for (unsigned i = 0; i < 7; i++)
        if (b & names[i].m) { if (out[0]) strncat(out, "+", sz - strlen(out) - 1); strncat(out, names[i].n, sz - strlen(out) - 1); }
    if (!out[0]) snprintf(out, sz, "nothing");
    return out;
}

static inline int hz_dbl_cmp(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

/* Finds the newest valid status frame in the bytes read and publishes it. */
static inline void hz_link_status(hz_link_t *l, const uint8_t *rx, unsigned n) {
    const uint8_t *f = NULL;
    for (unsigned i = 0; i + 32 <= n; i++) {
        if (rx[i] != 0xC3 || rx[i + 1] != 0x3C) continue;
        uint8_t x = 0x5A;
        for (unsigned j = 2; j < 31; j++) x ^= rx[i + j];
        if (x == rx[i + 31]) f = rx + i;
    }
    if (!f) return;
    int depth = f[2], ev = f[3];
    uint32_t acc = (uint32_t)f[4] | (uint32_t)f[5] << 8 | (uint32_t)f[6] << 16 | (uint32_t)f[7] << 24;
    uint16_t c[5];
    for (unsigned i = 0; i < 5; i++) c[i] = (uint16_t)(f[8 + 2 * i] | f[9 + 2 * i] << 8);
    uint8_t bad = f[18];
    int pwr = (int8_t)f[19];
    uint32_t hz = (uint32_t)f[20] | (uint32_t)f[21] << 8 | (uint32_t)f[22] << 16 | (uint32_t)f[23] << 24;
    int seq = f[24] | f[25] << 8;
    /* passed the xor by chance: a frequency outside the sub-GHz range and the 2.4 GHz band, or an impossible LR2021
     * power (0.5 dBm: -39 on the 2.4 GHz path .. 44 on sub-GHz) */
    if (!(((hz >= 100000000u && hz <= 1000000000u) || (hz >= 2400000000u && hz <= 2483500000u)) && pwr >= -39 && pwr <= 44))
        return;
    double now = hz_now();
    pthread_mutex_lock(&l->lock);
    l->status_frames++;
    if (l->written < 0 || acc < l->accepted) l->written = acc;  /* first status, or the RP2350 restarted */
    if (acc != l->accepted) l->acc_change_t = now;
    l->depth = depth; l->accepted = acc; l->have_status = 1;
    if (l->evcount >= 0 && ev != l->evcount) {
        char e[120];
        snprintf(e, sizeof e, "command / change event: freq_hz=%u pwr=%d", hz, pwr);
        hz_link_event(l, e);
    }
    l->evcount = ev; l->freq_hz = hz; l->pwr = pwr;
    memcpy(l->cmd, f + 26, 3);
    l->cmd[2] &= 0x7F;
    l->cmd[3] = 0;
    l->ident_toggle = f[28] >> 7;
    uint16_t cmd_at = (uint16_t)(f[29] | f[30] << 8);
    if (l->seq >= 0 && seq != l->seq) {  /* new slot: the RP2350 publishes seq when the packet starts */
        double t = hz_xfer_mid;
        if (l->slot_t >= 0 && ((seq - l->seq) & 0xFFFF) == 1 && t - l->slot_t > 0.003 && t - l->slot_t < 1.0) {
            l->dts[l->dts_head] = t - l->slot_t;
            l->dts_head = (l->dts_head + 1) % HZ_DTS;
            if (l->ndts < HZ_DTS) l->ndts++;
            if (l->ndts >= 16) {  /* median of the last slots: robust against a late poll */
                double s[HZ_DTS];
                memcpy(s, l->dts, sizeof(double) * l->ndts);
                qsort(s, l->ndts, sizeof(double), hz_dbl_cmp);
                double m = s[l->ndts / 2], sum = 0;
                unsigned nn = 0;
                for (unsigned i = 0; i < l->ndts; i++)
                    if (fabs(s[i] - m) < 0.05 * m) { sum += s[i]; nn++; }  /* drop the odd missed / doubled interval */
                m = sum / nn;
                if (fabs(m - l->slot_s) > 0.0004) {
                    l->slot_s = m;
                    l->air_s = m - HZ_SLOT_MARGIN_S > 0.001 ? m - HZ_SLOT_MARGIN_S : 0.001;
                }
                l->slot_known = 1;
            }
        }
        l->slot_t = t;
    }
    if (l->seq >= 0) l->seq32 += (seq - l->seq) & 0xFFFF;
    else l->seq32 = seq;
    l->seq = seq;
    if (!l->have_prev) {
        memcpy(l->prev, c, sizeof c); l->prev_bad = bad; l->have_prev = 1; l->last_stat_t = now;
    } else if (now - l->last_stat_t >= 5.0) {
        char fl[80];
        const uint8_t *p = l->have_cmd_prev ? l->cmd_prev : l->cmd;
        snprintf(l->last_t, sizeof l->last_t,
                 "T seq=%d sent=%u fail=%u late=%u data=%u idle=%u bad=%u | cmd-slots %u heard %u last=%s at %.2f ms",
                 seq, (uint16_t)(c[0] - l->prev[0]), (uint16_t)(c[1] - l->prev[1]), (uint16_t)(c[2] - l->prev[2]),
                 (uint16_t)(c[3] - l->prev[3]), (uint16_t)(c[4] - l->prev[4]), (uint8_t)(bad - l->prev_bad),
                 (uint8_t)(l->cmd[0] - p[0]), (uint8_t)(l->cmd[1] - p[1]), hz_cmd_flags(l->cmd[2], fl, sizeof fl),
                 (cmd_at * 16 - 2000) / 1000.0);
        memcpy(l->cmd_prev, l->cmd, sizeof l->cmd); l->have_cmd_prev = 1;
        memcpy(l->prev, c, sizeof c); l->prev_bad = bad; l->last_stat_t = now;
    }
    pthread_mutex_unlock(&l->lock);
}

static inline void hz_link_refresh(hz_link_t *l) {
    uint8_t z[HZ_POLL_LEN] = {0}, rx[HZ_POLL_LEN];
    if (hz_xfer(l, z, rx, HZ_POLL_LEN)) {
        pthread_mutex_lock(&l->lock); l->spi_errors++; pthread_mutex_unlock(&l->lock);
        return;
    }
    hz_link_status(l, rx, HZ_POLL_LEN);
}

/* Packets queued on the RP2350 or still in flight; -1 until the RP2350 reports. */
static inline int hz_link_pending(hz_link_t *l) {
    pthread_mutex_lock(&l->lock);
    if (l->depth < 0) { pthread_mutex_unlock(&l->lock); return -1; }
    int64_t in_flight = l->written - (int64_t)l->accepted;
    double now = hz_now(), last = l->acc_change_t > l->write_t ? l->acc_change_t : l->write_t;
    if (in_flight > 64 || (in_flight > 0 && now - last > 0.04)) {
        /* a record was lost on the wire (the RP2350 never counted it): resync rather than stall */
        l->written = l->accepted;
        in_flight = 0;
    }
    int p = l->depth + (int)in_flight;
    pthread_mutex_unlock(&l->lock);
    return p;
}

static inline void hz_link_write(hz_link_t *l, const uint8_t *rec, unsigned n, unsigned records) {
    uint8_t rx[HZ_REC_LEN * 2];
    if (n > sizeof rx || hz_xfer(l, rec, rx, n)) {
        pthread_mutex_lock(&l->lock); l->spi_errors++; pthread_mutex_unlock(&l->lock);
        return;
    }
    hz_link_status(l, rx, n);  /* the same transfer also reads the status frame back */
    pthread_mutex_lock(&l->lock);
    l->written += records;
    l->write_t = hz_now();
    pthread_mutex_unlock(&l->lock);
}

/* TX-side power change: a 5-byte power record (A5 C7 pwr ~pwr pwr^5A, vlive_tx_luckfox) between video records.
 * The RP2350 applies it between two packets, reports it in the status frame (event counter + pwr) and announces it
 * over the air (downlink only), so the receivers show the new value. half_dbm: -19 (-9.5 dBm) .. 44 (22 dBm).
 * Returns 0 once the status frame shows the new power, -1 if it did not within ~3 s (3 tries). */
static inline int hz_link_set_power(hz_link_t *l, int half_dbm) {
    if (half_dbm < -19 || half_dbm > 44) return -1;
    uint8_t p = (uint8_t)(int8_t)half_dbm;
    uint8_t rec[8] = {0xA5, 0xC7, p, (uint8_t)~p, (uint8_t)(p ^ 0x5A), 0, 0, 0}, rx[8];
    for (int tries = 0; tries < 3; tries++) {
        if (hz_xfer(l, rec, rx, sizeof rec)) {
            pthread_mutex_lock(&l->lock); l->spi_errors++; pthread_mutex_unlock(&l->lock);
            continue;
        }
        double t_end = hz_now() + 1.0;
        while (hz_now() < t_end) {
            hz_sleep(0.05);
            pthread_mutex_lock(&l->lock);
            int now_pwr = l->pwr, ok = l->have_status;
            pthread_mutex_unlock(&l->lock);
            if (ok && now_pwr == half_dbm) return 0;
        }
    }
    return -1;
}

/* Frequency / hop plan (record A5 1C hz[4 LE] n step_khz[2 LE] xor(bytes 2..8)^5A; n = 0: one fixed frequency),
 * Horizon Dev pairing: sent at every start from horizon_vtx.conf. The RP2350 ignores it when already on that plan,
 * else announces it downlink and switches in an agreed slot, together with the receivers that hear it. Returns 0 once
 * the status frame reports freq_hz = hz (the hop plan itself is not in the status frame), -1 if not within ~4.5 s. */
static inline int hz_link_set_plan(hz_link_t *l, uint32_t hz, unsigned n, unsigned step_khz, double timeout_s) {
    uint8_t rec[10 + 6] = {0xA5, 0x1C, (uint8_t)hz, (uint8_t)(hz >> 8), (uint8_t)(hz >> 16), (uint8_t)(hz >> 24),
                           (uint8_t)n, (uint8_t)step_khz, (uint8_t)(step_khz >> 8), 0}, rx[sizeof rec];
    uint8_t x = 0x5A;
    for (unsigned i = 2; i < 9; i++) x ^= rec[i];
    rec[9] = x;
    int tries = (int)(timeout_s / 1.5) + 1;
    for (int t = 0; t < tries; t++) {
        if (hz_xfer(l, rec, rx, sizeof rec)) {
            pthread_mutex_lock(&l->lock); l->spi_errors++; pthread_mutex_unlock(&l->lock);
            continue;
        }
        double t_end = hz_now() + 1.5;
        while (hz_now() < t_end) {
            hz_sleep(0.05);
            pthread_mutex_lock(&l->lock);
            uint32_t now_hz = l->freq_hz;
            int ok = l->have_status;
            pthread_mutex_unlock(&l->lock);
            if (ok && now_hz == hz) return 0;
        }
    }
    return -1;
}

/* Callsign for the RP2350 (record A5 1D n blob[n] xor): it keeps it in flash and identifies with it every 5 minutes
 * (blob from hz_call_encode(); n = 0 clears it). */
static inline int hz_link_set_call(hz_link_t *l, const uint8_t *blob, unsigned n) {
    uint8_t rec[4 + HZ_CALL_MAX + 4] = {0xA5, 0x1D, (uint8_t)n}, rx[sizeof rec];
    if (n > HZ_CALL_MAX) return -1;
    uint8_t x = (uint8_t)(0x5A ^ n);
    for (unsigned i = 0; i < n; i++) { rec[3 + i] = blob[i]; x ^= blob[i]; }
    rec[3 + n] = x;
    if (hz_xfer(l, rec, rx, 4 + n + 4)) return -1;
    return 0;
}

/* Why there is no video (record A5 1B code ~code): the RP2350 puts it in its idle packets ("Air status", firmware
 * main.c), so a receiver can say why the picture stopped. 1 no camera detected, 2 the camera / ISP does not start,
 * 3 the encoder does not start, 4 another start-up error. Cleared by the next video record. Sent a few times: the
 * transmitter exits right after it. */
#define HZ_AIR_ERR_NO_CAMERA 1
#define HZ_AIR_ERR_CAMERA    2
#define HZ_AIR_ERR_ENCODER   3
#define HZ_AIR_ERR_OTHER     4
static inline void hz_link_report(hz_link_t *l, int code) {
    uint8_t rec[8] = {0xA5, 0x1B, (uint8_t)code, (uint8_t)~code, 0, 0, 0, 0}, rx[8];
    for (int i = 0; i < 3; i++) { hz_xfer(l, rec, rx, sizeof rec); hz_sleep(0.02); }
}

/* End of transmission / identify now (record A5 1E m ~m): m = 1 identify at the next command slot, m = 2 identify and
 * then stop transmitting until the next video record. Waits until the status frame shows the identification was sent
 * (bit 7 of byte 28 flips). Returns 0, or -1 if it did not within timeout_s. */
static inline int hz_link_ident(hz_link_t *l, int mode, double timeout_s) {
    uint8_t rec[8] = {0xA5, 0x1E, (uint8_t)mode, (uint8_t)~mode, 0, 0, 0, 0}, rx[8];
    pthread_mutex_lock(&l->lock);
    int before = l->ident_toggle;
    pthread_mutex_unlock(&l->lock);
    if (before < 0) return -1;
    double t_end = hz_now() + timeout_s, t_send = 0;
    while (hz_now() < t_end) {
        if (hz_now() >= t_send) {  /* (re)send: a record lost on the wire must not hold the stop */
            if (hz_xfer(l, rec, rx, sizeof rec) == 0) hz_link_status(l, rx, sizeof rx);
            t_send = hz_now() + 1.5;
        }
        hz_sleep(0.02);
        hz_link_refresh(l);
        pthread_mutex_lock(&l->lock);
        int now_t = l->ident_toggle;
        pthread_mutex_unlock(&l->lock);
        if (now_t >= 0 && now_t != before) return 0;
    }
    return -1;
}

/* Identity of the TX RP2350 (lora_vlink/src/main.c "Identity"): record A5 1F 01 FE, after which the RP2350 shows a
 * 32-byte identity frame on MISO instead of the status frame for 300 ms:
 *   0 C3  1 49  2 version  3 board id  4 over-the-air format  5..8 commit (u32 LE)  9 bit 0 dirty, bit 1 (version >= 2)
 *   a callsign is stored (without one the TX stays silent)  10..17 chip id
 *   18..29 build target without "vlive_"  30 (version >= 2) radio state: bits 0..1 = 0 not started yet, 1 running,
 *   2 failing to start; bits 2..7 = |last radio_init() error|   31 xor(2..30) ^ 0x5A
 * Firmware older than 2026-11-20 does not answer. Call before the feeder starts (no status while the frame shows). */
typedef struct {
    unsigned version, board, format;
    uint32_t commit;
    int dirty;
    uint8_t uid[8];
    char target[24];
    int radio;      /* -1 not reported (identity version 1), 0 not started yet, 1 running, 2 failing to start */
    int radio_rc;   /* |radio_init() error| while failing */
    int has_call;   /* -1 not reported (identity version 1), 0 none (cleared: the TX stays silent), 1 set, 2 the N0CALL default */
} hz_who_t;
static inline const char *hz_radio_name(int r) {
    return r == 1 ? "ok" : r == 2 ? "failing" : r == 0 ? "starting" : "unknown";
}

static inline const char *hz_board_name(unsigned board) {
    switch (board) {
    case 1: return "vlink";
    case 2: return "luckfox-carrier";
    case 3: return "video-tx";
    default: return "unknown";
    }
}

static inline int hz_link_whoami(hz_link_t *l, hz_who_t *w, double timeout_s) {
    static const uint8_t req[8] = {0xA5, 0x1F, 0x01, 0xFE, 0, 0, 0, 0};
    uint8_t rx[HZ_POLL_LEN + 32], z[HZ_POLL_LEN + 32] = {0}, junk[8];
    double t_end = hz_now() + timeout_s, t_send = 0;
    memset(w, 0, sizeof *w);
    while (hz_now() < t_end) {
        /* resend: the first record after the SPI device is opened can arrive bit-shifted and be dropped */
        if (hz_now() >= t_send) { hz_xfer(l, req, junk, sizeof req); t_send = hz_now() + 0.1; }
        hz_sleep(0.005);
        if (hz_xfer(l, z, rx, sizeof rx)) continue;
        for (unsigned i = 0; i + 32 <= sizeof rx; i++) {
            const uint8_t *f = rx + i;
            if (f[0] != 0xC3 || f[1] != 0x49) continue;
            uint8_t x = 0x5A;
            for (unsigned j = 2; j < 31; j++) x ^= f[j];
            if (x != f[31]) continue;
            w->version = f[2]; w->board = f[3]; w->format = f[4];
            w->commit = (uint32_t)f[5] | (uint32_t)f[6] << 8 | (uint32_t)f[7] << 16 | (uint32_t)f[8] << 24;
            w->dirty = f[9] & 1;
            memcpy(w->uid, f + 10, 8);
            w->radio = w->version >= 2 ? (int)(f[30] & 3u) : -1;
            w->radio_rc = w->version >= 2 ? (int)(f[30] >> 2) : 0;
            w->has_call = w->version < 2 ? -1 : (f[9] & 4) ? 2 : (f[9] >> 1) & 1;
            char name[13] = {0};
            memcpy(name, f + 18, 12);
            snprintf(w->target, sizeof w->target, "vlive_%s", name);
            return 0;
        }
    }
    return -1;
}

static inline void hz_link_snapshot(hz_link_t *l, double *slot_t, double *slot_s, double *air_s, int *known) {
    pthread_mutex_lock(&l->lock);
    *slot_t = l->slot_t; *slot_s = l->slot_s; *air_s = l->air_s; *known = l->slot_known;
    pthread_mutex_unlock(&l->lock);
}

static inline void hz_set_fifo(int prio) {
    if (prio <= 0) return;
    struct sched_param sp = {.sched_priority = prio};
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
        fprintf(stderr, "hz: warning: cannot set SCHED_FIFO %d\n", prio);
}

/* ---- the two link threads (same timing logic as live_sender.py) ---- */
typedef struct {
    hz_link_t *link;
    hz_sched_t *sched;
    int low;            /* build the next packet when <= this many wait on the RP2350 */
    double lead_s;      /* build each record this long before the RP2350 takes it */
    int prio;           /* SCHED_FIFO priority of the link threads (0 = normal) */
    volatile int stop;
    void (*tap)(void *ctx, const uint8_t *rec, double t);  /* optional: every record handed to the RP2350 */
    void *tap_ctx;
} hz_feeder_t;

static inline void *hz_poll_thread(void *arg) {
    hz_feeder_t *f = arg;
    hz_link_t *l = f->link;
    hz_set_fifo(f->prio);
    while (!f->stop) {
        hz_link_refresh(l);
        /* Poll densely only around the expected start of the next slot (to time it to ~0.3 ms), otherwise rarely. */
        double d = 0.0005, now = hz_now();
        pthread_mutex_lock(&l->lock);
        double st = l->slot_t;
        if (st >= 0 && now - st > 3 * l->slot_s + 0.05) {  /* slots stopped / firmware changed: learn the period again */
            l->slot_known = 0; l->ndts = 0; l->dts_head = 0;
        }
        if (st >= 0 && l->slot_known) {  /* (while the period is unknown, poll densely: a late change must not be missed) */
            double k = ceil((now - st) / l->slot_s - 0.05);
            double lead = st + l->slot_s * (k > 1 ? k : 1) - now;
            d = lead > 0.0025 ? lead - 0.0015 : 0.0003;
        }
        pthread_mutex_unlock(&l->lock);
        hz_sleep(d);
    }
    return NULL;
}

static int hz_trace_feed = -1;
static inline void hz_feed_one(hz_feeder_t *f) {
    uint8_t rec[HZ_REC_LEN];
    if (hz_trace_feed < 0) hz_trace_feed = getenv("HZ_TRACE_FEED") != NULL;
    if (hz_sched_next(f->sched, rec)) {
        if (f->tap) f->tap(f->tap_ctx, rec, hz_now());
        if (hz_trace_feed) fprintf(stderr, "feed %.4f S seq=%d\n", hz_now(), f->link->seq);
        hz_link_write(f->link, rec, HZ_REC_LEN, 1);
    } else if (hz_trace_feed) {
        fprintf(stderr, "feed %.4f IDLE seq=%d\n", hz_now(), f->link->seq);
    }
}

static inline void *hz_feeder_thread(void *arg) {
    hz_feeder_t *f = arg;
    hz_link_t *l = f->link;
    hz_set_fifo(f->prio);
    /* The RP2350 takes one record per slot, at the end of the previous packet. A record handed over any earlier
     * is padded with whatever was ready then and wastes the slot; so each record is built just before that
     * moment (lead), from everything that has queued up by then. */
    double done_st = -1;
    long done_n = -1;
    while (!f->stop) {
        double now = hz_now(), st, slot_s, air_s;
        int known;
        hz_link_snapshot(l, &st, &slot_s, &air_s, &known);
        if (st < 0 || now - st > 0.5) {  /* slot phase unknown: hand over as soon as the queue is empty */
            int p = hz_link_pending(l);
            uint8_t rec[HZ_REC_LEN];
            if (p >= 0 && p <= f->low && hz_sched_next(f->sched, rec)) {
                if (f->tap) f->tap(f->tap_ctx, rec, hz_now());
                hz_link_write(l, rec, HZ_REC_LEN, 1);
            } else hz_sleep(0.0005);
            continue;
        }
        long n = (long)((now - st) / slot_s);
        double w = st + n * slot_s + air_s - f->lead_s;
        int is_done = st == done_st && n == done_n;
        if (is_done || now < w - 0.0006) {
            if (is_done) hz_sleep(0.001);
            else {
                double s = w - now - 0.0005;
                hz_sleep(s < 0.0003 ? 0.0003 : (s > 0.004 ? 0.004 : s));
            }
            continue;
        }
        hz_link_refresh(l);
        done_st = st; done_n = n;
        int p = hz_link_pending(l);
        if (p < 0 || p > f->low) {
            if (hz_trace_feed > 0) fprintf(stderr, "feed %.4f SKIP pending=%d seq=%d\n", hz_now(), p, l->seq);
            continue;
        }
        hz_feed_one(f);
    }
    return NULL;
}
