/* Packet scheduler: cuts the video byte stream into 231-byte symbols and builds one record (A5 5A | entry[253] |
 * xor8(entry)) at a time, just before the RP2350 needs it. A packet's tail is only padded (zeros, skipped by the
 * receiver) when the queue runs dry. Optional block FEC (hz_sched_set_fec). Format: format/README.md "Packet contents"
 * and "Block FEC across packets". Thread-safe. */
#pragma once
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../format/packet.h"

#define HZ_SHARD HZ_SW_SYM                 /* video bytes per packet */
#define HZ_ENTRY HZ_SW_ENTRY
#define HZ_REC_LEN (2u + HZ_ENTRY + 1u)   /* A5 5A entry xor8 = 256 bytes */
#define HZ_FIFO_CAP (1u << 20)            /* bytes of video waiting to be cut into sources */
#define HZ_MARKS 512u
#define HZ_CMD_PERIOD 64u  /* radio_firmware/src/common.h CMD_PERIOD */
#define HZ_PARITY 0x02u    /* | (d - 1) << 4 */
#define HZ_BFEC_KM_MAX 16u /* k, m <= 16 (4 bits each in aux) */
#define HZ_BFEC_D_MAX 8u   /* interleaving depth d <= 8 */

typedef struct {
    pthread_mutex_t lock;
    uint8_t *fifo;
    size_t fifo_len;
    uint32_t next_src;                    /* id of the next source (16 bits on air) */
    /* statistics (reset by the reporter) */
    uint64_t bytes, pad, shards, waits, dropped;
    uint64_t pushed, consumed;
    struct { uint64_t end; double t; } marks[HZ_MARKS];
    unsigned mark_head, mark_n;
    double qdelay_sum, qdelay_max;      /* ms from push until the frame's last byte left in a record */
    unsigned qdelay_n;
    /* block FEC (fec_k = 0: off) */
    unsigned fec_k, fec_m, fec_d;
    uint8_t *fec_syms;                  /* [d][k][HZ_SW_SYM]: the open group's video symbols, block by block */
    unsigned fec_nsym[HZ_BFEC_D_MAX];   /* symbols in each block of the open group */
    uint32_t fec_first;                 /* id of the group's first video packet */
    unsigned fec_n;                     /* video packets in the open group */
    unsigned fec_age;                   /* slots (calls) since the open group began */
    uint8_t *fec_q;                     /* sealed parity entries waiting for slots [d * m][HZ_ENTRY] */
    unsigned fec_q_head, fec_q_n;
    uint64_t parity;                    /* statistics: parity packets sent (reset by the reporter) */
} hz_sched_t;

static inline double hz_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline int hz_sched_init(hz_sched_t *s) {
    memset(s, 0, sizeof *s);
    s->fifo = malloc(HZ_FIFO_CAP);
    if (!s->fifo) return -1;
    pthread_mutex_init(&s->lock, NULL);
    hz_sw_init();
    return 0;
}

/* k, m 1..16, d 1..8. Call before the first record. Returns 0, or -1 (FEC stays off). */
static inline int hz_sched_set_fec(hz_sched_t *s, unsigned k, unsigned m, unsigned d) {
    if (k < 1 || k > HZ_BFEC_KM_MAX || m < 1 || m > HZ_BFEC_KM_MAX || d < 1 || d > HZ_BFEC_D_MAX) return -1;
    uint8_t *syms = malloc((size_t)d * k * HZ_SW_SYM), *q = malloc((size_t)d * m * HZ_ENTRY);
    if (!syms || !q) { free(syms); free(q); return -1; }
    pthread_mutex_lock(&s->lock);
    free(s->fec_syms); free(s->fec_q);
    s->fec_syms = syms; s->fec_q = q;
    s->fec_k = k; s->fec_m = m; s->fec_d = d;
    s->fec_n = s->fec_age = s->fec_q_head = s->fec_q_n = 0;
    memset(s->fec_nsym, 0, sizeof s->fec_nsym);
    pthread_mutex_unlock(&s->lock);
    return 0;
}

/* Appends one frame's bytes. Returns 0, or -1 if the FIFO is full (nothing appended). */
static inline int hz_sched_push(hz_sched_t *s, const uint8_t *b, size_t n) {
    pthread_mutex_lock(&s->lock);
    if (s->fifo_len + n > HZ_FIFO_CAP) { s->dropped += n; pthread_mutex_unlock(&s->lock); return -1; }
    memcpy(s->fifo + s->fifo_len, b, n);
    s->fifo_len += n;
    s->pushed += n;
    if (s->mark_n < HZ_MARKS) {
        unsigned i = (s->mark_head + s->mark_n++) % HZ_MARKS;
        s->marks[i].end = s->pushed; s->marks[i].t = hz_now();
    }
    pthread_mutex_unlock(&s->lock);
    return 0;
}

static inline size_t hz_sched_backlog(hz_sched_t *s) {
    pthread_mutex_lock(&s->lock);
    size_t n = s->fifo_len;
    pthread_mutex_unlock(&s->lock);
    return n;
}

static inline void hz_make_record(uint8_t *rec, const uint8_t *entry) {
    rec[0] = 0xA5; rec[1] = 0x5A;
    memcpy(rec + 2, entry, HZ_ENTRY);
    uint8_t x = 0;
    for (unsigned i = 2; i < 2 + HZ_ENTRY; i++) x ^= rec[i];
    rec[HZ_REC_LEN - 1] = x;
}

static inline int hz_cmd_slot(uint32_t slot) { return (slot & 0xFFFFu) % HZ_CMD_PERIOD == 0; }

/* Queues the parity entries of the open group's d blocks (lock held). */
static inline void hz_sched_close_block(hz_sched_t *s) {
    const unsigned m = s->fec_m, d = s->fec_d;
    for (unsigned i = 0; i < d; i++) {
        unsigned k = s->fec_nsym[i];
        if (!k) continue;
        const uint8_t *data[HZ_BFEC_KM_MAX];
        uint8_t *par[HZ_BFEC_KM_MAX];
        for (unsigned n = 0; n < k; n++) data[n] = s->fec_syms + ((size_t)i * s->fec_k + n) * HZ_SW_SYM;
        uint32_t id = s->fec_first + i;
        for (unsigned j = 0; j < m; j++) {
            uint8_t *e = s->fec_q + (size_t)((s->fec_q_head + s->fec_q_n + j) % (d * m)) * HZ_ENTRY;
            e[0] = (uint8_t)(HZ_PARITY | (d - 1) << 4); e[1] = (uint8_t)id; e[2] = (uint8_t)(id >> 8);
            e[3] = (uint8_t)((k - 1) << 4 | j);
            par[j] = e + HZ_SW_HDR;
        }
        hz_fec_encode(data, k, par, m, HZ_SW_SYM);
        for (unsigned j = 0; j < m; j++) hz_sw_seal(par[j] - HZ_SW_HDR);
        s->fec_q_n += m;
    }
    memset(s->fec_nsym, 0, sizeof s->fec_nsym);
    s->fec_n = 0;
}

/* Lock held, s->fec_q_n > 0. */
static inline void hz_sched_pop_parity(hz_sched_t *s, uint8_t *rec) {
    hz_make_record(rec, s->fec_q + (size_t)s->fec_q_head * HZ_ENTRY);
    s->fec_q_head = (s->fec_q_head + 1) % (s->fec_d * s->fec_m);
    s->fec_q_n--;
    s->parity++;
}

/* Builds the next record into rec[HZ_REC_LEN]. Returns 1, or 0 if nothing is waiting (the RP2350 then sends its idle
 * filler for that slot). With block FEC, waiting parity goes first. */
static inline int hz_sched_next(hz_sched_t *s, uint8_t *rec) {
    uint8_t e[HZ_ENTRY];
    pthread_mutex_lock(&s->lock);
    if (s->fec_q_n) {  /* parity of the last group goes first */
        hz_sched_pop_parity(s, rec);
        pthread_mutex_unlock(&s->lock);
        return 1;
    }
    size_t take = s->fifo_len < HZ_SW_SYM ? s->fifo_len : HZ_SW_SYM;
    if (s->fec_k && s->fec_n) s->fec_age++;
    if (!take) {
        /* close only after k slots: closing at every short gap would send parity for nearly empty blocks */
        if (s->fec_k && s->fec_n && s->fec_age >= s->fec_k) {
            hz_sched_close_block(s);
            hz_sched_pop_parity(s, rec);
            pthread_mutex_unlock(&s->lock);
            return 1;
        }
        s->waits++;
        pthread_mutex_unlock(&s->lock);
        return 0;
    }
    uint32_t id = s->next_src++;
    e[0] = 0; e[1] = (uint8_t)id; e[2] = (uint8_t)(id >> 8); e[3] = 0;
    memcpy(e + HZ_SW_HDR, s->fifo, take);
    memset(e + HZ_SW_HDR + take, 0, HZ_SW_SYM - take);
    memmove(s->fifo, s->fifo + take, s->fifo_len - take);
    s->fifo_len -= take;
    s->consumed += take;
    s->pad += HZ_SW_SYM - take;
    double now = hz_now();
    while (s->mark_n && s->marks[s->mark_head].end <= s->consumed) {
        double d = (now - s->marks[s->mark_head].t) * 1000.0;
        s->qdelay_sum += d; s->qdelay_n++;
        if (d > s->qdelay_max) s->qdelay_max = d;
        s->mark_head = (s->mark_head + 1) % HZ_MARKS;
        s->mark_n--;
    }
    s->bytes += take;
    s->shards++;
    if (s->fec_k) {
        if (!s->fec_n) { s->fec_first = id; s->fec_age = 0; }
        unsigned b = s->fec_n % s->fec_d;
        memcpy(s->fec_syms + ((size_t)b * s->fec_k + s->fec_nsym[b]++) * HZ_SW_SYM, e + HZ_SW_HDR, HZ_SW_SYM);
        if (++s->fec_n == s->fec_k * s->fec_d) hz_sched_close_block(s);
    }
    pthread_mutex_unlock(&s->lock);
    hz_sw_seal(e);
    hz_make_record(rec, e);
    return 1;
}
