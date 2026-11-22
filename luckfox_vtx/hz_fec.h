/* Systematic Cauchy Reed-Solomon erasure code over GF(256), polynomial 0x11d.
 * Same code as pi5_vtx/blockfec.py (the Pi transmitter): bit-identical shards.
 * K data shards + M parity shards; any K of the K+M recover the data.
 * Parity row j, column i: C[j][i] = 1 / ((K + j) XOR i).
 * Header-only; call hz_fec_init() once before use. Not thread-safe during init. */
#pragma once
#include <stdint.h>
#include <string.h>

#define HZ_FEC_MAX 16  /* k limit of hz_fec_decode; block FEC (hz_sched.h) uses hz_fec_encode, hz_swfec.h the GF(256) tables */

static uint8_t hz_gf_exp[512];
static int hz_gf_log[256];

static inline void hz_fec_init(void) {
    unsigned x = 1;
    for (int i = 0; i < 255; i++) {
        hz_gf_exp[i] = (uint8_t)x;
        hz_gf_log[x] = i;
        x <<= 1;
        if (x & 0x100) x ^= 0x11D;
    }
    for (int i = 255; i < 510; i++) hz_gf_exp[i] = hz_gf_exp[i - 255];
    hz_gf_exp[510] = hz_gf_exp[511] = 0;
    hz_gf_log[0] = 0;
}

static inline uint8_t hz_gmul(uint8_t a, uint8_t b) {
    if (!a || !b) return 0;
    return hz_gf_exp[hz_gf_log[a] + hz_gf_log[b]];
}

static inline uint8_t hz_ginv(uint8_t a) { return hz_gf_exp[255 - hz_gf_log[a]]; }

/* acc ^= c * v over n bytes */
static inline void hz_mul_acc(uint8_t *acc, uint8_t c, const uint8_t *v, unsigned n) {
    if (!c) return;
    const int lc = hz_gf_log[c];
    for (unsigned i = 0; i < n; i++)
        if (v[i]) acc[i] ^= hz_gf_exp[hz_gf_log[v[i]] + lc];
}

static inline uint8_t hz_cauchy(unsigned k, unsigned j, unsigned i) { return hz_ginv((uint8_t)((k + j) ^ i)); }

/* data[k][n] -> parity[m][n] */
static inline void hz_fec_encode(const uint8_t *const *data, unsigned k, uint8_t *const *parity, unsigned m, unsigned n) {
    for (unsigned j = 0; j < m; j++) {
        memset(parity[j], 0, n);
        for (unsigned i = 0; i < k; i++) hz_mul_acc(parity[j], hz_cauchy(k, j, i), data[i], n);
    }
}

/* Gauss-Jordan inverse of a k x k matrix over GF(256), lowest-index nonzero pivot. Returns 0 on success. */
static inline int hz_fec_invert(uint8_t a[HZ_FEC_MAX][2 * HZ_FEC_MAX], unsigned n) {
    for (unsigned col = 0; col < n; col++) {
        unsigned piv = col;
        while (piv < n && !a[piv][col]) piv++;
        if (piv == n) return -1;
        if (piv != col)
            for (unsigned c = 0; c < 2 * n; c++) { uint8_t t = a[col][c]; a[col][c] = a[piv][c]; a[piv][c] = t; }
        uint8_t inv = hz_ginv(a[col][col]);
        for (unsigned c = 0; c < 2 * n; c++) a[col][c] = hz_gmul(a[col][c], inv);
        for (unsigned r = 0; r < n; r++) {
            if (r == col || !a[r][col]) continue;
            uint8_t f = a[r][col];
            for (unsigned c = 0; c < 2 * n; c++) a[r][c] ^= hz_gmul(f, a[col][c]);
        }
    }
    return 0;
}

/* Erasure decode, same policy as the receivers: present[idx] says whether shard idx (0..k+m-1) arrived,
 * shards[idx] points to it. Fills out[0..k-1] (n bytes each) with the data shards.
 * Returns 0 on success, -1 if fewer than k shards are present. */
static inline int hz_fec_decode(const uint8_t *const *shards, const uint8_t *present, unsigned k, unsigned m, unsigned n,
                         uint8_t *const *out) {
    unsigned have = 0, all_data = 1;
    for (unsigned i = 0; i < k + m; i++) have += present[i] ? 1u : 0u;
    for (unsigned i = 0; i < k; i++) all_data &= present[i] ? 1u : 0u;
    if (all_data) {
        for (unsigned i = 0; i < k; i++) memcpy(out[i], shards[i], n);
        return 0;
    }
    if (have < k) return -1;
    unsigned use[HZ_FEC_MAX], u = 0;
    for (unsigned i = 0; i < k + m && u < k; i++) if (present[i]) use[u++] = i;  /* sorted(shards)[:k] */
    uint8_t a[HZ_FEC_MAX][2 * HZ_FEC_MAX];
    memset(a, 0, sizeof a);
    for (unsigned r = 0; r < k; r++) {
        unsigned idx = use[r];
        for (unsigned c = 0; c < k; c++) a[r][c] = idx < k ? (uint8_t)(c == idx) : hz_cauchy(k, idx - k, c);
        a[r][k + r] = 1;
    }
    if (hz_fec_invert(a, k)) return -1;
    for (unsigned i = 0; i < k; i++) {
        if (present[i]) { memcpy(out[i], shards[i], n); continue; }
        memset(out[i], 0, n);
        for (unsigned r = 0; r < k; r++) hz_mul_acc(out[i], a[i][k + r], shards[use[r]], n);
    }
    return 0;
}
