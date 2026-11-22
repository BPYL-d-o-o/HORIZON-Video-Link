/* HORIZON over-the-air format v4: one video packet = one source entry with a payload CRC-16 and an inner Reed-Solomon
 * code. No cross-packet FEC: a lost packet costs the frames whose bytes it
 * carried; the keyframe-free row refresh repaints the picture. Must stay bit-identical to pi5_vtx/swfec.py
 * (checked by luckfox_vtx/test_swfec.py).
 *
 * Radio packet (255 bytes; SF5 / 1 MHz / long-interleaver 4/5; 17.54 ms slots):
 *   seq16 LE (added by the TX RP2350) | entry[253] built here:
 *   entry  0        type   0x00 = video source, 0xFF = filler / command ACK / power announcement (RP2350)
 *          1..2     id16 LE  source number (consecutive: the receiver sees every lost packet)
 *          3        aux      0
 *          4..234   symbol (231 bytes): the next bytes of the video byte stream (access units, each followed by an
 *                   AUD; never padded, a packet can hold the end of one frame and the start of the next)
 *          235..236 CRC-16/MODBUS (poly 0x8005 reflected, init 0xFFFF) of entry[0..235), little endian: a different
 *                   polynomial from the radio's own CRC-16 (CCITT 0x1021), so it also catches what the radio's misses,
 *                   and it rejects wrong Reed-Solomon repairs
 *          237..252 Reed-Solomon parity, RS(253, 237) over GF(256) 0x11d, roots alpha^0..alpha^15, of entry[0..237):
 *                   a packet that failed the radio CRC is repaired at the receiver if it has at most 8 wrong bytes (or
 *                   up to 16 bytes known bad, e.g. where two receivers' copies differ), and kept only if the CRC-16
 *                   then matches. */
#pragma once
#include <stdint.h>
#include <string.h>

#include "hz_fec.h"  /* GF(256) tables (hz_gf_exp / hz_gf_log, hz_fec_init) */

#define HZ_SW_PAYLOAD 255u
#define HZ_SW_ENTRY 253u
#define HZ_SW_HDR 4u
#define HZ_SW_RS_PAR 16u
#define HZ_SW_SYM (HZ_SW_ENTRY - HZ_SW_HDR - 2u - HZ_SW_RS_PAR)   /* 231 */
#define HZ_SW_CRC_AT (HZ_SW_HDR + HZ_SW_SYM)                      /* 235 */
#define HZ_SW_RS_DATA (HZ_SW_CRC_AT + 2u)                          /* 237 */
#define HZ_SW_FILLER 0xFFu

/* ---- CRC-16/MODBUS (poly 0x8005 reflected = 0xA001, init 0xFFFF, no xorout) */
static uint16_t hz_crc16_tab[256];
static inline void hz_crc16_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)i;
        for (int k = 0; k < 8; k++) c = c & 1 ? (uint16_t)(0xA001u ^ (c >> 1)) : (uint16_t)(c >> 1);
        hz_crc16_tab[i] = c;
    }
}
static inline uint16_t hz_crc16(const uint8_t *p, size_t n) {
    uint16_t c = 0xFFFFu;
    for (size_t i = 0; i < n; i++) c = (uint16_t)(hz_crc16_tab[(c ^ p[i]) & 0xFF] ^ (c >> 8));
    return c;
}

/* ---- RS(253,237) systematic encoder: parity = remainder of m(x) * x^16 mod g(x), m[0] = highest power */
static uint8_t hz_rs_gen[HZ_SW_RS_PAR + 1];  /* g(x) coefficients, hz_rs_gen[0] = 1 (x^16) */
static inline void hz_rs_init(void) {
    uint8_t g[HZ_SW_RS_PAR + 1] = {1};
    unsigned deg = 0;
    for (unsigned i = 0; i < HZ_SW_RS_PAR; i++) {  /* g *= (x - alpha^i) */
        uint8_t root = hz_gf_exp[i], ng[HZ_SW_RS_PAR + 1] = {0};
        for (unsigned j = 0; j <= deg; j++) {
            ng[j] ^= g[j];                       /* * x */
            ng[j + 1] ^= hz_gmul(g[j], root);    /* * root (subtraction = xor) */
        }
        deg++;
        memcpy(g, ng, sizeof g);
    }
    memcpy(hz_rs_gen, g, sizeof g);
}
static inline void hz_rs_encode(const uint8_t *msg, size_t n, uint8_t *par) {
    uint8_t r[HZ_SW_RS_PAR] = {0};
    for (size_t i = 0; i < n; i++) {
        uint8_t f = msg[i] ^ r[0];
        memmove(r, r + 1, HZ_SW_RS_PAR - 1);
        r[HZ_SW_RS_PAR - 1] = 0;
        if (f)
            for (unsigned j = 0; j < HZ_SW_RS_PAR; j++) r[j] ^= hz_gmul(hz_rs_gen[j + 1], f);
    }
    memcpy(par, r, HZ_SW_RS_PAR);
}

/* Finishes an entry whose header and symbol are filled: CRC-16 and RS parity. */
static inline void hz_sw_seal(uint8_t *e) {
    uint16_t c = hz_crc16(e, HZ_SW_CRC_AT);
    e[HZ_SW_CRC_AT] = (uint8_t)c; e[HZ_SW_CRC_AT + 1] = (uint8_t)(c >> 8);
    hz_rs_encode(e, HZ_SW_RS_DATA, e + HZ_SW_RS_DATA);
}

static inline void hz_sw_init(void) {
    hz_fec_init();
    hz_crc16_init();
    hz_rs_init();
}

/* ---- Identification (callsign) blob, as pi5_vtx/swfec.py encode_call(): see lora_vlink/src/main.c "Identification".
 *   blob[0] = bit 7 packed, bits 0..6 character count; packed: " A-Z0-9/- " alphabet (A..Z = 1..26, 0..9 = 27..36,
 *   '/' 37, '-' 38, ' ' 39), three characters per 16-bit LE word c0 * 1600 + c1 * 40 + c2; raw: the text bytes.
 *   Returns the blob length (0 = text empty or too long). */
#define HZ_CALL_MAX 112u
static inline int hz_call_code(int c) {
    if (c >= 'A' && c <= 'Z') return c - 'A' + 1;
    if (c >= '0' && c <= '9') return c - '0' + 27;
    return c == '/' ? 37 : c == '-' ? 38 : c == ' ' ? 39 : -1;
}
static inline unsigned hz_call_encode(const char *text, uint8_t *blob) {
    size_t n = strlen(text);
    int packed = n > 0;
    for (size_t i = 0; i < n; i++) if (hz_call_code((unsigned char)text[i]) < 0) packed = 0;
    if (!n || n > 127 || (!packed && 1 + n > HZ_CALL_MAX)) return 0;
    if (!packed) {
        blob[0] = (uint8_t)n;
        memcpy(blob + 1, text, n);
        return 1u + (unsigned)n;
    }
    unsigned len = 1;
    blob[0] = (uint8_t)(0x80u | n);
    for (size_t i = 0; i < n; i += 3) {
        unsigned w = 0;
        for (size_t j = 0; j < 3; j++) w = w * 40u + (i + j < n ? (unsigned)hz_call_code((unsigned char)text[i + j]) : 0u);
        blob[len++] = (uint8_t)w; blob[len++] = (uint8_t)(w >> 8);
    }
    return len <= HZ_CALL_MAX ? len : 0;
}
