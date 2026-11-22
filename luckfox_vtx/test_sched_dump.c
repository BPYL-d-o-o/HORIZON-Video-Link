/* Test driver for hz_swfec.h / hz_sched.h, format v4 (run by test_swfec.py, which checks the records with
 * pi5_vtx/swfec.py). stdin, one command per line:
 *   S              new scheduler -> OK
 *   P <hex>        push bytes    -> OK / FULL
 *   N              next record   -> hex of the 256-byte record, or "-" (nothing to send)
 *   R <hex>        RS parity of the given bytes -> hex (16 bytes)
 *   C <hex>        CRC-16/MODBUS of the given bytes -> 4 hex digits
 *   K <text>       callsign blob (hz_call_encode) -> hex, or "-"
 *   I <n>          set the next source id -> OK
 *   F <k> <m> <d>  block FEC on (hz_sched_set_fec) -> OK / BAD */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hz_sched.h"

static size_t unhex(const char *s, uint8_t *out, size_t max) {
    size_t n = 0;
    while (s[0] && s[1] && n < max) {
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}
static void puthex(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
}

int main(void) {
    static char line[1 << 22];
    static uint8_t buf[1 << 21];
    static hz_sched_t s;
    int have = 0;
    hz_sw_init();
    while (fgets(line, sizeof line, stdin)) {
        line[strcspn(line, "\r\n")] = 0;
        if (line[0] == 'S') {
            if (have) { free(s.fifo); free(s.fec_syms); free(s.fec_q); }
            if (hz_sched_init(&s)) { puts("INITFAIL"); fflush(stdout); continue; }
            have = 1;
            puts("OK");
        } else if (line[0] == 'P') {
            size_t n = unhex(line + 2, buf, sizeof buf);
            puts(hz_sched_push(&s, buf, n) ? "FULL" : "OK");
        } else if (line[0] == 'N') {
            uint8_t rec[HZ_REC_LEN];
            if (hz_sched_next(&s, rec)) { puthex(rec, HZ_REC_LEN); putchar('\n'); }
            else puts("-");
        } else if (line[0] == 'R') {
            size_t n = unhex(line + 2, buf, sizeof buf);
            uint8_t par[HZ_SW_RS_PAR];
            hz_rs_encode(buf, n, par);
            puthex(par, HZ_SW_RS_PAR);
            putchar('\n');
        } else if (line[0] == 'C') {
            size_t n = unhex(line + 2, buf, sizeof buf);
            printf("%04x\n", hz_crc16(buf, n));
        } else if (line[0] == 'I') {
            s.next_src = (uint32_t)strtoul(line + 2, NULL, 10);
            puts("OK");
        } else if (line[0] == 'F') {
            unsigned k = 0, m = 0, d = 0;
            puts(sscanf(line + 2, "%u %u %u", &k, &m, &d) == 3 && hz_sched_set_fec(&s, k, m, d) == 0 ? "OK" : "BAD");
        } else if (line[0] == 'K') {
            uint8_t b[HZ_CALL_MAX];
            unsigned n = hz_call_encode(line + 2, b);
            if (n) { puthex(b, n); putchar('\n'); } else puts("-");
        } else {
            return 2;
        }
        fflush(stdout);
    }
    return 0;
}
