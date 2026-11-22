/* SPI link test, Luckfox -> TX RP2350 (lora_vlink firmware, PIO SPI slave): how fast the host link runs cleanly.
 * Run on the Luckfox with hz_vtx stopped (sh /userdata/horizon_vtx/service.sh stop):
 *
 *   spi_speed_test [--dev /dev/spidev0.0] [--mode 0] [--dump] [--requests N] [--max-torn N] HZ [HZ...]
 *     --dump       print a few raw MISO reads
 *     --requests N identity requests in the MOSI test (default 60; each takes ~0.3 s)
 *     --max-torn N allow N bad MISO frames per clock: the RP2350 rewrites its status frame while the host may be
 *                  reading it, so 1..4 of ~3800 frames come out torn at any clock (measured at 4..16.5 MHz); bit
 *                  errors from a too-fast clock corrupt hundreds (the factory test uses 8)
 *
 * For each requested clock (the controller divides its clock by an even number: the real rate is at or below it):
 *   MISO  the RP2350 repeats a 32-byte status frame (C3 3C .. xor), or its identity frame (C3 49 .. xor) after an
 *         identity request (with hz_vtx stopped it can show that for minutes), on MISO. 3000 reads of 72 bytes: every frame
 *         found is checked against its xor; a bad one is a bit error on MISO.
 *   MOSI  --requests identity requests (record A5 1F 01 FE) sent at the clock under test: the RP2350 answers with its identity
 *         frame (C3 49 .. xor) for 300 ms only if it decoded the request correctly; a missing answer is an error on MOSI
 *         (or SCK). The answers are read back at SAFE_HZ, so this measures MOSI alone.
 * Prints EVENT lines per clock and RESULT PASS|FAIL (FAIL if any clock had errors). Changes nothing on the RP2350. */
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int fd;
#define SAFE_HZ 4000000u  /* reads that only check the RP2350's answer (MOSI test) */
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static int xfer(const uint8_t *tx, uint8_t *rx, unsigned n, uint32_t hz) {
    struct spi_ioc_transfer t = {.tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = n, .speed_hz = hz,
                                 .bits_per_word = 8};
    return ioctl(fd, SPI_IOC_MESSAGE(1), &t) < 0 ? -1 : 0;
}

/* frames starting a,b in rx: good (xor ok) and bad (header ok, xor wrong) */
static void frames(const uint8_t *rx, unsigned n, uint8_t a, uint8_t b, unsigned *good, unsigned *bad) {
    for (unsigned i = 0; i + 32 <= n; i++) {
        if (rx[i] != a || rx[i + 1] != b) continue;
        uint8_t x = 0x5A;
        for (unsigned j = 2; j < 31; j++) x ^= rx[i + j];
        if (x == rx[i + 31]) { (*good)++; i += 31; } else (*bad)++;
    }
}

int main(int argc, char **argv) {
    const char *dev = "/dev/spidev0.0";
    uint8_t mode = 0;
    int first = 1, dump = 0;
    unsigned max_torn = 0, requests = 60;  /* --requests: identity requests in the MOSI test */
    unsigned max_torn_unused = 0; (void)max_torn_unused;  /* --max-torn: MISO frames allowed to fail (see the usage note) */
    while (first < argc && argv[first][0] == '-') {
        if (!strcmp(argv[first], "--dev") && first + 1 < argc) dev = argv[first + 1], first += 2;
        else if (!strcmp(argv[first], "--dump")) dump = 1, first++;
        else if (!strcmp(argv[first], "--requests") && first + 1 < argc) requests = (unsigned)atoi(argv[first + 1]), first += 2;
        else if (!strcmp(argv[first], "--max-torn") && first + 1 < argc) max_torn = (unsigned)atoi(argv[first + 1]), first += 2;
        else if (!strcmp(argv[first], "--mode") && first + 1 < argc) mode = (uint8_t)atoi(argv[first + 1]), first += 2;
        else { fprintf(stderr, "usage: %s [--dev DEV] [--mode N] [--dump] [--requests N] [--max-torn N] HZ [HZ...]\n", argv[0]); return 2; }
    }
    if (first >= argc) { fprintf(stderr, "usage: %s [--dev DEV] [--mode N] [--dump] [--requests N] [--max-torn N] HZ [HZ...]\n", argv[0]); return 2; }
    fd = open(dev, O_RDWR);
    if (fd < 0) { printf("FATAL cannot open %s\n", dev); return 1; }
    uint8_t bits = 8;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    int fails = 0;
    for (int a = first; a < argc; a++) {
        uint32_t hz = (uint32_t)strtoul(argv[a], NULL, 10);
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) { printf("ERROR clock %u refused by the controller\n", hz); fails++; continue; }
        uint8_t z[72] = {0}, rx[72], junk[8];
        /* MISO */
        unsigned good = 0, bad = 0, empty = 0, errs = 0;
        double t0 = now();
        for (int i = 0; i < 3000; i++) {
            if (xfer(z, rx, sizeof rx, hz)) { errs++; continue; }
            if (dump && (i < 3 || i == 1500)) {  /* --dump: a few raw back-to-back reads */
                printf("EVENT clock_req=%u read=%d raw=", hz, i);
                for (unsigned k = 0; k < sizeof rx; k++) printf("%02X", rx[k]);
                printf("\n");
            }
            unsigned g = 0, b = 0;
            frames(rx, sizeof rx, 0xC3, 0x3C, &g, &b);
            frames(rx, sizeof rx, 0xC3, 0x49, &g, &b);  /* the identity frame, still shown after an earlier request */
            good += g; bad += b;
            if (!g) empty++;
        }
        double miso_s = now() - t0;
        /* diagnostics: the same reads with a 1 ms pause (the RP2350 realigns its MISO bits when the bus has been idle),
         * and frames found in the stream shifted by 1..7 bits (a bit slip) */
        unsigned gap_good = 0, gap_empty = 0, slipped = 0;
        for (int i = 0; i < 300; i++) {
            usleep(1000);
            if (xfer(z, rx, sizeof rx, hz)) continue;
            unsigned g = 0, b = 0;
            frames(rx, sizeof rx, 0xC3, 0x3C, &g, &b);
            frames(rx, sizeof rx, 0xC3, 0x49, &g, &b);
            gap_good += g;
            if (!g) gap_empty++;
        }
        for (int i = 0; i < 300; i++) {
            if (xfer(z, rx, sizeof rx, hz)) continue;
            for (int s = 1; s < 8; s++) {
                uint8_t sh[71];
                for (unsigned k = 0; k < sizeof sh; k++) sh[k] = (uint8_t)(rx[k] << s | rx[k + 1] >> (8 - s));
                unsigned g = 0, b = 0;
                frames(sh, sizeof sh, 0xC3, 0x3C, &g, &b);
                frames(sh, sizeof sh, 0xC3, 0x49, &g, &b);
                if (g) { slipped++; break; }
            }
        }
        printf("EVENT clock_req=%u diag: reads_with_1ms_pause_without_frame=%u/300 back_to_back_reads_with_a_bit_slipped_frame=%u/300\n",
               hz, gap_empty, slipped);
        (void)gap_good;
        /* MOSI: identity requests */
        static const uint8_t req[8] = {0xA5, 0x1F, 0x01, 0xFE, 0, 0, 0, 0};
        unsigned answered = 0, id_bad = 0;
        uint8_t uid0[8] = {0};
        int have_uid = 0, uid_diff = 0;
        for (unsigned i = 0; i < requests; i++) {
            double wait_end = now() + 1.0;
            while (now() < wait_end) {  /* wait until the previous identity window is over (status frames again) */
                xfer(z, rx, sizeof rx, SAFE_HZ);
                unsigned g = 0, b = 0, gi = 0, bi = 0;
                frames(rx, sizeof rx, 0xC3, 0x3C, &g, &b);
                frames(rx, sizeof rx, 0xC3, 0x49, &gi, &bi);
                if (g && !gi) break;
                usleep(20000);
            }
            xfer(req, junk, sizeof req, hz);
            int ok = 0;
            double end = now() + 0.25;
            while (now() < end && !ok) {
                usleep(5000);
                if (xfer(z, rx, sizeof rx, SAFE_HZ)) continue;
                for (unsigned k = 0; k + 32 <= sizeof rx; k++) {
                    if (rx[k] != 0xC3 || rx[k + 1] != 0x49) continue;
                    uint8_t x = 0x5A;
                    for (unsigned j = 2; j < 31; j++) x ^= rx[k + j];
                    if (x != rx[k + 31]) { id_bad++; continue; }
                    if (!have_uid) { memcpy(uid0, rx + k + 10, 8); have_uid = 1; }
                    else if (memcmp(uid0, rx + k + 10, 8)) uid_diff++;
                    ok = 1;
                    break;
                }
            }
            answered += ok;
        }
        double kbps = 3000.0 * 72 * 8 / miso_s / 1000;
        int pass = !errs && bad <= max_torn && empty <= max_torn && good >= 3500 && answered == requests && !id_bad && !uid_diff;
        if (!pass) fails++;
        printf("EVENT clock_req=%u miso_frames=%u miso_bad=%u miso_reads_without_frame=%u xfer_errors=%u "
               "mosi_answered=%u/%u id_bad=%u id_mismatch=%d throughput_kbps=%.0f %s\n",
               hz, good, bad, empty, errs, answered, requests, id_bad, uid_diff, kbps, pass ? "ok" : "ERRORS");
        fflush(stdout);
    }
    close(fd);
    printf("RESULT %s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
