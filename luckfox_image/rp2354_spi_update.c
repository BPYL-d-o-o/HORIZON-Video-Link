/* Firmware update of the TX RP2354A over the SPI host link (radio_firmware/src/tx.c, "Firmware update"): well under a
 * second, where SWD from the Luckfox (OpenOCD driving GPIOs) takes ~14 s. Run with hz_vtx stopped
 * (rp2354_update.sh does it, and falls back to SWD when this fails):
 *
 *   rp2354_spi_update [--dev /dev/spidev0.0] [--hz 8000000] FIRMWARE.bin
 *
 * FIRMWARE.bin is a build's flat image (air_radio.bin: the flash from offset 0). Steps: the RP2354A must answer
 * (status or identity frame on MISO); the start record A5 18 (size, CRC-32); it switches its status frame to the update
 * frame (C3 55 ...) within a slot or two, which a firmware without the update never does (exit 3, "not supported");
 * then 1 KB chunks, each sent until the update frame counts it; then it writes its flash, reads it back and restarts.
 * Exit codes: 0 updated and running again, 1 failed after the flash was written (the board needs SWD), 2 usage or file,
 * 3 not started (no answer, no update support, or failed before the flash was touched: the old firmware runs).
 * Prints EVENT lines and RESULT PASS|FAIL. */
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define CHUNK 1024u
static int fd;
static uint32_t hz = 8000000u;
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }

static int xfer(const uint8_t *tx, uint8_t *rx, unsigned n) {
    struct spi_ioc_transfer t = {.tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = n, .speed_hz = hz,
                                 .bits_per_word = 8};
    return ioctl(fd, SPI_IOC_MESSAGE(1), &t) < 0 ? -1 : 0;
}

static uint32_t crc32(const uint8_t *p, uint32_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}
static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

/* One MISO read: the first good frame starting with C3 `b` (xor ok), copied to f; 0 if none. */
static int frame(uint8_t b, uint8_t *f) {
    static const uint8_t z[72];
    uint8_t rx[72];
    if (xfer(z, rx, sizeof rx)) return 0;
    for (unsigned i = 0; i + 32 <= sizeof rx; i++) {
        if (rx[i] != 0xC3 || rx[i + 1] != b) continue;
        uint8_t x = 0x5A;
        for (unsigned j = 2; j < 31; j++) x ^= rx[i + j];
        if (x == rx[i + 31]) { memcpy(f, rx + i, 32); return 1; }
    }
    return 0;
}
/* Any frame the firmware sends (status C3 3C, identity C3 49, update C3 55). */
static int any_frame(void) {
    uint8_t f[32];
    return frame(0x3C, f) || frame(0x49, f) || frame(0x55, f);
}
/* The update frame, waiting up to `s` seconds: 1 found (state, received count), 0 not seen. */
static int upd_frame(double s, uint8_t *state, uint32_t *rcv) {
    double end = now() + s;
    uint8_t f[32];
    do {
        if (frame(0x55, f)) { *state = f[2]; *rcv = le32(f + 3); return 1; }
        usleep(1000);
    } while (now() < end);
    return 0;
}

int main(int argc, char **argv) {
    const char *dev = "/dev/spidev0.0", *path = NULL;
    for (int a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "--dev") && a + 1 < argc) dev = argv[++a];
        else if (!strcmp(argv[a], "--hz") && a + 1 < argc) hz = (uint32_t)strtoul(argv[++a], NULL, 10);
        else if (argv[a][0] != '-' && !path) path = argv[a];
        else path = NULL, a = argc;
    }
    if (!path) { fprintf(stderr, "usage: %s [--dev DEV] [--hz HZ] FIRMWARE.bin\n", argv[0]); return 2; }
    FILE *fp = fopen(path, "rb");
    if (!fp) { printf("FATAL cannot read %s\nRESULT FAIL\n", path); return 2; }
    static uint8_t img[256 * 1024];
    size_t size = fread(img, 1, sizeof img, fp);
    int more = fgetc(fp) != EOF;
    fclose(fp);
    if (!size || more || size > sizeof img - 256) { printf("FATAL %s: empty or larger than 255 KB\nRESULT FAIL\n", path); return 2; }
    uint32_t crc = crc32(img, (uint32_t)size);
    fd = open(dev, O_RDWR);
    if (fd < 0) { printf("FATAL cannot open %s\nRESULT FAIL\n", dev); return 2; }
    uint8_t mode = 0, bits = 8;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz);
    double t0 = now();
    printf("EVENT image %s size=%zu crc32=%08x spi_hz=%u\n", path, size, crc, hz);

    /* 1. the RP2354A answers */
    int ok = 0;
    for (double end = now() + 2.0; !ok && now() < end;) { ok = any_frame(); if (!ok) usleep(10000); }
    if (!ok) { printf("ERROR rp2354 does not answer on SPI\nRESULT FAIL\n"); return 3; }

    /* 2. start: A5 18 size crc xor; the update frame shows that the firmware took it */
    uint8_t st = 0;
    uint32_t rcv = 0;
    int started = 0;
    for (int tries = 0; tries < 3 && !started; tries++) {
        uint8_t r[16] = {0xA5, 0x18}, junk[16];
        for (int i = 0; i < 4; i++) { r[2 + i] = (uint8_t)(size >> 8 * i); r[6 + i] = (uint8_t)(crc >> 8 * i); }
        uint8_t x = 0x5A;
        for (int i = 2; i < 10; i++) x ^= r[i];
        r[10] = x;
        xfer(r, junk, 11);
        started = upd_frame(0.5, &st, &rcv) && st == 1;
    }
    if (!started) { printf("ERROR rp2354 firmware without SPI update (no update frame)\nRESULT FAIL\n"); return 3; }
    printf("EVENT update started (%.0f ms)\n", (now() - t0) * 1000);

    /* 3. the image in chunks, each until the RP2354A counts it */
    static uint8_t rec[9 + CHUNK], junk[9 + CHUNK];
    uint32_t off = 0;
    unsigned resends = 0;
    while (off < size) {
        uint32_t n = size - off < CHUNK ? (uint32_t)(size - off) : CHUNK;
        rec[0] = 0xA5; rec[1] = 0x19;
        for (int i = 0; i < 4; i++) rec[2 + i] = (uint8_t)(off >> 8 * i);
        rec[6] = (uint8_t)n; rec[7] = (uint8_t)(n >> 8);
        memcpy(rec + 8, img + off, n);
        uint8_t x = 0x5A;
        for (uint32_t i = 2; i < 8 + n; i++) x ^= rec[i];
        rec[8 + n] = x;
        int counted = 0;
        for (int tries = 0; tries < 5 && !counted; tries++) {
            if (tries) resends++;
            xfer(rec, junk, 9 + n);
            double end = now() + 0.2;
            while (now() < end && !counted) {
                if (!upd_frame(0.05, &st, &rcv)) continue;
                if (st != 1) { printf("ERROR rp2354 stopped the update: state 0x%02x after %u bytes\nRESULT FAIL\n", st, rcv); return 3; }
                if (rcv >= off + n) counted = 1;
            }
        }
        if (!counted) { printf("ERROR chunk at %u not taken after 5 tries\nRESULT FAIL\n", off); return 3; }
        off += n;
    }
    printf("EVENT image sent (%.0f ms, %u resends)\n", (now() - t0) * 1000, resends);

    /* 4. the RP2354A checks the CRC, writes its flash, reads it back: state 2, then 3 (or 0x8x) */
    double end = now() + 10.0;
    uint8_t last = 1;
    while (now() < end) {
        if (!upd_frame(0.1, &st, &rcv)) continue;
        last = st;
        if (st == 3 || st >= 0x80) break;
    }
    if (last == 0x82) { printf("ERROR image CRC wrong in the rp2354 (old firmware kept)\nRESULT FAIL\n"); return 3; }
    if (last == 0x81) { printf("ERROR rp2354 timed out taking the image (old firmware kept)\nRESULT FAIL\n"); return 3; }
    if (last != 3) {
        printf("ERROR flash write not confirmed (state 0x%02x): the rp2354 needs SWD\nRESULT FAIL\n", last);
        return 1;
    }
    printf("EVENT flash written and read back (%.0f ms)\n", (now() - t0) * 1000);

    /* 5. it restarts on the new firmware: status frames again */
    usleep(400000);
    ok = 0;
    for (end = now() + 5.0; !ok && now() < end;) {
        uint8_t f[32];
        ok = frame(0x3C, f) || frame(0x49, f);
        if (!ok) usleep(20000);
    }
    if (!ok) { printf("ERROR rp2354 does not answer after the update\nRESULT FAIL\n"); return 1; }
    printf("EVENT rp2354 running again (%.0f ms in total)\nRESULT PASS\n", (now() - t0) * 1000);
    close(fd);
    return 0;
}
