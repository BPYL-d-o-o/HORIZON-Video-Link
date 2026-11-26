// The transmitter role (LIVE_TX builds: hat_tx, air_radio): the host link (SPI or PIO SPI), the slot loop,
// identification, the firmware update over the host link.
#include "common.h"

// Host -> RP2350 video record: A5 5A | entry[253] (format/README.md "Packet contents") | xor8(entry).
// RP2350 -> USB, every 5 s: "T seq=.. sent=.. fail=.. late=.. data=.. idle=.. bad=..".
#ifdef LIVE_TX
#define LQ_CAP  24u
#define REC_LEN (2u + ENTRY + 1u)
static uint8_t lq[LQ_CAP][ENTRY];
static uint32_t lq_head, lq_tail, rec_ok, rec_bad;
static uint8_t rec[REC_LEN];
static unsigned rec_n;

// A DMA channel drains the SPI RX FIFO into this ring, so no byte is lost while the CPU is busy with the radio.
// poll_host() only parses as much as fits the queue: the host paces itself from the status frame.
#define HOST_RING_BITS 12u
#define HOST_RING_SIZE (1u << HOST_RING_BITS)
static uint8_t host_ring[HOST_RING_SIZE] __attribute__((aligned(HOST_RING_SIZE)));
static unsigned host_dma, host_rd, host_tx_dma;
static uint32_t g_host_bytes;                   // bytes taken from the host ring
static void host_status_publish(void);

// Status frame, 32 bytes, repeated endlessly on MISO by a second DMA channel; the host finds it by sync + checksum
// (a frame torn by an update fails the checksum):
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
// The power at the LR2021 pin (half dBm): on the 2.4 GHz path radio.c keeps RADIO_HF_POWER whatever is asked.
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

// No callsign stored: identify as "N0CALL" (packed blob, not saved). An explicitly cleared callsign still means silence.
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
static void call_link(void) {
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
// Pairing profile, transmitter side (hat_tx): the receiver's "K" lines, so the setup site pairs the HAT over USB like
// the Ground. Host plan / power records override the saved profile until the next restart. The stored link is
// for reading back only; the TX's link comes from its callsign.
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
// Interrupts off ~50 ms: call between two packets.
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

// Firmware update over the host link (luckfox_image/rp2354_spi_update.c); SWD is the fallback.
//   host -> TX  A5 18 size[4 LE] crc32[4 LE] xor(2..9)^5A         start: the TX stops transmitting and takes the image
//               A5 19 off[4 LE] n[2 LE] data[n] xor(2..7+n)^5A    one chunk (n <= UPD_CHUNK), in order: off = bytes
//                                                                  taken so far (a repeated chunk is ignored)
//   TX -> host  the status frame becomes C3 55 state rcv[4 LE] size[4 LE] crc32[4 LE] 0.. xor(2..30)^5A
//               state 1 taking the image, 2 writing the flash, 3 written and read back correctly (it restarts on the new
//               firmware 0.3 s later); failed: 0x81 no data for 5 s, 0x82 image CRC wrong, 0x83 read-back wrong after 3
//               tries. 0x81 / 0x82 restart on the old firmware (the flash was not touched); 0x83 leaves no usable
//               firmware: the Luckfox flashes it over SWD.
// The image (a build's .bin from flash offset 0) is checked in RAM (CRC-32 as zlib) before the flash is touched; only
// the sectors it covers are erased, so the callsign and pairing profile at the end of the flash stay.
#define HOST_UPD_SYNC 0x18u
#define HOST_UPD_DATA 0x19u
#define UPD_MAX       (256u * 1024u)
#define UPD_CHUNK     1024u
#define UPD_ST_TAKING   1u
#define UPD_ST_WRITING  2u
#define UPD_ST_DONE     3u
#define UPD_ST_TIMEOUT  0x81u
#define UPD_ST_BAD_CRC  0x82u
#define UPD_ST_BAD_READ 0x83u
static uint8_t upd_buf[UPD_MAX] __attribute__((aligned(4)));
#ifdef HZ_HOST_PIO
static uint32_t host_junk, host_resyncs;  // (defined with the PIO host link below)
static void host_check_resync(void);
static void host_realign_if_idle(void);
#endif
static volatile bool g_upd_req;
static uint32_t g_upd_size, g_upd_crc;

// RAM-resident helpers: they also run while the flash is erased. No library calls (memcpy / memset live in flash):
// volatile pointers keep the compiler from turning the loops into such calls.
static uint32_t __no_inline_not_in_flash_func(upd_crc32)(const volatile uint8_t* p, uint32_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (unsigned k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}
static void __no_inline_not_in_flash_func(upd_frame)(uint8_t state, uint32_t rcv) {
    volatile uint8_t f[STATUS_LEN];
    for (unsigned i = 0; i < STATUS_LEN; i++) f[i] = 0;
    f[0] = 0xC3; f[1] = 0x55; f[2] = state;
    for (unsigned i = 0; i < 4; i++) {
        f[3 + i] = (uint8_t)(rcv >> (8 * i));
        f[7 + i] = (uint8_t)(g_upd_size >> (8 * i));
        f[11 + i] = (uint8_t)(g_upd_crc >> (8 * i));
    }
    uint8_t x = 0x5A;
    for (unsigned i = 2; i <= 30; i++) x ^= f[i];
    f[31] = x;
    volatile uint8_t* s = host_status;
    for (unsigned i = 0; i < STATUS_LEN; i++) s[i] = f[i];
}
// Never returns. Interrupts are off.
static void __no_inline_not_in_flash_func(upd_write)(uint32_t size, rom_reboot_fn reboot) {
    uint32_t span = (size + FLASH_SECTOR_SIZE - 1u) & ~(FLASH_SECTOR_SIZE - 1u);
    uint32_t pages = (size + FLASH_PAGE_SIZE - 1u) & ~(FLASH_PAGE_SIZE - 1u);
    volatile uint8_t* b = upd_buf;
    for (uint32_t i = size; i < pages; i++) b[i] = 0xFF;
    uint8_t st = UPD_ST_BAD_READ;
    for (unsigned tries = 0; tries < 3; tries++) {
        flash_range_erase(0, span);
        flash_range_program(0, upd_buf, pages);
        if (upd_crc32((const volatile uint8_t*)XIP_NOCACHE_NOALLOC_BASE, size) == g_upd_crc) { st = UPD_ST_DONE; break; }
    }
    upd_frame(st, size);
    uint32_t t0 = timer_hw->timerawl;
    while (timer_hw->timerawl - t0 < 300000u) {}  // the host reads the result
    reboot(REBOOT2_FLAG_REBOOT_TYPE_NORMAL, 10, 0, 0);
    for (;;) {}
}

// Never returns; on a timeout or a wrong CRC it restarts on the firmware it has.
static void fw_update(void) {
    uint32_t size = g_upd_size;
    if (g_radio_state == 1) lr20xx_system_set_standby_mode(radio, LR20XX_SYSTEM_STANDBY_MODE_RC);  // radio off
    printf("C update: taking %lu bytes, crc32 %08lx\n", (unsigned long)size, (unsigned long)g_upd_crc);
    static uint8_t r[8u + UPD_CHUNK + 1u];  // the record being parsed
    unsigned n = 0;
    uint32_t rcv = 0;
    uint8_t st = UPD_ST_TAKING;
    upd_frame(UPD_ST_TAKING, 0);
    uint64_t last = time_us_64();
    while (rcv < size) {
        if (time_us_64() - last > 5000000u) { st = UPD_ST_TIMEOUT; break; }
        unsigned wr = (unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
        while (host_rd != wr) {
            uint8_t c = host_ring[host_rd];
            host_rd = (host_rd + 1) & (HOST_RING_SIZE - 1);
            if (n == 0) {
                if (c == 0xA5) r[n++] = c;
#ifdef HZ_HOST_PIO
                else if (c != 0) host_junk++;
#endif
                continue;
            }
            if (n == 1) { if (c == HOST_UPD_DATA) r[n++] = c; else n = c == 0xA5 ? 1u : 0u; continue; }
            r[n++] = c;
            if (n < 8) continue;
            unsigned len = r[6] | (unsigned)r[7] << 8;
            if (!len || len > UPD_CHUNK) { n = 0; continue; }
            if (n < 9u + len) continue;
            n = 0;
            uint8_t x = 0x5A;
            for (unsigned i = 2; i < 8u + len; i++) x ^= r[i];
            if (x != r[8u + len]) continue;
            uint32_t off = (uint32_t)r[2] | (uint32_t)r[3] << 8 | (uint32_t)r[4] << 16 | (uint32_t)r[5] << 24;
            if (off != rcv || off + len > size) continue;  // a repeat (the host did not see the last answer) or a gap
            memcpy(upd_buf + off, r + 8, len);
            rcv += len;
            last = time_us_64();
#ifdef HZ_HOST_PIO
            host_junk = 0;
#endif
            upd_frame(UPD_ST_TAKING, rcv);
        }
#ifdef HZ_HOST_PIO
        uint32_t rs = host_resyncs;
        host_check_resync();
        if (rs != host_resyncs) n = 0;
        host_realign_if_idle();
#endif
    }
    if (st == UPD_ST_TAKING && upd_crc32(upd_buf, size) != g_upd_crc) st = UPD_ST_BAD_CRC;
    if (st != UPD_ST_TAKING) {
        printf("C update failed: state 0x%02x after %lu bytes; restarting on this firmware\n", st, (unsigned long)rcv);
        upd_frame(st, rcv);
        sleep_ms(500);
        hz_guard_stop();
        rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_NORMAL, 10, 0, 0);
        for (;;) {}
    }
    printf("C update: image ok, writing the flash\n");
    upd_frame(UPD_ST_WRITING, rcv);
    sleep_ms(5);
    rom_reboot_fn reboot = (rom_reboot_fn)rom_func_lookup(ROM_FUNC_REBOOT);  // looked up while the flash still runs
    hz_guard_stop();  // the flash write runs with interrupts off for seconds
    (void)save_and_disable_interrupts();
    upd_write(size, reboot);
}

// Small host records (not video): bytes needed in total, 0 = not known yet.
static unsigned host_small_len(const uint8_t* r, unsigned n) {
    switch (r[1]) {
    case HOST_UPD_SYNC: return 11u;
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
           c == HOST_PLAN_SYNC || c == HOST_STAT_SYNC || c == HOST_UPD_SYNC;
}
// False if the record is malformed.
static bool host_small_record(const uint8_t* r) {
    if (r[1] == HOST_UPD_SYNC) {  // A5 18 size[4] crc32[4] xor: firmware update
        uint8_t x = 0x5A;
        for (unsigned i = 2; i < 10; i++) x ^= r[i];
        uint32_t size = (uint32_t)r[2] | (uint32_t)r[3] << 8 | (uint32_t)r[4] << 16 | (uint32_t)r[5] << 24;
        if (x != r[10] || !size || size > UPD_MAX - FLASH_PAGE_SIZE || size > PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE) return false;
        g_upd_size = size;
        g_upd_crc = (uint32_t)r[6] | (uint32_t)r[7] << 8 | (uint32_t)r[8] << 16 | (uint32_t)r[9] << 24;
        g_upd_req = true;
        return true;
    }
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
        if (hz == g_freq_hz && hop_same(&h, &g_hop)) return true;  // every start sends it
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

#ifdef HZ_HOST_PIO
// SPI slave in PIO (air_radio), mode 0. Two state machines, each fed / drained by an endless DMA channel like the
// SPI1 version below.
//   RX (MOSI -> ring):   0 wait 1 CS | 1 wait 0 CS | 2 mov isr,null | 3 wait 1 SCK | 4 in pins,1 | 5 wait 0 SCK | 6 jmp pin(CS) 1
//                        wrap 6 -> 3. Autopush every 8 bits (MSB first). A deselect (CS high) seen after a bit
//                        clears any partial byte, so every select starts on a byte boundary.
//   TX (status -> MISO): 0 out pins,1 | 1 wait 1 SCK | 2 wait 0 SCK | 3 out pins,1   wrap 3 -> 1. Autopull every 8 bits;
//                        the next bit is put on MISO after each falling edge. The host finds the frame by its sync.
// Once CS has stayed high for HOST_IDLE_US, the TX state machine is restarted (FIFO and DMA untouched): an
// interrupted transfer would otherwise leave the status frame bits off. Only TX: restarting RX loses records.
// RX alignment is guarded by host_check_resync().
static PIO host_pio = pio0;
static unsigned host_rx_sm, host_tx_sm, host_rx_off, host_tx_off;
static uint32_t host_junk;  // unexpected (non-zero, non-record) bytes since the last good record
static uint32_t host_resyncs;

static dma_channel_config host_rx_cfg, host_tx_cfg;

// The DMA channels are stopped before the FIFOs are cleared: a FIFO cleared under a running channel leaves its
// DREQ credit out of step and the channel stalls.
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

// Bit-slip guard (e.g. an SCK glitch while the host boots): only junk arrives, so restart both state machines while
// the host is deselected.
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

// Identity frame (common.h "Identity").
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
    strncpy((char*)f + 18, HZ_FW_TARGET, 12);
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

// Status LED (PIN_LED, the red LED by the RP2354A):
//   on            sending video from the Luckfox
//   slow blink    radio running, no video from the Luckfox (1 s on, 1 s off): the Luckfox is starting, or its
//                 camera or transmitter program is down (its own LED says which)
//   double blink  silent: no callsign set, or stopped by the Luckfox (two short flashes every 2 s)
//   fast blink    the radio module does not start (5 per second)
//   off           no power, or the RP2354A is not running
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
    // End the identity window even when nothing else publishes a status (transmitter stopped or silent).
    if (g_who_until && time_us_64() >= g_who_until) host_status_publish();
    unsigned wr =(unsigned)((uintptr_t)dma_channel_hw_addr(host_dma)->write_addr - (uintptr_t)host_ring) & (HOST_RING_SIZE - 1);
    int avail = (int)((wr - host_rd) & (HOST_RING_SIZE - 1));
    if (avail > 64) avail = 64;  // bound the time spent per call
    if (avail > 0) g_host_seen_us = time_us_64();
    g_host_bytes += (uint32_t)avail;
    uint32_t ok0 = rec_ok;
    // Non-record bytes (status-poll zeros) are always consumed; a record start waits in the ring until the queue has room.
    for (int k = 0; k < avail; k++) {
        int c = host_ring[host_rd];
        if (rec_n == 0) {
            if (c == 0xA5) { if (lq_head - lq_tail >= LQ_CAP) break; rec[rec_n++] = 0xA5; }
#ifdef HZ_HOST_PIO
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
#ifdef HZ_HOST_PIO
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
#ifdef HZ_HOST_PIO
            host_junk += 32u;
#endif
            continue;
        }
        memcpy(lq[lq_head % LQ_CAP], rec + 2, ENTRY);
        lq_head++;
        rec_ok++;
        g_host_rec_us = time_us_64(); g_host_err = 0;
#ifdef HZ_HOST_PIO
        host_junk = 0;
#endif
    }
    if (rec_ok != ok0) host_status_publish();
#ifdef HZ_HOST_PIO
    host_check_resync();
    host_realign_if_idle();
#endif
}

#define HOST_QUIET_US 100u  // no USB work this close to a packet start: start time stays exact


// TX USB text: "I" identify now, "I <hex blob>" set the callsign (blob as format/packet.py / hz_vtx encode it), "I -" clear it.
static void tx_call_text(const char* line) {
    const char* h = line + 1;
    while (*h == ' ') h++;
    if (!*h) { if (g_ident_req < 1) g_ident_req = 1; printf("C ident requested\n"); return; }
    if (*h == '-') { uint8_t z = 0; call_set(&z, 0); return; }
    uint8_t b[CALL_MAX];
    unsigned n = 0;
    while (h[0] && h[1] && n < CALL_MAX && hexval(h[0]) >= 0 && hexval(h[1]) >= 0) { b[n++] = (uint8_t)(hexval(h[0]) << 4 | hexval(h[1])); h += 2; }
    if (!n || *h || blob_len(b, n) != n) { printf("C fail call (I <hex blob>, as format/packet.py encodes it)\n"); return; }
    call_set(b, n);
}

// TX USB port: text commands (the setup site, a terminal). Non-blocking; called once per slot while a packet is on air.
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
static void slot_report(uint32_t seq) {
    g_txseq = seq;
    host_status_publish();
}

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

// common.h "Air status", into an idle entry (FF FF FF already set).
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
    printf("TX id=%s freq_hz=%lu slot_us=%u\n", g_id, (unsigned long)g_freq_hz, SLOT_US);
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
        if (g_upd_req) fw_update();  // firmware update from the host (never returns)
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
                // Silent: a host's plan / power request is applied at once, nobody needs the announcement.
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
            // TX-side power change: reported in the status frame and announced downlink.
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
            // TX-side hop plan change: announced CMD_ACKS times ahead (ACK format, id 0xFF), then applied like a
            // receiver's command, so every receiver that hears it switches in the same slot.
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
            // Power announcement (id 0); while hopping also the plan, regularly on the centre channel where
            // unlocked receivers park.
            vcmd_t a = {.id = 0, .flags = CMD_F_PWR, .hz = g_freq_hz, .pwr = pwr_out(pwr), .eff = s16};
            if (g_hop.n) { a.flags |= CMD_F_HOP; a.hn = g_hop.n; a.hstep = g_hop.step_khz; }
            vcmd_encode(pkt, s16, &a, true);
            if (ann_left) ann_left--;
            last_plan_ann = seq;
        } else {
            pkt[0] = (uint8_t)seq; pkt[1] = (uint8_t)(seq >> 8);
            if (lq_head != lq_tail) {
                memcpy(pkt + 2, lq[lq_tail % LQ_CAP], ENTRY);
                if (pkt[2] != 0xFF) link_key_entry(pkt + 2);  // the host builds unkeyed entries: keyed here
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
        slot_report(seq);
        tx_usb_poll();
        if (t0 - last_t >= 5000000u) {
            last_t = t0;
            printf("T seq=%lu sent=%lu fail=%lu late=%lu data=%lu idle=%lu bad=%lu air_us=%lu..%lu idents=%lu\n", (unsigned long)seq,
                   (unsigned long)sent, (unsigned long)fail, (unsigned long)late, (unsigned long)data,
                   (unsigned long)idle, (unsigned long)rec_bad, (unsigned long)air_min, (unsigned long)air_max,
                   (unsigned long)g_idents);
            // Host link counters over the same 5 s, to tell which side stopped.
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
        // If we overran, skip whole slots so seq stays on schedule.
        seq++; t += SLOT_US;
        uint64_t now = time_us_64();
        while (now + 150 > t) { t += SLOT_US; seq++; late++, tot_late++; }
    }
}
static void tx_wait(void) {  // while the radio does not start: keep answering the host (identity, records, updates)
    poll_host();
    if (g_upd_req) fw_update();
}

// The host link starts first, so the host can identify this board and read the radio state even when the radio
// does not start.
void tx_main(void) {
    host_spi_init();
    host_status_publish();
    radio_start(host_status_publish, tx_wait);
#ifdef HZ_HOST_PIO
    printf("TX host=luckfox-pio spi_mode=0 sck=%u rx=%u tx=%u cs=%u\n", PIN_HOST_SCK, PIN_HOST_RX, PIN_HOST_TX,
           PIN_HOST_CS);
#endif
    run_live_tx();
}
#endif
