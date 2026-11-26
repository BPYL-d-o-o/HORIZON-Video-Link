/* hz_vtx, the Horizon video link transmitter, one program for both Air units:
 *   Horizon Dev Air      Luckfox Pico, hardware HEVC encoder (camera_luckfox.c), RP2354A running air_radio
 *   Raspberry Pi 5       Pi camera and x265 (camera_pi.cpp), the Horizon HAT's RP2350 running hat_tx (-DHZ_PI)
 * camera -> HEVC encoder -> entries (scheduler.h) -> SPI -> radio controller. Entries are built unkeyed; the radio
 * controller keys them with the link ID. Over-the-air format: format/README.md.
 * Latency: the newest camera frame is encoded at once; a frame is skipped while more than --max-backlog bytes wait,
 * so the queue never grows; each record is built --lead-ms before the RP2350 needs it.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "camera.h"
#include "options.h"
#include "radio_link.h"

#ifndef HZ_GIT
#define HZ_GIT "unknown"
#endif
#define HZ_ID_FILE "/run/hz_vtx.id"   /* read by service.sh id */

static const uint8_t AUD[] = {0, 0, 0, 1, 0x46, 0x01, 0x50};   /* access unit delimiter (NAL 35) */
static const uint8_t STAMP_NAL[] = {0, 0, 0, 1, 0x60, 0x01};   /* NAL 48 + 8-byte wall-clock us (--stamp) */

static volatile sig_atomic_t g_stop, g_stop_sig;
static int no_call;  /* the RP2350 has no callsign (and none given): it sends nothing, not even identification */
static void on_signal(int s) { (void)s; g_stop = 1; g_stop_sig = 1; }  /* a stop request: identify, then go silent */


static int nal_type(const uint8_t *n) { return (n[0] >> 1) & 63; }

/* Annex-B -> NAL units without start codes; trailing zeros stripped from all but the last. Returns the count. */
static unsigned nal_split(const uint8_t *d, size_t n, const uint8_t **start, size_t *len, unsigned max) {
    unsigned cnt = 0;
    size_t i = 0;
    while (i + 3 <= n && !(d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1)) i++;
    if (i + 3 > n) return 0;
    while (i + 3 <= n) {
        size_t s = i + 3, j = s;
        while (j + 3 <= n && !(d[j] == 0 && d[j + 1] == 0 && d[j + 2] == 1)) j++;
        int last = j + 3 > n;
        size_t end = last ? n : j;
        if (!last) while (end > s && d[end - 1] == 0) end--;
        if (end > s && cnt < max) { start[cnt] = d + s; len[cnt] = end - s; cnt++; }
        if (last) break;
        i = j;
    }
    return cnt;
}

typedef struct { uint8_t *p; size_t n, cap; } buf_t;
static void buf_add(buf_t *b, const void *d, size_t n) {
    if (b->n + n > b->cap) {
        size_t cap = (b->n + n) * 2;
        uint8_t *q = realloc(b->p, cap);
        if (!q) { fprintf(stderr, "hz: out of memory\n"); exit(1); }
        b->p = q; b->cap = cap;
    }
    memcpy(b->p + b->n, d, n);
    b->n += n;
}

static uint64_t wall_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000u + (uint64_t)tv.tv_usec;
}

static uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void usage(void) {
    fprintf(stderr,
#ifdef HZ_PI
        "hz_vtx: Horizon video link transmitter for the Raspberry Pi 5 + Horizon HAT (x265)\n"
        "  Raspberry Pi: --width/--height default 288x160, --keyint N = the intra-refresh period (default 120 = 4 s: no\n"
        "  keyframes after the first, columns coded intra in turn), --util 0.9, --header-every 1.0, SPI mode 1 at 4 MHz.\n"
        "  --preset P         x265 preset (default superfast)   --x265-extra K=V:K=V  more x265 settings\n"
        "  --source FILE      test video instead of the camera: any file ffmpeg reads (converted once to the encode size\n"
        "                     and frame rate, centre-cropped, cached in ~/.cache/horizon) or a raw I420 .yuv; looped\n"
        "  The Rockchip options below (camera mode, QP limits) do not apply.\n"
#else
        "hz_vtx: Horizon video link transmitter for the Luckfox Pico (hardware HEVC)\n"
#endif
        "  --spi-dev DEV      SPI to the TX RP2350 (default /dev/spidev0.0)\n"
        "  --spi-hz HZ        SPI clock, at most 14200000 (default: that, 14.14 MHz)  --spi-mode N  (default 0)\n"
        "  --width W --height H  encoded size (default 576x320)\n"
        "  --fps F            encoded frame rate (default 30; the camera runs free, the newest frame is taken)\n"
        "  --cam-fps F        sensor mode (default: auto, the fastest = freshest frames)\n"
        "  --kbps K           video bitrate (default: --util x link capacity)  --util U (default 1.0)\n"
        "  --keyint N         IDR period in encoded frames (default 60 = 2 s at 30 fps)\n"
        "  --header-every S   VPS/SPS/PPS are repeated in front of a picture once per S seconds (default 2.0; at once\n"
        "                     when they change), so a receiver that starts late can begin decoding\n"
        "  --max-backlog B    skip a camera frame while more than B bytes wait in the sender (default 940)\n"
        "  --low N --lead-ms MS  record hand-over (default 0 / 4.0)\n"
        "  --stamp            wall-clock stamp per frame (the receiver's link latency, needs NTP on both ends)\n"
        "  --min-qp/--max-qp N   P-frame QP limits (default 10 / 51)\n"
        "  --min-iqp/--max-iqp N keyframe QP limits (default 44 / 44: a fixed keyframe QP. Left to the rate control,\n"
        "                     a 10 s keyframe was 4-9 KB = 0.6-1.4 s frozen picture; at 44 it is ~1.25 KB, <= 200 ms)\n"
        "  --prio P           SCHED_FIFO priority of the SPI threads (default 20, 0 = normal)\n"
        "  --frames N         stop after N encoded frames\n"
        "  --mirror 0|1 --flip 0|1  sensor horizontal / vertical flip (default 1 / 1 = rotated 180 degrees,\n"
        "                     as the camera is mounted on the Air)\n"
        "  --tx-power N       set the TX power at start, 0.5 dBm units, -19 (-9.5 dBm) .. 44 (22 dBm)\n"
        "                     (default: keep the RP2350's value = its boot value, or what a receiver last set)\n"
        "  --ctl PATH         control FIFO (default /run/hz_vtx.ctl): write \"pwr N\" to change the TX power while\n"
        "                     running (service.sh power N); the current value is kept in PATH.pwr. \"\" = off\n"
        "  --callsign TEXT    station identification: stored by the RP2350 (flash) and sent every 5 minutes in a\n"
        "                     command slot (no video slot is used); default: keep the RP2350's (set from the receiver).\n"
        "                     Without a callsign the RP2350 does not transmit at all.\n"
        "                     When hz_vtx is stopped (SIGTERM / SIGINT: service.sh stop), it identifies once more and\n"
        "                     the RP2350 stops transmitting until the next start\n"
        "  --freq HZ          frequency (or hop centre) to use from the start, e.g. 868000000 (default: keep the\n"
        "                     RP2350's). Horizon Dev pairing keeps it in horizon_vtx.conf and the same in the receiver\n"
        "                     (setup site, Pair), so both always start on the same plan\n"
        "  --hop N,SPACING_HZ  with --freq: hop over N channels (2..128) SPACING_HZ apart, one packet on each\n"
        "                     (default: one fixed frequency)\n"
        "  --fec K,M[,D]|off  block FEC across packets, as the Pi sender's --fec (default 8,4): M parity packets\n"
        "                     for every block of K video packets (1..16 each); the receiver rebuilds up to M lost\n"
        "                     packets per block. D > 1 (up to 8) interleaves D blocks, so a fade of up to D*M packets\n"
        "                     in a row is rebuilt too (recovery waits ~D times longer). The video bitrate is scaled\n"
        "                     to K/(K+M) of the link. Default 8,4 (any 4 of 12 packets may be lost); off: every\n"
        "                     packet carries video\n"
        "  --version          print hz_vtx's version and exit (the RP2350's identity: service.sh id)\n");
}

static int parse(int argc, char **argv, opts_t *o) {
    *o = (opts_t){.spi_dev = "/dev/spidev0.0", .spi_hz = HZ_SPI_HZ_MAX, .spi_mode = 0, .width = 576,
                  .height = 320, .fps = 30, .util = 1.0, .keyint = 60, .header_every = 2.0, .low = 0,
                  .lead_ms = 4.0, .max_backlog = 4 * HZ_SHARD,
                  .fec_k = 8, .fec_m = 4, .fec_d = 1, .prio = 20, .min_qp = 10, .max_qp = 51, .min_iqp = 44, .max_iqp = 44, .iq_dir = "/oem/usr/share/iqfiles",
                  .mirror = 1, .flip = 1, .tx_power = HZ_NO_PWR, .ctl = "/run/hz_vtx.ctl", .preset = "superfast", .x265_extra = ""};
#ifdef HZ_PI
    /* the Pi's defaults; the service's control FIFO comes from its environment (pi_image/horizon_vtx.service) */
    o->spi_hz = 4000000; o->spi_mode = 1; o->width = 288; o->height = 160; o->keyint = 120; o->util = 0.9;
    o->header_every = 1.0;
    o->ctl = getenv("HORIZON_VTX_CTL") ? getenv("HORIZON_VTX_CTL") : "/tmp/horizon_vtx.ctl";
#endif
    enum { O_SPIDEV = 1000, O_SPIHZ, O_SPIMODE, O_W, O_H, O_FPS, O_CAMFPS, O_KBPS, O_UTIL, O_KEYINT, O_HDR,
           O_BACKLOG, O_LOW, O_LEAD, O_STAMP, O_MINQP, O_MAXQP, O_MINIQP, O_MAXIQP, O_PRIO, O_FRAMES,
           O_IQ, O_MIRROR, O_FLIP, O_HELP, O_TXPWR, O_CTL, O_CALL, O_VERSION, O_FREQ, O_HOP, O_FEC,
           O_PRESET, O_X265, O_SOURCE, O_LINK };
    static const struct option lo[] = {
        {"spi-dev", 1, 0, O_SPIDEV}, {"spi-hz", 1, 0, O_SPIHZ}, {"spi-mode", 1, 0, O_SPIMODE}, {"width", 1, 0, O_W}, {"height", 1, 0, O_H}, {"fps", 1, 0, O_FPS}, {"cam-fps", 1, 0, O_CAMFPS},
        {"kbps", 1, 0, O_KBPS}, {"util", 1, 0, O_UTIL}, {"keyint", 1, 0, O_KEYINT}, {"header-every", 1, 0, O_HDR},
        {"max-backlog", 1, 0, O_BACKLOG}, {"low", 1, 0, O_LOW}, {"lead-ms", 1, 0, O_LEAD}, {"stamp", 0, 0, O_STAMP},
        {"min-qp", 1, 0, O_MINQP}, {"max-qp", 1, 0, O_MAXQP}, {"min-iqp", 1, 0, O_MINIQP}, {"max-iqp", 1, 0, O_MAXIQP},
        {"prio", 1, 0, O_PRIO}, {"frames", 1, 0, O_FRAMES},
        {"iq-dir", 1, 0, O_IQ}, {"mirror", 1, 0, O_MIRROR}, {"flip", 1, 0, O_FLIP}, {"help", 0, 0, O_HELP},
        {"tx-power", 1, 0, O_TXPWR}, {"ctl", 1, 0, O_CTL},
        {"callsign", 1, 0, O_CALL},
        {"version", 0, 0, O_VERSION}, {"freq", 1, 0, O_FREQ}, {"hop", 1, 0, O_HOP},
        {"fec", 1, 0, O_FEC}, {"preset", 1, 0, O_PRESET}, {"x265-extra", 1, 0, O_X265}, {"source", 1, 0, O_SOURCE},
        {"link", 1, 0, O_LINK}, {"refresh", 1, 0, O_KEYINT},
        {0, 0, 0, 0}};
    int c;
    while ((c = getopt_long(argc, argv, "", lo, NULL)) != -1) {
        switch (c) {
        case O_SPIDEV: o->spi_dev = optarg; break;
        case O_SPIHZ: o->spi_hz = strtoul(optarg, NULL, 10); break;
        case O_SPIMODE: o->spi_mode = strtoul(optarg, NULL, 10); break;
        case O_W: o->width = strtoul(optarg, NULL, 10); break;
        case O_H: o->height = strtoul(optarg, NULL, 10); break;
        case O_FPS: o->fps = atof(optarg); break;
        case O_CAMFPS: o->cam_fps = strtoul(optarg, NULL, 10); break;
        case O_KBPS: o->kbps = atof(optarg); break;
        case O_UTIL: o->util = atof(optarg); break;
        case O_KEYINT: o->keyint = strtoul(optarg, NULL, 10); break;
        case O_HDR: o->header_every = atof(optarg); break;
        case O_BACKLOG: o->max_backlog = strtoul(optarg, NULL, 10); break;
        case O_LOW: o->low = atoi(optarg); break;
        case O_LEAD: o->lead_ms = atof(optarg); break;
        case O_STAMP: o->stamp = 1; break;
        case O_MINQP: o->min_qp = strtoul(optarg, NULL, 10); break;
        case O_MAXQP: o->max_qp = strtoul(optarg, NULL, 10); break;
        case O_MINIQP: o->min_iqp = strtoul(optarg, NULL, 10); break;
        case O_MAXIQP: o->max_iqp = strtoul(optarg, NULL, 10); break;
        case O_PRIO: o->prio = atoi(optarg); break;
        case O_FRAMES: o->frames = strtoul(optarg, NULL, 10); break;
        case O_IQ: o->iq_dir = optarg; break;
        case O_MIRROR: o->mirror = atoi(optarg) != 0; break;
        case O_FLIP: o->flip = atoi(optarg) != 0; break;
        case O_TXPWR: o->tx_power = atoi(optarg); break;
        case O_CTL: o->ctl = optarg; break;
        case O_CALL: o->callsign = optarg; break;
        case O_FREQ: o->freq_hz = strtoul(optarg, NULL, 10); break;
        case O_HOP:
            /* A third number in older configs is ignored. */
            if (sscanf(optarg, "%u,%u", &o->hop_n, &o->hop_step_hz) != 2) o->hop_n = 9999;
            break;
        case O_PRESET: o->preset = optarg; break;
        case O_X265: o->x265_extra = optarg; break;
        case O_SOURCE: o->source = optarg; break;
        case O_LINK:  /* the Pi sender's --link: only SPI is left */
            if (strcmp(optarg, "spi") && strcmp(optarg, "auto")) { fprintf(stderr, "hz: --link %s: only spi\n", optarg); return -1; }
            break;
        case O_FEC: {   /* "", "0" or "off" = off */
            if (!optarg[0] || !strcmp(optarg, "0") || !strcmp(optarg, "off")) { o->fec_k = 0; break; }
            int n = sscanf(optarg, "%u,%u,%u", &o->fec_k, &o->fec_m, &o->fec_d);
            if (n == 2) o->fec_d = 1;
            else if (n != 3) o->fec_k = 9999;
            break;
        }
        case O_VERSION: printf("hz_vtx git=%s format=4\n", HZ_GIT); exit(0);
        default: usage(); return -1;
        }
    }
    if (optind < argc) { usage(); return -1; }
    const char *bad = NULL;
    uint8_t cb[HZ_CALL_MAX];
    if (o->callsign && o->callsign[0] && !hz_call_encode(o->callsign, cb)) bad = "--callsign (1..32 characters)";
    else if (o->width < 64 || o->width > 1920 || o->height < 64 || o->height > 1080 || (o->width & 1) || (o->height & 1))
        bad = "--width/--height (even, 64..1920 x 64..1080)";
    else if (o->fps < 1 || o->fps > 120) bad = "--fps (1..120)";
    else if (o->keyint < 1 || o->keyint > 65536) bad = "--keyint (1..65536)";
    else if (o->kbps < 0 || o->kbps > 10000 || o->util <= 0 || o->util > 1) bad = "--kbps / --util";
    else if (o->spi_mode > 3 || o->spi_hz < 100000 || o->spi_hz > HZ_SPI_HZ_MAX) bad = "--spi-mode / --spi-hz";
    else if (o->min_qp > o->max_qp || o->min_iqp > o->max_iqp || o->max_qp > 51 || o->max_iqp > 51 || o->max_qp < 8)
        bad = "QP limits";
    else if (o->lead_ms < 0 || o->lead_ms > 15 || o->low < 0 || o->low > 20) bad = "--lead-ms / --low";
    else if (o->tx_power != HZ_NO_PWR && (o->tx_power < -19 || o->tx_power > 44)) bad = "--tx-power (-19..44, 0.5 dBm units)";
    else if (o->freq_hz && !((o->freq_hz >= 150000000u && o->freq_hz <= 960000000u) ||
                             (o->freq_hz >= 2400000000u && o->freq_hz <= 2483500000u)))
        bad = "--freq (150000000..960000000 Hz, or the 2.4 GHz band 2400000000..2483500000 Hz)";
    else if (o->hop_n && (!o->freq_hz || o->hop_n < 2 || o->hop_n > 128 || !o->hop_step_hz || o->hop_step_hz % 1000 ||
             o->hop_step_hz / 1000 > 65535))
        bad = "--hop (N 2..128, SPACING_HZ in kHz steps; needs --freq)";
    else if (o->fec_k && (o->fec_k > 16 || o->fec_m < 1 || o->fec_m > 16 || o->fec_d < 1 || o->fec_d > 8))
        bad = "--fec (K,M[,D] with 1 <= K, M <= 16, 1 <= D <= 8)";
    if (bad) { fprintf(stderr, "hz: invalid %s\n", bad); return -1; }
    return 0;
}

/* TX power control from the Luckfox side. */
typedef struct { hz_link_t *link; const char *path; volatile int stop; } ctl_t;

static void ctl_write_state(const char *path, int pwr) {
    char tmp[160], fin[160];
    snprintf(fin, sizeof fin, "%s.pwr", path);
    snprintf(tmp, sizeof tmp, "%s.pwr.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "%d\n", pwr);
    fclose(f);
    rename(tmp, fin);
}

/* Reads "pwr <half_dbm>" lines from the control FIFO and keeps PATH.pwr = the RP2350's current power. */
static void *ctl_thread(void *arg) {
    ctl_t *c = arg;
    unlink(c->path);
    if (mkfifo(c->path, 0660)) { fprintf(stderr, "hz: WARNING cannot create %s: %s\n", c->path, strerror(errno)); return NULL; }
    int fd = open(c->path, O_RDWR | O_NONBLOCK);  /* RDWR: never sees EOF when a writer closes */
    if (fd < 0) { fprintf(stderr, "hz: WARNING cannot open %s\n", c->path); return NULL; }
    char buf[128];
    size_t n = 0;
    int shown = HZ_NO_PWR;
    while (!c->stop && !g_stop) {
        pthread_mutex_lock(&c->link->lock);
        int pwr = c->link->have_status ? c->link->pwr : HZ_NO_PWR;
        pthread_mutex_unlock(&c->link->lock);
        if (pwr != shown && pwr != HZ_NO_PWR) { ctl_write_state(c->path, pwr); shown = pwr; }
        struct pollfd p = {.fd = fd, .events = POLLIN};
        if (poll(&p, 1, 200) <= 0) continue;
        ssize_t r = read(fd, buf + n, sizeof buf - 1 - n);
        if (r <= 0) continue;
        n += (size_t)r;
        char *nl;
        while ((nl = memchr(buf, '\n', n))) {
            *nl = 0;
            int v;
            if (sscanf(buf, "pwr %d", &v) == 1) {
                if (v < -19 || v > 44) printf("hz: control: TX power %d out of range (-19..44, 0.5 dBm units)\n", v);
                else if (hz_link_set_power(c->link, v) == 0) printf("hz: control: TX power set to %d (%.1f dBm)\n", v, v / 2.0);
                else printf("hz: control: TX power %d NOT confirmed by the RP2350 (firmware without TX-side power?)\n", v);
            } else if (buf[0]) {
                printf("hz: control: unknown command '%s' (use: pwr <half_dbm>)\n", buf);
            }
            fflush(stdout);
            size_t used = (size_t)(nl - buf) + 1;
            memmove(buf, nl + 1, n - used);
            n -= used;
        }
        if (n >= sizeof buf - 1) n = 0;
    }
    close(fd);
    unlink(c->path);
    return NULL;
}

int main(int argc, char **argv) {
    opts_t o;
    if (parse(argc, argv, &o)) return 2;
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int result = 1, rc;
    int air_err = HZ_AIR_ERR_OTHER;  /* what a failed start reports to the receivers (hz_link_report) */
    camera_t cam = {0};
    hz_link_t link;
    hz_sched_t sched;
    hz_feeder_t feeder = {0};
    pthread_t th_poll, th_feed, th_ctl;
    ctl_t ctl = {0};
    int have_ctl = 0;
    int threads = 0, have_link = 0;
    buf_t body = {0}, headers = {0}, sent_hdr = {0};
    uint64_t total_frames = 0;

    /* The RP2350 link first: its identity is known even without a camera. */
    printf("hz: hz_vtx git=%s\n", HZ_GIT);
    if (hz_sched_init(&sched)) { fprintf(stderr, "hz: ERROR scheduler\n"); goto done; }
    if (o.fec_k && hz_sched_set_fec(&sched, o.fec_k, o.fec_m, o.fec_d)) { fprintf(stderr, "hz: ERROR block FEC\n"); goto done; }
    double slot_s = HZ_SLOT_S;
    int slot_known = 0;
    if (hz_link_open(&link, o.spi_dev, o.spi_hz, (uint8_t)o.spi_mode)) {
        fprintf(stderr, "hz: ERROR cannot open %s\n", o.spi_dev);
        goto done;
    }
    have_link = 1;
    fprintf(stderr, "hz: spi %s requested_hz=%u mode=%u\n", o.spi_dev, o.spi_hz, o.spi_mode);
    feeder = (hz_feeder_t){.link = &link, .sched = &sched, .low = o.low, .lead_s = o.lead_ms / 1000.0, .prio = o.prio};
    if (pthread_create(&th_poll, NULL, hz_poll_thread, &feeder)) goto done;
    threads = 1;
    double t_end = hz_now() + 5;
    while (hz_link_pending(&link) < 0) {
        if (hz_now() > t_end || g_stop) {
            fprintf(stderr, "hz: ERROR no status from the RP2350 over SPI: is it running its transmitter firmware (air_radio / hat_tx) "
                            "(air_radio), powered and wired (CS/CLK/MOSI/MISO/GND)?\n");
            goto done;
        }
        hz_sleep(0.05);
    }
    t_end = hz_now() + 3;
    while (hz_now() < t_end && !g_stop) {  /* learn the slot length the firmware really uses */
        double st, a;
        hz_link_snapshot(&link, &st, &slot_s, &a, &slot_known);
        if (slot_known) break;
        hz_sleep(0.05);
    }
    {
        hz_who_t w;
        FILE *idf = fopen(HZ_ID_FILE ".tmp", "w");
        if (idf) fprintf(idf, "hz_vtx_git=%s\n", HZ_GIT);
        if (hz_link_whoami(&link, &w, 1.0) == 0) {
            char uid[17];
            for (unsigned i = 0; i < 8; i++) snprintf(uid + 2 * i, 3, "%02X", w.uid[i]);
            printf("hz: RP2350 board=%s (%u) fw=%s git=%08x%s format=%u uid=%s\n", hz_board_name(w.board), w.board,
                   w.target, (unsigned)w.commit, w.dirty ? "-dirty" : "", w.format, uid);
            /* hz_vtx builds format 4 entries; a format 5 RP2350 keys them with its link ID (firmware "Link ID") */
            if (w.format < 4 || w.format > 5)
                printf("hz: WARNING the RP2350 firmware uses over-the-air format %u, hz_vtx needs 4 or 5\n", w.format);
            if (idf) fprintf(idf, "rp2350_board=%s\nrp2350_board_id=%u\nrp2350_fw=%s\nrp2350_git=%08x%s\nrp2350_format=%u\n"
                             "rp2350_uid=%s\nrp2350_radio=%s\nrp2350_radio_rc=%d\nrp2350_callsign=%s\n",
                             hz_board_name(w.board), w.board, w.target, (unsigned)w.commit, w.dirty ? "-dirty" : "",
                             w.format, uid, hz_radio_name(w.radio), w.radio_rc,
                             (o.callsign && o.callsign[0]) || w.has_call == 1 ? "set" : w.has_call == 2 ? "default" : w.has_call == 0 ? "none" : "unknown");
            no_call = w.has_call == 0 && !(o.callsign && o.callsign[0]);
            if (no_call)
                printf("hz: WARNING no callsign set: the transmitter stays silent until one is set (--callsign, setup site Pair)\n");
            if (w.has_call == 2 && !(o.callsign && o.callsign[0]))
                printf("hz: WARNING identifying as the N0CALL default (debugging only): set your callsign (--callsign, setup site Pair)\n");
            if (w.radio == 2)
                printf("hz: ERROR the RP2350's radio does not start (radio_init error %d): LR2021 module, its power "
                       "or its SPI wiring\n", -w.radio_rc);
        } else {
            printf("hz: RP2350 identity unknown (firmware older than 2026-11-20 does not report one)\n");
            if (idf) fprintf(idf, "rp2350_board=unknown\n");
        }
        if (idf) { fclose(idf); rename(HZ_ID_FILE ".tmp", HZ_ID_FILE); }
    }
    if (o.callsign && o.callsign[0]) {  /* the RP2350 keeps it (flash) and only rewrites it when it changed */
        uint8_t cb[HZ_CALL_MAX];
        unsigned cn = hz_call_encode(o.callsign, cb);
        if (hz_link_set_call(&link, cb, cn) == 0) printf("hz: callsign \"%s\" (%u bytes on air) given to the RP2350\n", o.callsign, cn);
        else printf("hz: WARNING callsign not handed to the RP2350\n");
    }
    if (o.freq_hz) {  /* the paired plan (horizon_vtx.conf): the RP2350 announces it and switches with the receivers */
        if (hz_link_set_plan(&link, o.freq_hz, o.hop_n, o.hop_step_hz / 1000, 5.0) == 0)
            printf("hz: plan freq_hz=%u hop=%u,%u in use\n", o.freq_hz, o.hop_n, o.hop_step_hz);
        else printf("hz: WARNING plan freq_hz=%u not confirmed by the RP2350 (firmware without plan records?)\n", o.freq_hz);
    }
    if (o.tx_power != HZ_NO_PWR) {
        if (hz_link_set_power(&link, o.tx_power) == 0) printf("hz: TX power set to %d (%.1f dBm)\n", o.tx_power, o.tx_power / 2.0);
        else printf("hz: WARNING TX power %d not confirmed by the RP2350 (firmware without TX-side power?)\n", o.tx_power);
    }
    if (pthread_create(&th_feed, NULL, hz_feeder_thread, &feeder)) goto done;
    threads = 2;
    if (o.ctl && o.ctl[0]) {
        ctl = (ctl_t){.link = &link, .path = o.ctl};
        if (pthread_create(&th_ctl, NULL, ctl_thread, &ctl) == 0) have_ctl = 1;
    }
    /* 1 slot in 64 is the command slot; with block FEC only K of every K + M packets carry video */
    double cap_bps = HZ_SHARD * 8.0 / slot_s * 63.0 / 64.0;
    char fec_txt[48] = "no cross-packet FEC";
    if (o.fec_k) {
        cap_bps = cap_bps * o.fec_k / (o.fec_k + o.fec_m);
        if (o.fec_d > 1) snprintf(fec_txt, sizeof fec_txt, "block FEC %u+%u, interleaved x%u", o.fec_k, o.fec_m, o.fec_d);
        else snprintf(fec_txt, sizeof fec_txt, "block FEC %u+%u", o.fec_k, o.fec_m);
        printf("hz: %s\n", fec_txt);
    }
    unsigned kbps = o.kbps > 0 ? (unsigned)o.kbps : (unsigned)(o.util * cap_bps / 1000);
    if (kbps < 2) kbps = 2;
    if (have_link && slot_s < 0.015)
        printf("hz: WARNING the RP2350 uses %.1f ms slots: not the live transmitter firmware (17.54 ms)\n", slot_s * 1000);
    printf("hz: %s, link %.1f kbit/s (slot %.2f ms%s, %s), video %u kbit/s, %ux%u <=%.1f fps, %s %u, "
           "max backlog %u B%s\n", o.spi_dev, cap_bps / 1000, slot_s * 1000,
           slot_known ? "" : ", assumed", fec_txt, kbps, o.width, o.height, o.fps,
           "keyint", o.keyint, o.max_backlog,
           o.stamp ? ", stamped" : "");
    if (have_link) printf("hz: RP2350 reports freq_hz=%u pwr=%d (half dBm)\n", link.freq_hz, link.pwr);

    int cam_fail;
    if (camera_open(&cam, &o, kbps, &cam_fail)) {
        air_err = cam_fail == CAM_FAIL_NO_CAMERA ? HZ_AIR_ERR_NO_CAMERA : cam_fail == CAM_FAIL_CAMERA ? HZ_AIR_ERR_CAMERA
                  : HZ_AIR_ERR_ENCODER;
        goto done;
    }

    const uint64_t frame_us = 1000000u / cam.sensor_fps;
    const uint64_t min_gap_us = (uint64_t)(1e6 / o.fps) - frame_us / 2;  /* frame-rate cap, tolerant of jitter */
    uint64_t last_admit_pts = 0, frames = 0, skipped = 0, capped = 0, drained = 0, enc_bytes = 0, idr_n = 0, idr_bytes = 0;
    double age_sum = 0, age_max = 0, enc_sum = 0, enc_max = 0, last_hdr = -1e9, last_rep = hz_now();
    unsigned nstat = 0;
    air_err = HZ_AIR_ERR_OTHER;  /* running: a later failure is no start-up stage */
    printf("hz: running\n");
    while (!g_stop) {
        uint64_t pts;
        rc = camera_grab(&cam, &g_stop, &pts, &drained);
        if (rc == 1) break;  /* a stop request interrupted the wait: normal exit */
        if (rc) goto done;
        if (last_admit_pts && pts - last_admit_pts < min_gap_us) {  /* --fps below the sensor rate */
            camera_drop(&cam);
            capped++;
            continue;
        }
        if (hz_sched_backlog(&sched) > o.max_backlog) {  /* link fell behind: skip a frame rather than add delay */
            camera_drop(&cam);
            skipped++;
            continue;
        }
        last_admit_pts = pts;
        uint64_t now_us = mono_us();
        double age = now_us > pts && now_us - pts < 10000000u ? (now_us - pts) / 1000.0 : 0;
        const uint8_t *au;
        size_t au_len;
        double enc_ms;
        if (camera_encode(&cam, &au, &au_len, &enc_ms)) goto done;

        /* frame body: [headers] + the access unit without parameter sets / AUD / SEI [+ stamp] + AUD */
        const uint8_t *nal[64];
        size_t nlen[64];
        unsigned nn = au ? nal_split(au, au_len, nal, nlen, 64) : 0;
        int idr = 0, new_hdr = 0;
        body.n = 0;
        for (unsigned i = 0; i < nn; i++) {
            int t = nal_type(nal[i]);
            if (t >= 32 && t <= 34) {
                if (!new_hdr) { headers.n = 0; new_hdr = 1; }
                buf_add(&headers, "\0\0\0\1", 4);
                buf_add(&headers, nal[i], nlen[i]);
                continue;
            }
            if (t == 35 || t == 39 || t == 40) continue;  /* AUD / SEI */
            if (t >= 16 && t <= 21) idr = 1;               /* IRAP picture */
            buf_add(&body, "\0\0\0\1", 4);
            buf_add(&body, nal[i], nlen[i]);
        }
        camera_done(&cam);
        if (!body.n) continue;
        double now = hz_now();
        buf_t out = {0};
        /* Parameter sets: with an IDR (a receiver cannot start elsewhere), or with the keyframe-free refresh in front of
         * any picture, at most once per --header-every s; and at once when they change. */
        int changed = headers.n && (headers.n != sent_hdr.n || memcmp(headers.p, sent_hdr.p, headers.n));
        int hdr_due = (idr || cam.refresh) && now - last_hdr >= o.header_every;
        if (headers.n && (hdr_due || changed)) {
            buf_add(&out, headers.p, headers.n);
            last_hdr = now;
            if (changed) { sent_hdr.n = 0; buf_add(&sent_hdr, headers.p, headers.n); }
        }
        buf_add(&out, body.p, body.n);
        if (o.stamp) {
            uint64_t us = wall_us();
            uint8_t le[8];
            for (unsigned i = 0; i < 8; i++) le[i] = (uint8_t)(us >> (8 * i));
            buf_add(&out, STAMP_NAL, sizeof STAMP_NAL);
            buf_add(&out, le, 8);
        }
        buf_add(&out, AUD, sizeof AUD);
        hz_sched_push(&sched, out.p, out.n);

        free(out.p);
        frames++; total_frames++; nstat++;
        enc_bytes += body.n;
        if (idr) { idr_n++; idr_bytes += body.n; }
        age_sum += age; if (age > age_max) age_max = age;
        enc_sum += enc_ms; if (enc_ms > enc_max) enc_max = enc_ms;
        if (o.frames && total_frames >= o.frames) break;
        if (now - last_rep >= 5.0) {
            double dt = now - last_rep;
            pthread_mutex_lock(&sched.lock);
            uint64_t sb = sched.bytes, sp = sched.pad, ss = sched.shards, spar = sched.parity;
            double qd = sched.qdelay_n ? sched.qdelay_sum / sched.qdelay_n : 0, qm = sched.qdelay_max;
            sched.bytes = sched.pad = sched.shards = sched.parity = 0; sched.qdelay_sum = sched.qdelay_max = 0; sched.qdelay_n = 0;
            pthread_mutex_unlock(&sched.lock);
            char tline[220] = "", ev[4][120], par_txt[40] = "";
            if (o.fec_k) snprintf(par_txt, sizeof par_txt, " + parity %llu", (unsigned long long)spar);
            unsigned nev = 0;
            double ls = slot_s, la = 0.0174;
            if (have_link) {
                pthread_mutex_lock(&link.lock);
                snprintf(tline, sizeof tline, "%s", link.last_t);
                nev = link.nevents;
                memcpy(ev, link.events, sizeof ev);
                link.nevents = 0;
                ls = link.slot_s; la = link.air_s;
                pthread_mutex_unlock(&link.lock);
            }
            printf("tx: %.1f fps, %.1f kbit/s video (encoder %.1f), %llu B/frame avg, IDR %llu (%llu B), shards %llu%s, pad %.0f%%, "
                   "backlog %zu B, skipped %llu, capped %llu, stale %llu\n  RP2350: %s\n",
                   nstat / dt, sb * 8 / dt / 1000, enc_bytes * 8 / dt / 1000, (unsigned long long)(nstat ? enc_bytes / nstat : 0),
                   (unsigned long long)idr_n, (unsigned long long)(idr_n ? idr_bytes / idr_n : 0), (unsigned long long)ss, par_txt,
                   100.0 * sp / (sb + sp ? sb + sp : 1), hz_sched_backlog(&sched), (unsigned long long)skipped,
                   (unsigned long long)capped, (unsigned long long)drained, tline);
            printf("  latency (ms, mean/max): camera frame age %.0f/%.0f, encode %.1f/%.1f, wait in sender queue %.0f/%.0f; "
                   "radio adds ~%.0f slot wait + %.1f air\n", nstat ? age_sum / nstat : 0, age_max, nstat ? enc_sum / nstat : 0,
                   enc_max, qd, qm, ls * 500, la * 1000);
            if (have_link) printf("  RP2350: freq_hz=%u pwr=%d (half dBm), spi_errors=%llu\n", link.freq_hz, link.pwr,
                                  (unsigned long long)link.spi_errors);
            for (unsigned i = 0; i < nev; i++) printf("  RP2350: %s\n", ev[i]);
            nstat = 0; enc_bytes = 0; idr_n = 0; idr_bytes = 0; skipped = capped = drained = 0; (void)frames;
            age_sum = age_max = enc_sum = enc_max = 0;
            last_rep = now;
        }
    }
    result = 0;
done:
    if (have_link && threads >= 2 && (g_stop_sig || (o.frames && total_frames >= o.frames))) {
        /* let the queued records go out (a stop request: at most 1 s, the frame in the queue) */
        double t_end = hz_now() + (g_stop_sig ? 1 : 10);
        while (hz_now() < t_end && hz_sched_backlog(&sched)) hz_sleep(0.02);
        hz_sleep(0.1);
    }
    if (have_link && threads >= 1 && result && !g_stop_sig) {
        /* a failed start: the RP2350 tells the receivers why (Air status) */
        hz_link_report(&link, air_err);
        printf("hz: reported to the receivers: %s\n", air_err == HZ_AIR_ERR_NO_CAMERA ? "no camera"
               : air_err == HZ_AIR_ERR_CAMERA ? "camera does not start" : air_err == HZ_AIR_ERR_ENCODER ? "encoder does not start"
               : "start-up error");
    }
    feeder.stop = 1;
    if (have_ctl) { ctl.stop = 1; pthread_join(th_ctl, NULL); }
    if (threads >= 2) pthread_join(th_feed, NULL);
    if (have_link && threads >= 1 && g_stop_sig && no_call) {
        printf("hz: stop requested: nothing to identify (no callsign set, the transmitter was silent)\n");
    } else if (have_link && threads >= 1 && g_stop_sig) {
        /* stopped on request: end of transmission. The RP2350 identifies once more, then stays silent until video
         * records arrive again. */
        printf("hz: stop requested: final identification, then the transmitter goes silent\n");
        fflush(stdout);
        if (hz_link_ident(&link, 2, 6.0) == 0) printf("hz: final identification sent, transmitter silent\n");
        else printf("hz: WARNING final identification not confirmed by the RP2350 (old firmware?)\n");
    }
    feeder.stop = 1;
    if (threads >= 1) pthread_join(th_poll, NULL);
    camera_close(&cam);
    free(body.p); free(headers.p); free(sent_hdr.p);
    printf("hz: stopped (%s)\n", result ? "error" : "ok");
    return result;
}
