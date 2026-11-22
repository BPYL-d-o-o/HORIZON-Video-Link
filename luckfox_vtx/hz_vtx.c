/* Horizon video link transmitter for the Luckfox Pico (Horizon Dev Air) (RV1103 / RV1106 + SC3336 camera), over-the-air format v4.
 *
 *   camera (free running) -> ISP (scaled NV12) -> hardware HEVC encoder (one encode per frame, stock CBR)
 *   -> 231-byte video packets with CRC-16 + RS(253,237) (hz_sched.h) [+ optional block FEC parity packets, --fec]
 *   -> SPI -> TX RP2350 running
 *   vlive_tx_luckfox -> LR2021 (SF5 / 1 MHz / long-interleaver 4/5, 255-byte packets, 17.54 ms slots)
 *
 * The over-the-air format, slot timing and frequency / power control are those of the Pi transmitter
 * (pi5_vtx/live_sender.py), so the same receivers (the Ground and its receiver apps) receive either:
 *   each frame = [VPS SPS PPS] + slice NALs (4-byte start codes, no VPS/SPS/PPS/AUD/SEI) [+ stamp] + AUD
 *   VPS/SPS/PPS in-band with an IDR, at most once per --header-every s (a receiver can only start at an IDR;
 *   a running decoder keeps them), and at once whenever the encoder's parameter sets change. AUD after
 *   every frame.
 *
 * Latency: each camera frame is encoded the moment it arrives (frames that queued up in the ISP are
 * dropped in favour of the newest one); a frame is skipped before encoding while more than --max-backlog
 * bytes still wait in the sender, so the queue never grows; each radio record is built --lead-ms before the
 * RP2350 needs it. Encoding is synchronous and takes a few ms in the hardware encoder.
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

#include "hz_link.h"
#include "rk_aiq_user_api2_ae.h"
#include "rk_aiq_user_api2_imgproc.h"
#include "rk_aiq_user_api2_sysctl.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"

#ifndef HZ_GIT
#define HZ_GIT "unknown"   /* commit hz_vtx was built from (build.sh) */
#endif
#define HZ_ID_FILE "/run/hz_vtx.id"   /* identity of this transmitter (written at start, read by service.sh id) */

static const uint8_t AUD[] = {0, 0, 0, 1, 0x46, 0x01, 0x50};   /* access unit delimiter (NAL 35) */
static const uint8_t STAMP_NAL[] = {0, 0, 0, 1, 0x60, 0x01};   /* NAL 48 + 8-byte wall-clock us (--stamp) */

static volatile sig_atomic_t g_stop, g_stop_sig;
static int no_call;  /* the RP2350 has no callsign (and none given): it sends nothing, not even identification */
static void on_signal(int s) { (void)s; g_stop = 1; g_stop_sig = 1; }  /* a stop request: identify, then go silent */

/* SPI clock to the RP2354A. The Luckfox's SPI0 runs from 198 MHz with even dividers: 14.2 MHz gives 14.14 MHz
 * (198 / 14). With the RP2354A at 160 MHz its replies were measured clean up to 16.5 MHz and corrupt at 19.8; at
 * the stock 150 MHz they were already corrupt at 16.5. 14.14 MHz keeps one divider step of margin. */
#define HZ_SPI_HZ_MAX 14200000u

typedef struct {
    const char *spi_dev;
    unsigned spi_hz, spi_mode;
    unsigned width, height;
    double fps;            /* encoded frame rate cap */
    unsigned cam_fps;      /* sensor mode frame rate (0 = auto) */
    double kbps, util;
    unsigned keyint;
    double header_every;
    int low;
    double lead_ms;
    unsigned max_backlog;
    int stamp;
    int prio;
    unsigned min_qp, max_qp, min_iqp, max_iqp;
    unsigned frames;       /* stop after this many encoded frames (0 = run forever) */
    const char *dump;      /* write the video byte stream pushed into the FEC scheduler */
    const char *tap;       /* write every record handed to the RP2350 */
    int no_spi;
    int list_modes;
    int mirror, flip;      /* sensor orientation */
    const char *iq_dir;
    int tx_power;          /* half dBm to set at start (HZ_NO_PWR = leave the RP2350's value) */
    const char *ctl;       /* control FIFO ("pwr <half_dbm>" lines), "" = none */
    const char *raw_out;   /* test: write every frame handed to the encoder (NV12, no stride padding) */
    const char *raw_in;    /* test: encode the frames of this file instead of the camera's (same size), then stop */
    double raw_fps;        /* test: the --raw-in file's frame rate: its frame is chosen by elapsed time (0: one per encode) */
    unsigned refresh_stripe;   /* rows per intra-refresh stripe (multiple of 32); 0 = periodic keyframes instead */
    unsigned refresh_interval; /* frames between two refreshed stripes */
    int refresh_qp;            /* QP offset of the intra band (relative; or absolute with refresh_abs) */
    int refresh_abs;
    const char *callsign;      /* station identification for the RP2350 (NULL = keep its stored one) */
    unsigned freq_hz;          /* frequency / hop centre to set at start (0 = keep the RP2350's) */
    unsigned hop_n, hop_step_hz;   /* hop plan with freq_hz (n = 0: one fixed frequency); one packet per channel */
    unsigned fec_k, fec_m, fec_d;             /* block FEC (--fec K,M[,D]; k = 0: off) */
} opts_t;
#define HZ_NO_PWR (-1000)

/* ------------------------------------------------------------------ HEVC helpers (as sender.py) */
static int nal_type(const uint8_t *n) { return (n[0] >> 1) & 63; }

/* Splits Annex-B data into NAL units: start codes removed, trailing zero bytes of
 * every NAL except the last stripped. Returns the count; starts/lens filled up to max. */
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

/* ------------------------------------------------------------------ record tap (for the receiver simulation) */
typedef struct { FILE *f; pthread_mutex_t lock; } tap_t;
static void tap_record(void *ctx, const uint8_t *rec, double t) {
    tap_t *tp = ctx;
    pthread_mutex_lock(&tp->lock);
    fwrite(&t, sizeof t, 1, tp->f);
    fwrite(rec, HZ_REC_LEN, 1, tp->f);
    pthread_mutex_unlock(&tp->lock);
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
        "hz_vtx: Horizon video link transmitter for the Luckfox Pico (hardware HEVC)\n"
        "  --spi-dev DEV      SPI to the TX RP2350 (default /dev/spidev0.0)\n"
        "  --spi-hz HZ        SPI clock, at most 14200000 (default: that, 14.14 MHz)  --spi-mode N  (default 0)\n"
        "  --width W --height H  encoded size (default 576x320)\n"
        "  --fps F            encoded frame rate (default 30; the camera runs free, the newest frame is taken)\n"
        "  --cam-fps F        sensor mode (default: auto, the fastest = freshest frames)\n"
        "  --kbps K           video bitrate (default: --util x link capacity)  --util U (default 1.0)\n"
        "  --refresh-stripe R --refresh-interval F  keyframe-free refresh instead of keyframes (default off / 16): one\n"
        "                     slice per R-row band; every F frames one band is coded intra, moving down the picture\n"
        "                     (320x176: 6 bands x 16 frames = full picture every 3.2 s at 30 fps). Only the very\n"
        "                     first picture is a keyframe. Default --refresh-stripe 0: periodic keyframes (--keyint),\n"
        "                     which scored higher with and without loss (VMAF lab 2026-11-22) at a similar delay\n"
"  --refresh-qp Q    the intra band is coded Q coarser than the rest of the picture (default 10; \"=Q\" = absolute\n"
        "                     QP Q), which keeps a refresh band to a few packets\n"
        "  --keyint N         IDR period in encoded frames when --refresh-stripe 0 (default 60 = 2 s at 30 fps)\n"
        "  --header-every S   VPS/SPS/PPS are repeated in front of a picture once per S seconds (default 2.0; at once\n"
        "                     when they change), so a receiver that starts late can begin decoding\n"
        "  --max-backlog B    skip a camera frame while more than B bytes wait in the sender (default 940)\n"
        "  --low N --lead-ms MS  record hand-over (default 0 / 4.0, as live_sender.py)\n"
        "  --stamp            wall-clock stamp per frame (the receiver's link latency, needs NTP on both ends)\n"
        "  --min-qp/--max-qp N   P-frame QP limits (default 10 / 51)\n"
        "  --min-iqp/--max-iqp N keyframe QP limits (default 44 / 44: a fixed keyframe QP. Left to the rate control,\n"
        "                     a 10 s keyframe was 4-9 KB = 0.6-1.4 s frozen picture; at 44 it is ~1.25 KB, <= 200 ms)\n"
        "  --prio P           SCHED_FIFO priority of the SPI threads (default 20, 0 = normal)\n"
        "  --frames N         stop after N encoded frames   --no-spi  encode only (test)\n"
        "  --dump FILE        write the video byte stream   --tap FILE  write every record sent to the RP2350\n"
        "  --mirror 0|1 --flip 0|1  sensor horizontal / vertical flip (default 1 / 1 = rotated 180 degrees,\n"
        "                     as the camera is mounted on the Luckfox carrier)\n"
        "  --list-modes       print the camera sensor modes and exit\n"
        "  --tx-power N       set the TX power at start, 0.5 dBm units, -19 (-9.5 dBm) .. 44 (22 dBm)\n"
        "                     (default: keep the RP2350's value = its boot value, or what a receiver last set)\n"
        "  --raw-out FILE --raw-in FILE   test: record the frames given to the encoder / encode recorded frames\n"
        "                     instead of the camera's (same --width/--height; stops at the end of the file)\n"
        "  --raw-fps F        test: the --raw-in file is F frames per second and follows the clock as a scene would\n"
        "                     (frames skipped by the sender are skipped in the file too); --stamp then carries the\n"
        "                     file position instead of the wall clock (dev/luckfox_lab.py). Test-only encoder settings\n"
        "                     come from the environment: HZ_RC=cbr|vbr|avbr HZ_STAT=S HZ_MAXBR=PCT HZ_MINBR=PCT\n"
        "                     HZ_GOP=normalp|smartp|tsvc2|tsvc3|tsvc4 HZ_VIRIDR=N HZ_DBLK=off|BETA,TC\n"
        "                     HZ_SAO=L,C (default 0,0: SAO off) HZ_CUDQP=0..2 (default 0) HZ_SCENE=0..3 HZ_ANTIRING=0..3 HZ_ANTILINE=0..3 HZ_DEBREATH=S0,S1\n"
        "                     HZ_MDEBLUR=STRENGTH HZ_HIERQP=D0,D1,D2,D3/N0,N1,N2,N3 HZ_DELTAIP=-7..7 HZ_REENC=0..3\n"
        "                     HZ_FIRSTQP=Q HZ_MSS=1 (each one that is set is logged as \"hz: lab ...\")\n"
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
                  .lead_ms = 4.0, .max_backlog = 4 * HZ_SHARD, .refresh_stripe = 0, .refresh_interval = 16, .refresh_qp = 10,
                  .fec_k = 8, .fec_m = 4, .fec_d = 1, .prio = 20, .min_qp = 10, .max_qp = 51, .min_iqp = 44, .max_iqp = 44, .iq_dir = "/oem/usr/share/iqfiles",
                  .mirror = 1, .flip = 1, .tx_power = HZ_NO_PWR, .ctl = "/run/hz_vtx.ctl"};
    enum { O_SPIDEV = 1000, O_SPIHZ, O_SPIMODE, O_W, O_H, O_FPS, O_CAMFPS, O_KBPS, O_UTIL, O_KEYINT, O_HDR,
           O_BACKLOG, O_LOW, O_LEAD, O_STAMP, O_MINQP, O_MAXQP, O_MINIQP, O_MAXIQP, O_PRIO, O_FRAMES,
           O_DUMP, O_TAP, O_NOSPI, O_MODES, O_IQ, O_MIRROR, O_FLIP, O_HELP, O_TXPWR, O_CTL, O_RAWOUT, O_RAWIN, O_RSTRIPE, O_RINT, O_CALL, O_RQP, O_VERSION, O_FREQ, O_HOP, O_FEC, O_RAWFPS };
    static const struct option lo[] = {
        {"spi-dev", 1, 0, O_SPIDEV}, {"spi-hz", 1, 0, O_SPIHZ}, {"spi-mode", 1, 0, O_SPIMODE}, {"width", 1, 0, O_W}, {"height", 1, 0, O_H}, {"fps", 1, 0, O_FPS}, {"cam-fps", 1, 0, O_CAMFPS},
        {"kbps", 1, 0, O_KBPS}, {"util", 1, 0, O_UTIL}, {"keyint", 1, 0, O_KEYINT}, {"header-every", 1, 0, O_HDR},
        {"max-backlog", 1, 0, O_BACKLOG}, {"low", 1, 0, O_LOW}, {"lead-ms", 1, 0, O_LEAD}, {"stamp", 0, 0, O_STAMP},
        {"min-qp", 1, 0, O_MINQP}, {"max-qp", 1, 0, O_MAXQP}, {"min-iqp", 1, 0, O_MINIQP}, {"max-iqp", 1, 0, O_MAXIQP},
        {"prio", 1, 0, O_PRIO}, {"frames", 1, 0, O_FRAMES},
        {"dump", 1, 0, O_DUMP}, {"tap", 1, 0, O_TAP}, {"no-spi", 0, 0, O_NOSPI}, {"list-modes", 0, 0, O_MODES},
        {"iq-dir", 1, 0, O_IQ}, {"mirror", 1, 0, O_MIRROR}, {"flip", 1, 0, O_FLIP}, {"help", 0, 0, O_HELP},
        {"tx-power", 1, 0, O_TXPWR}, {"ctl", 1, 0, O_CTL},
        {"raw-out", 1, 0, O_RAWOUT}, {"raw-in", 1, 0, O_RAWIN}, {"raw-fps", 1, 0, O_RAWFPS}, {"refresh-stripe", 1, 0, O_RSTRIPE},
        {"refresh-interval", 1, 0, O_RINT}, {"callsign", 1, 0, O_CALL}, {"refresh-qp", 1, 0, O_RQP},
        {"version", 0, 0, O_VERSION}, {"freq", 1, 0, O_FREQ}, {"hop", 1, 0, O_HOP},
        {"fec", 1, 0, O_FEC},
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
        case O_DUMP: o->dump = optarg; break;
        case O_TAP: o->tap = optarg; break;
        case O_NOSPI: o->no_spi = 1; break;
        case O_MODES: o->list_modes = 1; break;
        case O_IQ: o->iq_dir = optarg; break;
        case O_MIRROR: o->mirror = atoi(optarg) != 0; break;
        case O_FLIP: o->flip = atoi(optarg) != 0; break;
        case O_TXPWR: o->tx_power = atoi(optarg); break;
        case O_CTL: o->ctl = optarg; break;
        case O_RAWOUT: o->raw_out = optarg; break;
        case O_RAWIN: o->raw_in = optarg; break;
        case O_RAWFPS: o->raw_fps = atof(optarg); break;
        case O_RSTRIPE: o->refresh_stripe = strtoul(optarg, NULL, 10); break;
        case O_RINT: o->refresh_interval = strtoul(optarg, NULL, 10); break;
        case O_CALL: o->callsign = optarg; break;
        case O_FREQ: o->freq_hz = strtoul(optarg, NULL, 10); break;
        case O_HOP:
            /* (a third number, the slots per channel of older configs, is ignored) */
            if (sscanf(optarg, "%u,%u", &o->hop_n, &o->hop_step_hz) != 2) o->hop_n = 9999;
            break;
        case O_FEC: {   /* "", "0" or "off" = off */
            if (!optarg[0] || !strcmp(optarg, "0") || !strcmp(optarg, "off")) { o->fec_k = 0; break; }
            int n = sscanf(optarg, "%u,%u,%u", &o->fec_k, &o->fec_m, &o->fec_d);
            if (n == 2) o->fec_d = 1;
            else if (n != 3) o->fec_k = 9999;
            break;
        }
        case O_VERSION: printf("hz_vtx git=%s format=4\n", HZ_GIT); exit(0);
        case O_RQP:
            o->refresh_abs = optarg[0] == '=';
            o->refresh_qp = atoi(optarg + o->refresh_abs);
            break;
        default: usage(); return -1;
        }
    }
    if (optind < argc) { usage(); return -1; }
    const char *bad = NULL;
    uint8_t cb[HZ_CALL_MAX];
    if (o->callsign && o->callsign[0] && !hz_call_encode(o->callsign, cb)) bad = "--callsign (1..32 characters)";
    else if (o->refresh_stripe % 32 || o->refresh_stripe > o->height || (o->refresh_stripe && (o->refresh_interval < 1 ||
             o->refresh_interval > 120))) bad = "--refresh-stripe (multiple of 32, <= height) / --refresh-interval (1..120)";
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

/* ------------------------------------------------------------------ TX power control from the Luckfox side */
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

#define CHECK(call) do { int rc_ = (call); if (rc_) { fprintf(stderr, "hz: ERROR %s = %#x\n", #call, rc_); goto done; } } while (0)

int main(int argc, char **argv) {
    opts_t o;
    if (parse(argc, argv, &o)) return 2;
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int result = 1, sys = 0, dev = 0, chn = 0, venc = 0, receiving = 0;
    int air_err = HZ_AIR_ERR_OTHER;  /* what a failed start reports to the receivers (hz_link_report) */
    rk_aiq_sys_ctx_t *aiq = NULL;
    FILE *dump = NULL;
    tap_t tap = {0};
    hz_link_t link;
    hz_sched_t sched;
    hz_feeder_t feeder = {0};
    pthread_t th_poll, th_feed, th_ctl;
    ctl_t ctl = {0};
    int have_ctl = 0;
    int threads = 0, have_link = 0;
    FILE *raw_in = NULL, *raw_out = NULL;
    buf_t body = {0}, headers = {0}, sent_hdr = {0};
    uint64_t total_frames = 0;

    /* ---- link to the RP2350 (first: its identity is known even without a camera) ---- */
    printf("hz: hz_vtx git=%s\n", HZ_GIT);
    if (hz_sched_init(&sched)) { fprintf(stderr, "hz: ERROR scheduler\n"); goto done; }
    if (o.fec_k && hz_sched_set_fec(&sched, o.fec_k, o.fec_m, o.fec_d)) { fprintf(stderr, "hz: ERROR block FEC\n"); goto done; }
    double slot_s = HZ_SLOT_S;
    int slot_known = 0;
    if (!o.no_spi && !o.list_modes) {
        if (hz_link_open(&link, o.spi_dev, o.spi_hz, (uint8_t)o.spi_mode)) {
            fprintf(stderr, "hz: ERROR cannot open %s\n", o.spi_dev);
            goto done;
        }
        have_link = 1;
        fprintf(stderr, "hz: spi %s requested_hz=%u mode=%u\n", o.spi_dev, o.spi_hz, o.spi_mode);
        feeder = (hz_feeder_t){.link = &link, .sched = &sched, .low = o.low, .lead_s = o.lead_ms / 1000.0, .prio = o.prio};
        if (o.tap) {
            tap.f = fopen(o.tap, "wb");
            if (!tap.f) { fprintf(stderr, "hz: ERROR cannot write %s\n", o.tap); goto done; }
            pthread_mutex_init(&tap.lock, NULL);
            feeder.tap = tap_record; feeder.tap_ctx = &tap;
        }
        if (pthread_create(&th_poll, NULL, hz_poll_thread, &feeder)) goto done;
        threads = 1;
        double t_end = hz_now() + 5;
        while (hz_link_pending(&link) < 0) {
            if (hz_now() > t_end || g_stop) {
                fprintf(stderr, "hz: ERROR no status from the RP2350 over SPI: is it running its vlive_tx_* firmware "
                                "(vlive_tx_luckfox / vlive_tx_video_tx), powered and wired (CS/CLK/MOSI/MISO/GND)?\n");
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
        {  /* who am I: which board and firmware this host drives (for updates / support) */
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
    }
    /* video capacity: every packet carries 231 video bytes; 1 slot in 64 is the command slot; with block FEC only
     * K of every K + M packets carry video */
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
        printf("hz: WARNING the RP2350 uses %.1f ms slots: not format v4 firmware (17.5 ms)\n", slot_s * 1000);
    printf("hz: format v4, %s, link %.1f kbit/s (slot %.2f ms%s, %s), video %u kbit/s, %ux%u <=%.1f fps, %s %u, "
           "max backlog %u B%s\n", o.no_spi ? "no SPI" : o.spi_dev, cap_bps / 1000, slot_s * 1000,
           slot_known ? "" : ", assumed", fec_txt, kbps, o.width, o.height, o.fps,
           o.refresh_stripe ? "refresh every" : "keyint", o.refresh_stripe ? o.refresh_interval : o.keyint, o.max_backlog,
           o.stamp ? ", stamped" : "");
    if (have_link) printf("hz: RP2350 reports freq_hz=%u pwr=%d (half dBm)\n", link.freq_hz, link.pwr);
    if (o.dump) {
        dump = fopen(o.dump, "wb");
        if (!dump) { fprintf(stderr, "hz: ERROR cannot write %s\n", o.dump); goto done; }
    }

    /* ---- camera sensor mode ---- */
    air_err = HZ_AIR_ERR_NO_CAMERA;
    setenv("HDR_MODE", "0", 1);
    rk_aiq_static_info_t sensor;
    memset(&sensor, 0, sizeof sensor);
    CHECK(rk_aiq_uapi2_sysctl_enumStaticMetas(0, &sensor));
    const rk_frame_fmt_t *mode = NULL;
    for (int i = 0; i < sensor.sensor_info.num && i < SUPPORT_FMT_MAX; i++) {
        const rk_frame_fmt_t *c = &sensor.sensor_info.support_fmt[i];
        if (o.list_modes)
            printf("mode %d: %dx%d @ %d fps hdr=%d\n", i, c->width, c->height, c->fps, c->hdr_mode);
        if (c->hdr_mode != 0 || c->width < (int)o.width || c->height < (int)o.height) continue;
        if (o.cam_fps && c->fps != (int)o.cam_fps) continue;
        /* auto: the fastest mode (shortest frame interval = freshest frames); ties: the widest field of view */
        if (!mode || c->fps > mode->fps || (c->fps == mode->fps && c->width * c->height > mode->width * mode->height))
            mode = c;
    }
    if (o.list_modes) { result = 0; goto done; }
    if (!mode) { fprintf(stderr, "hz: ERROR no sensor mode for %ux%u cam-fps=%u\n", o.width, o.height, o.cam_fps); goto done; }
    printf("hz: sensor %s mode %dx%d @ %d fps\n", sensor.sensor_info.sensor_name, mode->width, mode->height, mode->fps);

    /* ---- ISP + VI ---- */
    air_err = HZ_AIR_ERR_CAMERA;
    aiq = rk_aiq_uapi2_sysctl_init(sensor.sensor_info.sensor_name, o.iq_dir, NULL, NULL);
    if (!aiq) { fprintf(stderr, "hz: ERROR rk_aiq init\n"); goto done; }
    CHECK(rk_aiq_uapi2_sysctl_prepare(aiq, mode->width, mode->height, RK_AIQ_WORKING_MODE_NORMAL));
    CHECK(rk_aiq_uapi2_sysctl_start(aiq));
    /* after start: the ISP resets the sensor orientation while it starts */
    CHECK(rk_aiq_uapi2_setMirrorFlip(aiq, o.mirror != 0, o.flip != 0, 3));
    CHECK(RK_MPI_SYS_Init()); sys = 1;
    VI_DEV_ATTR_S da;
    memset(&da, 0, sizeof da);
    int rc = RK_MPI_VI_GetDevAttr(0, &da);
    if (rc == RK_ERR_VI_NOT_CONFIG) CHECK(RK_MPI_VI_SetDevAttr(0, &da));
    else if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VI_GetDevAttr = %#x\n", rc); goto done; }
    CHECK(RK_MPI_VI_EnableDev(0)); dev = 1;
    VI_DEV_BIND_PIPE_S bind;
    memset(&bind, 0, sizeof bind);
    bind.u32Num = 1; bind.PipeId[0] = 0;
    CHECK(RK_MPI_VI_SetDevBindPipe(0, &bind));
    VI_CHN_ATTR_S va;
    memset(&va, 0, sizeof va);
    va.stIspOpt.u32BufCount = 3;
    va.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    va.stSize.u32Width = o.width; va.stSize.u32Height = o.height;
    va.enPixelFormat = RK_FMT_YUV420SP;
    va.enCompressMode = COMPRESS_MODE_NONE;
    va.u32Depth = 1;
    CHECK(RK_MPI_VI_SetChnAttr(0, 0, &va));
    CHECK(RK_MPI_VI_EnableChn(0, 0)); chn = 1;

    /* First frame tells the buffer layout the encoder must accept. */
    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof frame);
    CHECK(RK_MPI_VI_GetChnFrame(0, 0, &frame, 3000));
    unsigned vir_w = frame.stVFrame.u32VirWidth, vir_h = frame.stVFrame.u32VirHeight;
    printf("hz: camera frame %ux%u (buffer %ux%u)\n", frame.stVFrame.u32Width, frame.stVFrame.u32Height, vir_w, vir_h);
    RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
    if (frame.stVFrame.u32Width != o.width || frame.stVFrame.u32Height != o.height || vir_w < o.width || vir_h < o.height) {
        fprintf(stderr, "hz: ERROR camera did not deliver %ux%u\n", o.width, o.height);
        goto done;
    }

    /* ---- hardware HEVC encoder ---- */
    VENC_CHN_ATTR_S ea;
    memset(&ea, 0, sizeof ea);
    ea.stVencAttr.enType = RK_VIDEO_ID_HEVC;
    ea.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
    ea.stVencAttr.u32PicWidth = o.width; ea.stVencAttr.u32PicHeight = o.height;
    ea.stVencAttr.u32VirWidth = vir_w; ea.stVencAttr.u32VirHeight = vir_h;
    ea.stVencAttr.u32StreamBufCnt = 2;
    ea.stVencAttr.u32BufSize = o.width * o.height * 3 / 2;
    unsigned fps_i = (unsigned)(o.fps + 0.5);
    ea.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    /* keyframe-free refresh: GOP 0 = no periodic keyframe (documented range 1..65536) */
    ea.stRcAttr.stH265Cbr.u32Gop = o.refresh_stripe ? 0 : o.keyint;
    ea.stRcAttr.stH265Cbr.u32BitRate = kbps;
    ea.stRcAttr.stH265Cbr.u32StatTime = 1;
    ea.stRcAttr.stH265Cbr.u32SrcFrameRateNum = fps_i; ea.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
    ea.stRcAttr.stH265Cbr.fr32DstFrameRateNum = fps_i; ea.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
    ea.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    /* test-only encoder settings (dev/luckfox_lab.py): rate control and GOP mode */
    const char *lab_rc = getenv("HZ_RC");
    unsigned lab_stat = getenv("HZ_STAT") ? (unsigned)atoi(getenv("HZ_STAT")) : 1;
    ea.stRcAttr.stH265Cbr.u32StatTime = lab_stat;
    if (lab_rc && strcmp(lab_rc, "cbr")) {
        int avbr = !strcmp(lab_rc, "avbr");
        if (!avbr && strcmp(lab_rc, "vbr")) { fprintf(stderr, "hz: ERROR HZ_RC=%s (cbr, vbr or avbr)\n", lab_rc); goto done; }
        VENC_H265_VBR_S v;
        memset(&v, 0, sizeof v);
        v.u32Gop = ea.stRcAttr.stH265Cbr.u32Gop;
        v.u32SrcFrameRateNum = fps_i; v.u32SrcFrameRateDen = 1; v.fr32DstFrameRateNum = fps_i; v.fr32DstFrameRateDen = 1;
        v.u32BitRate = kbps;
        v.u32MaxBitRate = kbps * (getenv("HZ_MAXBR") ? atoi(getenv("HZ_MAXBR")) : 120) / 100;
        v.u32MinBitRate = kbps * (getenv("HZ_MINBR") ? atoi(getenv("HZ_MINBR")) : 50) / 100;
        if (v.u32MaxBitRate < kbps) v.u32MaxBitRate = kbps;
        if (v.u32MinBitRate < 2) v.u32MinBitRate = 2;
        v.u32StatTime = lab_stat;
        ea.stRcAttr.enRcMode = avbr ? VENC_RC_MODE_H265AVBR : VENC_RC_MODE_H265VBR;
        if (avbr) memcpy(&ea.stRcAttr.stH265Avbr, &v, sizeof v); else memcpy(&ea.stRcAttr.stH265Vbr, &v, sizeof v);
        printf("hz: lab rate control %s %u kbit/s (max %u, min %u), stat %u s\n", lab_rc, kbps, v.u32MaxBitRate,
               v.u32MinBitRate, lab_stat);
    } else if (getenv("HZ_STAT")) printf("hz: lab CBR stat time %u s\n", lab_stat);
    if (getenv("HZ_GOP")) {
        const char *g = getenv("HZ_GOP");
        ea.stGopAttr.enGopMode = !strcmp(g, "smartp") ? VENC_GOPMODE_SMARTP : !strcmp(g, "tsvc2") ? VENC_GOPMODE_TSVC2 :
                                 !strcmp(g, "tsvc3") ? VENC_GOPMODE_TSVC3 : !strcmp(g, "tsvc4") ? VENC_GOPMODE_TSVC4 :
                                 VENC_GOPMODE_NORMALP;
        ea.stGopAttr.s32VirIdrLen = getenv("HZ_VIRIDR") ? atoi(getenv("HZ_VIRIDR")) : 0;
        printf("hz: lab GOP mode %s (virtual IDR %d)\n", g, ea.stGopAttr.s32VirIdrLen);
    }
    air_err = HZ_AIR_ERR_ENCODER;
    rc = RK_MPI_VENC_CreateChn(0, &ea);
    if (rc && o.refresh_stripe) {
        printf("hz: GOP 0 rejected (%#x): using GOP 65536 (one keyframe every %.0f min)\n", rc, 65536.0 / o.fps / 60);
        if (ea.stRcAttr.enRcMode == VENC_RC_MODE_H265VBR) ea.stRcAttr.stH265Vbr.u32Gop = 65536;
        else if (ea.stRcAttr.enRcMode == VENC_RC_MODE_H265AVBR) ea.stRcAttr.stH265Avbr.u32Gop = 65536;
        else ea.stRcAttr.stH265Cbr.u32Gop = 65536;
        rc = RK_MPI_VENC_CreateChn(0, &ea);
    }
    if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_CreateChn = %#x\n", rc); goto done; }
    venc = 1;
    VENC_RC_PARAM_S rp;
    memset(&rp, 0, sizeof rp);
    CHECK(RK_MPI_VENC_GetRcParam(0, &rp));
    rp.stParamH265.u32MinQp = o.min_qp; rp.stParamH265.u32MaxQp = o.max_qp;
    rp.stParamH265.u32MinIQp = o.min_iqp; rp.stParamH265.u32MaxIQp = o.max_iqp;
    rp.stParamH265.u32FrmMinQp = o.min_qp; rp.stParamH265.u32FrmMaxQp = o.max_qp;
    rp.stParamH265.u32FrmMinIQp = o.min_iqp; rp.stParamH265.u32FrmMaxIQp = o.max_iqp;
    /* one encode per frame */
    rp.stParamH265.s32MaxReEncodeTimes = 0;
    if (getenv("HZ_REENC")) {
        rp.stParamH265.s32MaxReEncodeTimes = atoi(getenv("HZ_REENC"));
        printf("hz: lab re-encode %d\n", rp.stParamH265.s32MaxReEncodeTimes);
    }
    if (getenv("HZ_DELTAIP")) {
        rp.stParamH265.s32DeltIpQp = atoi(getenv("HZ_DELTAIP"));
        printf("hz: lab I/P QP delta %d\n", rp.stParamH265.s32DeltIpQp);
    }
    if (getenv("HZ_FIRSTQP")) {
        rp.s32FirstFrameStartQp = (RK_U32)atoi(getenv("HZ_FIRSTQP"));
        printf("hz: lab first QP %s\n", getenv("HZ_FIRSTQP"));
    }
    printf("hz: encoder CBR, QP P %u..%u, keyframes %u..%u, no re-encoding\n", rp.stParamH265.u32MinQp,
           rp.stParamH265.u32MaxQp, rp.stParamH265.u32MinIQp, rp.stParamH265.u32MaxIQp);
    CHECK(RK_MPI_VENC_SetRcParam(0, &rp));
    /* test-only encoder settings (dev/luckfox_lab.py): coding tools; a setting the encoder rejects stops the run */
    if (getenv("HZ_DBLK")) {
        VENC_H265_DBLK_S d;
        memset(&d, 0, sizeof d);
        if (!strcmp(getenv("HZ_DBLK"), "off")) d.slice_deblocking_filter_disabled_flag = 1;
        else if (sscanf(getenv("HZ_DBLK"), "%d,%d", &d.slice_beta_offset_div2, &d.slice_tc_offset_div2) != 2) {
            fprintf(stderr, "hz: ERROR HZ_DBLK\n"); goto done;
        }
        CHECK(RK_MPI_VENC_SetH265Dblk(0, &d));
        printf("hz: lab deblocking %s\n", getenv("HZ_DBLK"));
    }
    /* SAO off and CU delta-QP 0 by default: both scored higher at this bitrate (VMAF lab 2026-11-22: SAO off
     * +2.5, CU dQP 0 +0.8 on a clean link, no loss either under packet loss). HZ_SAO / HZ_CUDQP override them. */
    {
        VENC_H265_SAO_S d;
        memset(&d, 0, sizeof d);
        if (getenv("HZ_SAO") && sscanf(getenv("HZ_SAO"), "%u,%u", &d.slice_sao_luma_flag, &d.slice_sao_chroma_flag) != 2) {
            fprintf(stderr, "hz: ERROR HZ_SAO\n"); goto done;
        }
        CHECK(RK_MPI_VENC_SetH265Sao(0, &d));
        printf("hz: %sSAO luma %u chroma %u\n", getenv("HZ_SAO") ? "lab " : "", d.slice_sao_luma_flag, d.slice_sao_chroma_flag);
    }
    {
        VENC_H265_CU_DQP_S d = {.u32CuDqp = getenv("HZ_CUDQP") ? (RK_U32)atoi(getenv("HZ_CUDQP")) : 0};
        CHECK(RK_MPI_VENC_SetH265CuDqp(0, &d));
        printf("hz: %sCU dQP %u\n", getenv("HZ_CUDQP") ? "lab " : "", d.u32CuDqp);
    }
    if (getenv("HZ_SCENE")) {
        CHECK(RK_MPI_VENC_SetSceneMode(0, (VENC_SCENE_MODE_E)atoi(getenv("HZ_SCENE"))));
        printf("hz: lab scene mode %s\n", getenv("HZ_SCENE"));
    }
    if (getenv("HZ_ANTIRING")) {
        VENC_ANTI_RING_S d = {.u32AntiRing = (RK_U32)atoi(getenv("HZ_ANTIRING"))};
        CHECK(RK_MPI_VENC_SetAntiRing(0, &d));
        printf("hz: lab anti-ring %u\n", d.u32AntiRing);
    }
    if (getenv("HZ_ANTILINE")) {
        VENC_ANTI_LINE_S d = {.u32AntiLine = (RK_U32)atoi(getenv("HZ_ANTILINE"))};
        CHECK(RK_MPI_VENC_SetAntiLine(0, &d));
        printf("hz: lab anti-line %u\n", d.u32AntiLine);
    }
    if (getenv("HZ_DEBREATH")) {
        VENC_DEBREATHEFFECT_S d;
        memset(&d, 0, sizeof d);
        d.bEnable = RK_TRUE;
        if (sscanf(getenv("HZ_DEBREATH"), "%d,%d", &d.s32Strength0, &d.s32Strength1) != 2) {
            fprintf(stderr, "hz: ERROR HZ_DEBREATH\n"); goto done;
        }
        CHECK(RK_MPI_VENC_SetDeBreathEffect(0, &d));
        printf("hz: lab debreath %d,%d\n", d.s32Strength0, d.s32Strength1);
    }
    if (getenv("HZ_MDEBLUR")) {
        CHECK(RK_MPI_VENC_EnableMotionDeblur(0, RK_TRUE));
        CHECK(RK_MPI_VENC_SetMotionDeblurStrength(0, (RK_U32)atoi(getenv("HZ_MDEBLUR"))));
        printf("hz: lab motion deblur %s\n", getenv("HZ_MDEBLUR"));
    }
    if (getenv("HZ_MSS")) {
        CHECK(RK_MPI_VENC_EnableMotionStaticSwitch(0, RK_TRUE));
        printf("hz: lab motion/static switch\n");
    }
    if (getenv("HZ_HIERQP")) {
        VENC_HIERARCHICAL_QP_S d;
        memset(&d, 0, sizeof d);
        d.bHierarchicalQpEn = RK_TRUE;
        if (sscanf(getenv("HZ_HIERQP"), "%d,%d,%d,%d/%d,%d,%d,%d", &d.s32HierarchicalQpDelta[0], &d.s32HierarchicalQpDelta[1],
                   &d.s32HierarchicalQpDelta[2], &d.s32HierarchicalQpDelta[3], &d.s32HierarchicalFrameNum[0],
                   &d.s32HierarchicalFrameNum[1], &d.s32HierarchicalFrameNum[2], &d.s32HierarchicalFrameNum[3]) != 8) {
            fprintf(stderr, "hz: ERROR HZ_HIERQP\n"); goto done;
        }
        CHECK(RK_MPI_VENC_SetHierarchicalQp(0, &d));
        printf("hz: lab hierarchical QP %s\n", getenv("HZ_HIERQP"));
    }
    {
        VENC_SUPERFRAME_CFG_S sf;
        memset(&sf, 0, sizeof sf);
        sf.enSuperFrmMode = SUPERFRM_NONE;
        sf.enRcPriority = VENC_RC_PRIORITY_BITRATE_FIRST;
        CHECK(RK_MPI_VENC_SetSuperFrameStrategy(0, &sf));
    }
    const unsigned bands = o.refresh_stripe ? (o.height + o.refresh_stripe - 1) / o.refresh_stripe : 0;
    if (o.refresh_stripe) {  /* one slice per band, so an intra band is decodable on its own */
        VENC_SLICE_SPLIT_S ss;
        memset(&ss, 0, sizeof ss);
        ss.bSplitEnable = RK_TRUE;
        ss.u32SplitMode = 1;  /* by CTU count (32x32 CTUs) */
        ss.u32SplitSize = ((o.width + 31) / 32) * (o.refresh_stripe / 32);
        CHECK(RK_MPI_VENC_SetSliceSplit(0, &ss));
        printf("hz: keyframe-free refresh: %u bands of %u rows, one band intra every %u frames (full picture every %u "
               "frames = %.1f s)\n", bands, o.refresh_stripe, o.refresh_interval, bands * o.refresh_interval,
               bands * o.refresh_interval / o.fps);
    }
    if (o.raw_in && !(raw_in = fopen(o.raw_in, "rb"))) { fprintf(stderr, "hz: ERROR cannot read %s\n", o.raw_in); goto done; }
    if (o.raw_out && !(raw_out = fopen(o.raw_out, "wb"))) { fprintf(stderr, "hz: ERROR cannot write %s\n", o.raw_out); goto done; }
    VENC_RECV_PIC_PARAM_S recv;
    memset(&recv, 0, sizeof recv);
    recv.s32RecvPicNum = -1;
    CHECK(RK_MPI_VENC_StartRecvFrame(0, &recv)); receiving = 1;

    /* ---- main loop: newest camera frame -> encode -> sender ---- */
    const uint64_t frame_us = 1000000u / (unsigned)mode->fps;
    const uint64_t min_gap_us = (uint64_t)(1e6 / o.fps) - frame_us / 2;  /* frame-rate cap, tolerant of jitter */
    uint64_t refresh_frame = 0;
    uint64_t raw_pts0 = 0;
    double raw_t = -1;   /* --raw-fps: the file position (s) of the picture being encoded */
    uint64_t last_admit_pts = 0, frames = 0, skipped = 0, capped = 0, drained = 0, enc_bytes = 0, idr_n = 0, idr_bytes = 0;
    double age_sum = 0, age_max = 0, enc_sum = 0, enc_max = 0, last_hdr = -1e9, last_rep = hz_now();
    unsigned nstat = 0;
    air_err = HZ_AIR_ERR_OTHER;  /* running: a later failure is no start-up stage */
    printf("hz: running\n");
    const int trace = getenv("HZ_TRACE") != NULL;
    while (!g_stop) {
        memset(&frame, 0, sizeof frame);
        /* Non-blocking poll every 1 ms: the blocking call (timeout) only notices a new frame on a 10 ms grid
         * (measured), so frames arrived up to ~25 ms late and missed the radio packet they could have made. */
        {
            double t_end = hz_now() + 2.0;
            while ((rc = RK_MPI_VI_GetChnFrame(0, 0, &frame, 0)) != 0 && !g_stop && hz_now() < t_end) hz_sleep(0.001);
        }
        if (rc && g_stop) break;  /* a stop request interrupted the wait: normal exit */
        if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VI_GetChnFrame = %#x\n", rc); goto done; }
        for (unsigned i = 0; i < 4; i++) {  /* take the newest frame if older ones queued up */
            VIDEO_FRAME_INFO_S newer;
            memset(&newer, 0, sizeof newer);
            if (RK_MPI_VI_GetChnFrame(0, 0, &newer, 0)) break;
            RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
            frame = newer;
            drained++;
        }
        uint64_t pts = frame.stVFrame.u64PTS;
        if (trace) fprintf(stderr, "trace: frame pts=%llu drained=%llu\n", (unsigned long long)pts, (unsigned long long)drained);
        if (getenv("HZ_TRACE_FEED")) fprintf(stderr, "cam %.4f pts %llu age %.1f ms drained %llu\n", hz_now(),
                                             (unsigned long long)pts, (mono_us() - pts) / 1000.0, (unsigned long long)drained);
        if (last_admit_pts && pts - last_admit_pts < min_gap_us) {  /* --fps below the sensor rate */
            RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
            capped++;
            continue;
        }
        if (hz_sched_backlog(&sched) > o.max_backlog) {  /* link fell behind: skip a frame rather than add delay */
            RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
            skipped++;
            continue;
        }
        last_admit_pts = pts;
        uint64_t now_us = mono_us();
        double age = now_us > pts && now_us - pts < 10000000u ? (now_us - pts) / 1000.0 : 0;
        if (raw_in || raw_out) {  /* test harness: replace / record the picture (NV12, rows without stride padding) */
            uint8_t *va = (uint8_t *)RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
            unsigned rows = o.height + o.height / 2, bad_io = 0;
            if (raw_in) {
                if (o.raw_fps > 0) {  /* the file follows the clock, as a scene in front of the camera would */
                    if (!raw_pts0) raw_pts0 = pts;
                    uint64_t idx = (uint64_t)((double)(pts - raw_pts0) * o.raw_fps / 1e6 + 0.5);
                    raw_t = idx / o.raw_fps;
                    if (fseeko(raw_in, (off_t)idx * o.width * rows, SEEK_SET)) bad_io = 1;
                }
                for (unsigned y = 0; y < rows && !bad_io; y++) {
                    uint8_t *row = y < o.height ? va + (size_t)y * vir_w : va + (size_t)vir_w * vir_h + (size_t)(y - o.height) * vir_w;
                    bad_io = fread(row, 1, o.width, raw_in) != o.width;
                }
                if (bad_io) {  /* end of the recording */
                    RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
                    printf("hz: end of %s\n", o.raw_in);
                    break;
                }
                RK_MPI_SYS_MmzFlushCache(frame.stVFrame.pMbBlk, RK_FALSE);
            } else {
                RK_MPI_SYS_MmzFlushCache(frame.stVFrame.pMbBlk, RK_TRUE);
            }
            if (raw_out)
                for (unsigned y = 0; y < rows; y++) {
                    const uint8_t *row = y < o.height ? va + (size_t)y * vir_w : va + (size_t)vir_w * vir_h + (size_t)(y - o.height) * vir_w;
                    fwrite(row, 1, o.width, raw_out);
                }
        }
        if (o.refresh_stripe) {  /* ROI 7 = forced-intra band */
            VENC_ROI_ATTR_S roi;
            memset(&roi, 0, sizeof roi);
            unsigned y = (unsigned)((refresh_frame / o.refresh_interval) % bands) * o.refresh_stripe;
            roi.u32Index = 7;
            roi.bEnable = refresh_frame % o.refresh_interval == 0 ? RK_TRUE : RK_FALSE;
            roi.bAbsQp = o.refresh_abs ? RK_TRUE : RK_FALSE;
            roi.s32Qp = o.refresh_qp;
            roi.bIntra = RK_TRUE;
            roi.stRect.s32X = 0;
            roi.stRect.s32Y = (RK_S32)y;
            roi.stRect.u32Width = o.width;
            roi.stRect.u32Height = y + o.refresh_stripe <= o.height ? o.refresh_stripe : o.height - y;
            rc = RK_MPI_VENC_SetRoiAttr(0, &roi);
            if (rc) { fprintf(stderr, "hz: ERROR refresh RK_MPI_VENC_SetRoiAttr = %#x\n", rc); goto done; }
            refresh_frame++;
        }
        double te = hz_now();
        rc = RK_MPI_VENC_SendFrame(0, &frame, 2000);
        RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
        if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_SendFrame = %#x\n", rc); goto done; }
        VENC_PACK_S pack;
        memset(&pack, 0, sizeof pack);
        VENC_STREAM_S stream;
        memset(&stream, 0, sizeof stream);
        stream.pstPack = &pack;
        stream.u32PackCount = 1;
        rc = RK_MPI_VENC_GetStream(0, &stream, 2000);
        if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_GetStream = %#x\n", rc); goto done; }
        double enc_ms = (hz_now() - te) * 1000.0;
        if (trace) {
            const uint8_t *q = (const uint8_t *)RK_MPI_MB_Handle2VirAddr(pack.pMbBlk);
            fprintf(stderr, "trace: encoded %u bytes in %.1f ms type=%d qp=%u/%u head=", pack.u32Len, enc_ms,
                    (int)pack.DataType.enH265EType, stream.stH265Info.u32StartQp, stream.stH265Info.u32MeanQp);
            for (unsigned i = 0; q && i < 12 && i < pack.u32Len; i++) fprintf(stderr, "%02x", q[i]);
            fputc('\n', stderr);
        }
        const uint8_t *au = (const uint8_t *)RK_MPI_MB_Handle2VirAddr(pack.pMbBlk);
        size_t au_len = pack.u32Len;  /* the address already points at this frame (u32Offset = its place in the ring) */

        /* frame body exactly as live_sender.py: [headers] + link_au(au) [+ stamp] + AUD */
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
        RK_MPI_VENC_ReleaseStream(0, &stream);
        if (!body.n) continue;
        double now = hz_now();
        buf_t out = {0};
        /* Parameter sets: with keyframes they go with an IDR (a receiver cannot start elsewhere); with the keyframe-free
         * refresh in front of any picture, once per --header-every s, so a receiver that starts late can begin. */
        int changed = headers.n && (headers.n != sent_hdr.n || memcmp(headers.p, sent_hdr.p, headers.n));
        int hdr_due = (idr || o.refresh_stripe) && now - last_hdr >= o.header_every;
        if (headers.n && (hdr_due || changed)) {
            buf_add(&out, headers.p, headers.n);
            last_hdr = now;
            if (changed) { sent_hdr.n = 0; buf_add(&sent_hdr, headers.p, headers.n); }
        }
        buf_add(&out, body.p, body.n);
        if (o.stamp) {
            uint64_t us = raw_t >= 0 ? (uint64_t)(raw_t * 1e6 + 0.5) : wall_us();  /* --raw-fps: the file position */
            uint8_t le[8];
            for (unsigned i = 0; i < 8; i++) le[i] = (uint8_t)(us >> (8 * i));
            buf_add(&out, STAMP_NAL, sizeof STAMP_NAL);
            buf_add(&out, le, 8);
        }
        buf_add(&out, AUD, sizeof AUD);
        hz_sched_push(&sched, out.p, out.n);
        if (getenv("HZ_TRACE_FEED")) fprintf(stderr, "push %.4f %zu B\n", hz_now(), out.n);

        if (dump) fwrite(out.p, 1, out.n, dump);
        free(out.p);
        frames++; total_frames++; nstat++;
        enc_bytes += body.n;
        if (idr) { idr_n++; idr_bytes += body.n; }
        age_sum += age; if (age > age_max) age_max = age;
        enc_sum += enc_ms; if (enc_ms > enc_max) enc_max = enc_ms;
        if (o.no_spi) {  /* encode-only test: drain the scheduler as if the link took everything */
            uint8_t rec[HZ_REC_LEN];
            while (hz_sched_next(&sched, rec)) {}
        }
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
    if (raw_in) fclose(raw_in);
    if (raw_out) fclose(raw_out);
    if (have_link && threads >= 2 && (g_stop_sig || (o.frames && total_frames >= o.frames) || (o.raw_in && result == 0))) {
        /* let the queued records go out (a stop request: at most 1 s, the frame in the queue) */
        double t_end = hz_now() + (g_stop_sig ? 1 : 10);
        while (hz_now() < t_end && hz_sched_backlog(&sched)) hz_sleep(0.02);
        hz_sleep(0.1);
    }
    if (have_link && threads >= 1 && result && !g_stop_sig) {
        /* a start that failed: the RP2350 tells the receivers why (Air status), instead of only going quiet */
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
        /* stopped on request (service.sh stop / SIGTERM): end of transmission. The RP2350 identifies once more (the
         * callsign it keeps) and then stops transmitting until video records arrive again. Not after a crash. */
        printf("hz: stop requested: final identification, then the transmitter goes silent\n");
        fflush(stdout);
        if (hz_link_ident(&link, 2, 6.0) == 0) printf("hz: final identification sent, transmitter silent\n");
        else printf("hz: WARNING final identification not confirmed by the RP2350 (old firmware?)\n");
    }
    feeder.stop = 1;
    if (threads >= 1) pthread_join(th_poll, NULL);
    if (receiving) RK_MPI_VENC_StopRecvFrame(0);
    if (venc) RK_MPI_VENC_DestroyChn(0);
    if (chn) RK_MPI_VI_DisableChn(0, 0);
    if (dev) RK_MPI_VI_DisableDev(0);
    if (sys) RK_MPI_SYS_Exit();
    if (aiq) { rk_aiq_uapi2_sysctl_stop(aiq, false); rk_aiq_uapi2_sysctl_deinit(aiq); }
    if (dump) fclose(dump);
    if (tap.f) fclose(tap.f);
    free(body.p); free(headers.p); free(sent_hdr.p);
    printf("hz: stopped (%s)\n", result ? "error" : "ok");
    return result;
}
