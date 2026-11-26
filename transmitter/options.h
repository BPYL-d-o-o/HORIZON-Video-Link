/* hz_vtx's options (main.c parses them; camera_luckfox.c / camera_pi.cpp read the camera and encoder ones). */
#pragma once

#ifdef HZ_PI
/* SPI clock to the HAT's RP2350 (SPI1 slave, mode 1): the RP2350 slave allows up to 12.5 MHz. */
#define HZ_SPI_HZ_MAX 12500000u
#else
/* The Luckfox's SPI0 divides 198 MHz evenly: 14.2 gives 14.14 MHz. The RP2354A at 160 MHz is clean to 16.5 MHz and
 * corrupt at 19.8, so this keeps one divider step of margin. */
#define HZ_SPI_HZ_MAX 14200000u
#endif
#define HZ_NO_PWR (-1000)

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
    int mirror, flip;      /* sensor orientation */
    const char *iq_dir;
    int tx_power;          /* half dBm to set at start (HZ_NO_PWR = leave the RP2350's value) */
    const char *ctl;       /* control FIFO ("pwr <half_dbm>" lines), "" = none */
    const char *callsign;      /* station identification for the RP2350 (NULL = keep its stored one) */
    unsigned freq_hz;          /* frequency / hop centre to set at start (0 = keep the RP2350's) */
    unsigned hop_n, hop_step_hz;   /* hop plan with freq_hz (n = 0: one fixed frequency); one packet per channel */
    unsigned fec_k, fec_m, fec_d;             /* block FEC (--fec K,M[,D]; k = 0: off) */
    const char *preset;        /* Pi: the x265 preset */
    const char *x265_extra;    /* Pi: more x265 settings, name=value:name=value */
    const char *source;        /* Pi: a test video instead of the camera (looped) */
} opts_t;
