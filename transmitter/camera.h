/* The camera and the H.265 encoder: everything hz_vtx asks of them, so main.c needs neither's API. One
 * implementation per board, the same interface:
 *   camera_luckfox.c  the Luckfox Pico (Horizon Dev Air): Rockchip rkaiq ISP + Rockit MPI, hardware HEVC encoder
 *   camera_pi.cpp     the Raspberry Pi 5: libcamera (C++), x265
 *
 *   camera_open()    sensor mode, ISP, the scaled picture, the encoder; starts the camera
 *   camera_grab()    the newest camera frame (older ones are dropped)
 *   camera_drop()    gives a grabbed frame back without encoding it
 *   camera_encode()  encodes the grabbed frame (one encode per frame); camera_done() releases its access unit
 *   camera_close()   stops and frees everything, whatever camera_open() got to */
#pragma once
#include <signal.h>
#include <stddef.h>
#include <stdint.h>

#include "options.h"

#ifdef __cplusplus
extern "C" {
#endif

/* how far camera_open() got when it failed (what the receivers are told: radio_link.h HZ_AIR_ERR_*) */
enum { CAM_FAIL_NO_CAMERA, CAM_FAIL_CAMERA, CAM_FAIL_ENCODER };

typedef struct {
    unsigned sensor_fps;  /* the sensor mode's frame rate */
    int refresh;          /* no periodic keyframes (gradual refresh): parameter sets go in front of any picture */
    void *impl;           /* the board's own state */
} camera_t;

/* 0 = encoding; -1 = failed (*fail: how far it got) */
int camera_open(camera_t *c, const opts_t *o, unsigned kbps, int *fail);
/* 0 = a frame (*pts: its capture time, us, CLOCK_MONOTONIC; *drained counts the older frames dropped);
 * 1 = *stop was set; -1 = error */
int camera_grab(camera_t *c, volatile sig_atomic_t *stop, uint64_t *pts, uint64_t *drained);
void camera_drop(camera_t *c);
/* 0 = *au / *len hold the access unit (release it with camera_done); -1 = error. *enc_ms: time in the encoder. */
int camera_encode(camera_t *c, const uint8_t **au, size_t *len, double *enc_ms);
void camera_done(camera_t *c);
void camera_close(camera_t *c);

#ifdef __cplusplus
}
#endif
