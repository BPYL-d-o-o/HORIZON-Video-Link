/* The Luckfox Pico's camera and hardware HEVC encoder (camera.h): Rockchip rkaiq ISP + Rockit MPI. */
#define _GNU_SOURCE
#include "camera.h"

#include <stdio.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include "rk_aiq_user_api2_ae.h"
#include "rk_aiq_user_api2_imgproc.h"
#include "rk_aiq_user_api2_sysctl.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"

typedef struct {
    rk_aiq_sys_ctx_t *aiq;
    int sys, dev, chn, venc, receiving;
    unsigned vir_w, vir_h;          /* the camera buffer's stride and height */
    VIDEO_FRAME_INFO_S frame;       /* the grabbed frame */
    VENC_STREAM_S stream;           /* and its encoded access unit */
    VENC_PACK_S pack;
} lf_t;

#define CHECK(call) do { int rc_ = (call); if (rc_) { fprintf(stderr, "hz: ERROR %s = %#x\n", #call, rc_); return -1; } } while (0)

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void sleep_s(double s) {
    struct timespec ts = {(time_t)s, (long)((s - (time_t)s) * 1e9)};
    while (nanosleep(&ts, &ts) && errno == EINTR) {}
}

/* SAO off and CU delta-QP 0 score higher at this bitrate. */
static int set_coding_tools(void) {
    VENC_H265_SAO_S sao;
    memset(&sao, 0, sizeof sao);
    VENC_H265_CU_DQP_S dqp = {.u32CuDqp = 0};
    CHECK(RK_MPI_VENC_SetH265Sao(0, &sao));
    CHECK(RK_MPI_VENC_SetH265CuDqp(0, &dqp));
    printf("hz: SAO luma %u chroma %u, CU dQP %u\n", sao.slice_sao_luma_flag, sao.slice_sao_chroma_flag, dqp.u32CuDqp);
    {
        VENC_SUPERFRAME_CFG_S sf;
        memset(&sf, 0, sizeof sf);
        sf.enSuperFrmMode = SUPERFRM_NONE;
        sf.enRcPriority = VENC_RC_PRIORITY_BITRATE_FIRST;
        CHECK(RK_MPI_VENC_SetSuperFrameStrategy(0, &sf));
    }
    return 0;
}

static int open_encoder(lf_t *c, const opts_t *o, unsigned kbps) {
    VENC_CHN_ATTR_S ea;
    memset(&ea, 0, sizeof ea);
    ea.stVencAttr.enType = RK_VIDEO_ID_HEVC;
    ea.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
    ea.stVencAttr.u32PicWidth = o->width; ea.stVencAttr.u32PicHeight = o->height;
    ea.stVencAttr.u32VirWidth = c->vir_w; ea.stVencAttr.u32VirHeight = c->vir_h;
    ea.stVencAttr.u32StreamBufCnt = 2;
    ea.stVencAttr.u32BufSize = o->width * o->height * 3 / 2;
    unsigned fps_i = (unsigned)(o->fps + 0.5);
    ea.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    /* keyframe-free refresh: GOP 0 = no periodic keyframe (documented range 1..65536) */
    ea.stRcAttr.stH265Cbr.u32Gop = o->keyint;
    ea.stRcAttr.stH265Cbr.u32BitRate = kbps;
    ea.stRcAttr.stH265Cbr.u32StatTime = 1;
    ea.stRcAttr.stH265Cbr.u32SrcFrameRateNum = fps_i; ea.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
    ea.stRcAttr.stH265Cbr.fr32DstFrameRateNum = fps_i; ea.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
    ea.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    int rc = RK_MPI_VENC_CreateChn(0, &ea);
    if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_CreateChn = %#x\n", rc); return -1; }
    c->venc = 1;
    VENC_RC_PARAM_S rp;
    memset(&rp, 0, sizeof rp);
    CHECK(RK_MPI_VENC_GetRcParam(0, &rp));
    rp.stParamH265.u32MinQp = o->min_qp; rp.stParamH265.u32MaxQp = o->max_qp;
    rp.stParamH265.u32MinIQp = o->min_iqp; rp.stParamH265.u32MaxIQp = o->max_iqp;
    rp.stParamH265.u32FrmMinQp = o->min_qp; rp.stParamH265.u32FrmMaxQp = o->max_qp;
    rp.stParamH265.u32FrmMinIQp = o->min_iqp; rp.stParamH265.u32FrmMaxIQp = o->max_iqp;
    rp.stParamH265.s32MaxReEncodeTimes = 0;
    printf("hz: encoder CBR, QP P %u..%u, keyframes %u..%u, no re-encoding\n", rp.stParamH265.u32MinQp,
           rp.stParamH265.u32MaxQp, rp.stParamH265.u32MinIQp, rp.stParamH265.u32MaxIQp);
    CHECK(RK_MPI_VENC_SetRcParam(0, &rp));
    return set_coding_tools();
}

int camera_open(camera_t *cam, const opts_t *o, unsigned kbps, int *fail) {
    memset(cam, 0, sizeof *cam);
    lf_t *c = cam->impl = calloc(1, sizeof(lf_t));
    if (!c) { fprintf(stderr, "hz: out of memory\n"); return -1; }

    *fail = CAM_FAIL_NO_CAMERA;
    setenv("HDR_MODE", "0", 1);
    rk_aiq_static_info_t sensor;
    memset(&sensor, 0, sizeof sensor);
    CHECK(rk_aiq_uapi2_sysctl_enumStaticMetas(0, &sensor));
    const rk_frame_fmt_t *mode = NULL;
    for (int i = 0; i < sensor.sensor_info.num && i < SUPPORT_FMT_MAX; i++) {
        const rk_frame_fmt_t *m = &sensor.sensor_info.support_fmt[i];
        if (m->hdr_mode != 0 || m->width < (int)o->width || m->height < (int)o->height) continue;
        if (o->cam_fps && m->fps != (int)o->cam_fps) continue;
        /* auto: the fastest mode (shortest frame interval = freshest frames); ties: the widest field of view */
        if (!mode || m->fps > mode->fps || (m->fps == mode->fps && m->width * m->height > mode->width * mode->height))
            mode = m;
    }
    if (!mode) { fprintf(stderr, "hz: ERROR no sensor mode for %ux%u cam-fps=%u\n", o->width, o->height, o->cam_fps); return -1; }
    printf("hz: sensor %s mode %dx%d @ %d fps\n", sensor.sensor_info.sensor_name, mode->width, mode->height, mode->fps);
    cam->sensor_fps = (unsigned)mode->fps;

    *fail = CAM_FAIL_CAMERA;
    rk_aiq_sys_ctx_t *aiq = c->aiq = rk_aiq_uapi2_sysctl_init(sensor.sensor_info.sensor_name, o->iq_dir, NULL, NULL);
    if (!aiq) { fprintf(stderr, "hz: ERROR rk_aiq init\n"); return -1; }
    CHECK(rk_aiq_uapi2_sysctl_prepare(aiq, mode->width, mode->height, RK_AIQ_WORKING_MODE_NORMAL));
    CHECK(rk_aiq_uapi2_sysctl_start(aiq));
    /* after start: the ISP resets the sensor orientation while it starts */
    CHECK(rk_aiq_uapi2_setMirrorFlip(aiq, o->mirror != 0, o->flip != 0, 3));
    CHECK(RK_MPI_SYS_Init()); c->sys = 1;
    VI_DEV_ATTR_S da;
    memset(&da, 0, sizeof da);
    int rc = RK_MPI_VI_GetDevAttr(0, &da);
    if (rc == RK_ERR_VI_NOT_CONFIG) CHECK(RK_MPI_VI_SetDevAttr(0, &da));
    else if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VI_GetDevAttr = %#x\n", rc); return -1; }
    CHECK(RK_MPI_VI_EnableDev(0)); c->dev = 1;
    VI_DEV_BIND_PIPE_S bind;
    memset(&bind, 0, sizeof bind);
    bind.u32Num = 1; bind.PipeId[0] = 0;
    CHECK(RK_MPI_VI_SetDevBindPipe(0, &bind));
    VI_CHN_ATTR_S va;
    memset(&va, 0, sizeof va);
    va.stIspOpt.u32BufCount = 3;
    va.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    va.stSize.u32Width = o->width; va.stSize.u32Height = o->height;
    va.enPixelFormat = RK_FMT_YUV420SP;
    va.enCompressMode = COMPRESS_MODE_NONE;
    va.u32Depth = 1;
    CHECK(RK_MPI_VI_SetChnAttr(0, 0, &va));
    CHECK(RK_MPI_VI_EnableChn(0, 0)); c->chn = 1;

    /* First frame tells the buffer layout the encoder must accept. */
    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof frame);
    CHECK(RK_MPI_VI_GetChnFrame(0, 0, &frame, 3000));
    c->vir_w = frame.stVFrame.u32VirWidth; c->vir_h = frame.stVFrame.u32VirHeight;
    printf("hz: camera frame %ux%u (buffer %ux%u)\n", frame.stVFrame.u32Width, frame.stVFrame.u32Height, c->vir_w, c->vir_h);
    RK_MPI_VI_ReleaseChnFrame(0, 0, &frame);
    if (frame.stVFrame.u32Width != o->width || frame.stVFrame.u32Height != o->height || c->vir_w < o->width ||
        c->vir_h < o->height) {
        fprintf(stderr, "hz: ERROR camera did not deliver %ux%u\n", o->width, o->height);
        return -1;
    }

    *fail = CAM_FAIL_ENCODER;
    if (open_encoder(c, o, kbps)) return -1;
    VENC_RECV_PIC_PARAM_S recv;
    memset(&recv, 0, sizeof recv);
    recv.s32RecvPicNum = -1;
    CHECK(RK_MPI_VENC_StartRecvFrame(0, &recv)); c->receiving = 1;
    return 0;
}

int camera_grab(camera_t *cam, volatile sig_atomic_t *stop, uint64_t *pts, uint64_t *drained) {
    lf_t *c = cam->impl;
    VIDEO_FRAME_INFO_S *frame = &c->frame;
    memset(frame, 0, sizeof *frame);
    /* Non-blocking poll every 1 ms: the blocking call only notices a new frame on a 10 ms grid. */
    int rc;
    double t_end = now_s() + 2.0;
    while ((rc = RK_MPI_VI_GetChnFrame(0, 0, frame, 0)) != 0 && !*stop && now_s() < t_end) sleep_s(0.001);
    if (rc && *stop) return 1;  /* a stop request interrupted the wait: normal exit */
    if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VI_GetChnFrame = %#x\n", rc); return -1; }
    for (unsigned i = 0; i < 4; i++) {  /* take the newest frame if older ones queued up */
        VIDEO_FRAME_INFO_S newer;
        memset(&newer, 0, sizeof newer);
        if (RK_MPI_VI_GetChnFrame(0, 0, &newer, 0)) break;
        RK_MPI_VI_ReleaseChnFrame(0, 0, frame);
        *frame = newer;
        (*drained)++;
    }
    *pts = frame->stVFrame.u64PTS;
    return 0;
}

void camera_drop(camera_t *cam) { RK_MPI_VI_ReleaseChnFrame(0, 0, &((lf_t *)cam->impl)->frame); }

int camera_encode(camera_t *cam, const uint8_t **au, size_t *len, double *enc_ms) {
    lf_t *c = cam->impl;
    VIDEO_FRAME_INFO_S *frame = &c->frame;
    int rc;
    double te = now_s();
    rc = RK_MPI_VENC_SendFrame(0, frame, 2000);
    RK_MPI_VI_ReleaseChnFrame(0, 0, frame);
    if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_SendFrame = %#x\n", rc); return -1; }
    VENC_PACK_S *pack = &c->pack;
    VENC_STREAM_S *stream = &c->stream;
    memset(pack, 0, sizeof *pack);
    memset(stream, 0, sizeof *stream);
    stream->pstPack = pack;
    stream->u32PackCount = 1;
    rc = RK_MPI_VENC_GetStream(0, stream, 2000);
    if (rc) { fprintf(stderr, "hz: ERROR RK_MPI_VENC_GetStream = %#x\n", rc); return -1; }
    *enc_ms = (now_s() - te) * 1000.0;
    *au = (const uint8_t *)RK_MPI_MB_Handle2VirAddr(pack->pMbBlk);
    *len = pack->u32Len;  /* the address already points at this frame (u32Offset = its place in the ring) */
    return 0;
}

void camera_done(camera_t *cam) { RK_MPI_VENC_ReleaseStream(0, &((lf_t *)cam->impl)->stream); }

void camera_close(camera_t *cam) {
    lf_t *c = cam->impl;
    if (!c) return;
    if (c->receiving) RK_MPI_VENC_StopRecvFrame(0);
    if (c->venc) RK_MPI_VENC_DestroyChn(0);
    if (c->chn) RK_MPI_VI_DisableChn(0, 0);
    if (c->dev) RK_MPI_VI_DisableDev(0);
    if (c->sys) RK_MPI_SYS_Exit();
    if (c->aiq) { rk_aiq_uapi2_sysctl_stop(c->aiq, false); rk_aiq_uapi2_sysctl_deinit(c->aiq); }
    free(c);
    cam->impl = NULL;
}
