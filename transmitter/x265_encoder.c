/* hz_x265: the Raspberry Pi's H.265 encoder, a separate process that hz_vtx (camera_pi.cpp) starts and feeds over
 * pipes. It is its own program because x265 is GPL: hz_vtx itself does not link it.
 *
 *   hz_x265 W H FPS PRESET KEYINT SETTINGS        SETTINGS: x265 settings, name=value:name=value
 *   stdin, per frame:  "FRM1" u32 flags (bit 0: keyframe), then W x H I420
 *   stdout, per frame: "AU01" u32 n, then the access unit: Annex B, no AUD, parameter sets with a keyframe
 *
 * Zero latency: no lookahead, no B-frames, one frame thread; one frame in, one access unit out. Gradual intra
 * refresh instead of keyframes (KEYINT = the refresh period); the rate control comes with SETTINGS (bitrate, VBV). */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <x265.h>

static int read_all(void *p, size_t n) {
    for (size_t got = 0, r; got < n; got += r)
        if (!(r = fread((char *)p + got, 1, n - got, stdin))) return 0;
    return 1;
}

int main(int argc, char **argv) {
    if (argc != 7) {
        fprintf(stderr, "usage: hz_x265 W H FPS PRESET KEYINT SETTINGS\n");
        return 2;
    }
    int w = atoi(argv[1]), h = atoi(argv[2]);
    x265_param *p = x265_param_alloc();
    if (!p || x265_param_default_preset(p, argv[4], "zerolatency") < 0) { fprintf(stderr, "hz_x265: preset %s\n", argv[4]); return 2; }
    p->sourceWidth = w;
    p->sourceHeight = h;
    p->fpsNum = (uint32_t)(atof(argv[3]) * 1000 + 0.5);
    p->fpsDenom = 1000;
    p->internalCsp = X265_CSP_I420;
    p->logLevel = X265_LOG_ERROR;
    p->keyframeMax = p->keyframeMin = atoi(argv[5]);
    p->bOpenGOP = 0;
    p->bframes = 0;
    p->scenecutThreshold = 0;
    p->lookaheadDepth = 0;
    p->frameNumThreads = 1;
    p->bRepeatHeaders = 1;
    p->bEnableAccessUnitDelimiters = 0;
    p->bAnnexB = 1;
    p->bEmitInfoSEI = 0;  /* a ~2 KB SEI with every header set */
    p->bIntraRefresh = 1;
    char set[1024];
    snprintf(set, sizeof set, "%s", argv[6]);
    for (char *kv = strtok(set, ":"); kv; kv = strtok(NULL, ":")) {
        char *eq = strchr(kv, '=');
        if (eq) *eq = 0;
        if (x265_param_parse(p, kv, eq ? eq + 1 : "1")) { fprintf(stderr, "hz_x265: setting %s\n", kv); return 2; }
    }
    x265_encoder *enc = x265_encoder_open(p);
    if (!enc) { fprintf(stderr, "hz_x265: x265 does not start\n"); return 1; }
    size_t fsz = (size_t)w * h * 3 / 2;
    uint8_t *frame = malloc(fsz), *au = malloc(1 << 20);
    x265_picture pic;
    x265_picture_init(p, &pic);
    pic.stride[0] = w;
    pic.stride[1] = pic.stride[2] = w / 2;
    pic.planes[0] = frame;
    pic.planes[1] = frame + w * h;
    pic.planes[2] = frame + w * h * 5 / 4;
    for (int64_t n = 0;; n++) {
        char magic[4];
        uint32_t flags;
        if (!read_all(magic, 4) || memcmp(magic, "FRM1", 4) || !read_all(&flags, 4) || !read_all(frame, fsz)) break;
        pic.sliceType = n == 0 || (flags & 1) ? X265_TYPE_IDR : X265_TYPE_AUTO;
        pic.pts = n;
        x265_nal *nals;
        uint32_t nn = 0, len = 0;
        if (x265_encoder_encode(enc, &nals, &nn, &pic, NULL) < 0) { fprintf(stderr, "hz_x265: encode failed\n"); return 1; }
        for (uint32_t i = 0; i < nn; i++)
            if (nals[i].type != 35 && len + nals[i].sizeBytes <= (1u << 20)) {  /* no AUD */
                memcpy(au + len, nals[i].payload, nals[i].sizeBytes);
                len += nals[i].sizeBytes;
            }
        fwrite("AU01", 1, 4, stdout);
        fwrite(&len, 4, 1, stdout);
        fwrite(au, 1, len, stdout);
        fflush(stdout);
    }
    x265_encoder_close(enc);
    x265_param_free(p);
    return 0;
}
