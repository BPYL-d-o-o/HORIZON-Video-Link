// x265 frame server for the Pi 5 video sender.
//
// Usage: frame_enc W H FPS PRESET KEYINT [INTRA_REFRESH 0|1] [X265_PARAMS k=v:k=v] [QP_MIN] [QP_MAX_DROP]
// stdin, per frame:  "FRM1" u32 target_bytes, u32 flags (bit0: force IDR), then W*H*3/2 I420.
// stdout, per frame: "AU01" u32 au_bytes, i32 qp, u32 type (1=IDR, 2=P, 3=P with an intra-refresh column), u32 pred_bytes,
//                    then the Annex-B access unit (parameter sets on IDR, no AUD).
// stderr: "HDR" u32 n + VPS/SPS/PPS once at start (for the receiver), then log text.
//
// Zero latency: no lookahead, no B-frames, one frame thread; one frame in, one AU out.
// The base QP of every frame is chosen here so the predicted size fits target_bytes:
//   size ~= c[type] * complexity * 2^(-qp/6)
// complexity = luma SAD vs the previous source (P) or luma gradient energy (I),
// c[type] learned from actual sizes (log-domain EMA).
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <x265.h>

static int W, H;

static int read_all(void* p, size_t n) {
    size_t got = 0;
    while (got < n) {
        size_t r = fread((char*)p + got, 1, n - got, stdin);
        if (!r) return 0;
        got += r;
    }
    return 1;
}
static double sad_p(const uint8_t* a, const uint8_t* b) {
    uint64_t s = 0;
    for (int i = 0; i < W * H; i++) s += (unsigned)abs(a[i] - b[i]);
    return (double)s / (W * H);
}
static double grad_i(const uint8_t* y) {
    uint64_t s = 0;
    for (int r = 0; r < H - 1; r++)
        for (int c = 0; c < W - 1; c++) {
            const uint8_t* p = y + r * W + c;
            s += (unsigned)abs(p[0] - p[1]) + (unsigned)abs(p[0] - p[W]);
        }
    return (double)s / (W * H);
}

int main(int argc, char** argv) {
    if (argc < 6) {
        fprintf(stderr, "usage: frame_enc W H FPS PRESET KEYINT [INTRA_REFRESH]\n");
        return 2;
    }
    W = atoi(argv[1]);
    H = atoi(argv[2]);
    double fps = atof(argv[3]);
    const char* preset = argv[4];
    int keyint = atoi(argv[5]);
    int refresh = argc > 6 ? atoi(argv[6]) : 1;
    const char* extra = argc > 7 ? argv[7] : "";
    const int qpMinArg = argc > 8 ? atoi(argv[8]) : 12;
    const int maxDrop = argc > 9 ? atoi(argv[9]) : 3;

    x265_param* p = x265_param_alloc();
    if (x265_param_default_preset(p, preset, "zerolatency") < 0) {
        fprintf(stderr, "bad preset %s\n", preset);
        return 2;
    }
    p->sourceWidth = W;
    p->sourceHeight = H;
    p->fpsNum = (uint32_t)(fps * 1000 + 0.5);
    p->fpsDenom = 1000;
    p->internalCsp = X265_CSP_I420;
    p->logLevel = X265_LOG_ERROR;
    p->keyframeMax = keyint;
    p->keyframeMin = keyint;
    p->bOpenGOP = 0;
    p->bframes = 0;
    p->scenecutThreshold = 0;
    p->lookaheadDepth = 0;
    p->frameNumThreads = 1;
    p->bRepeatHeaders = 1;
    p->bEnableAccessUnitDelimiters = 0;
    p->bAnnexB = 1;
    p->bEmitInfoSEI = 0;  // the ~2 KB version/options SEI rode along with every repeated header set
    p->bIntraRefresh = refresh;  // column refresh instead of periodic IDR
    // 16x16 CTUs: the refresh sweeps W/16 columns, one per frame, so each refresh
    // frame carries half the intra cost of the default 32x32 CTU.
    p->maxCUSize = 16;
    p->minCUSize = 8;
    p->rc.rateControlMode = X265_RC_CQP;
    p->rc.qp = 38;
    // Tuning overrides ("name=value:name=value"), applied last so they win.
    if (*extra) {
        char buf[1024];
        snprintf(buf, sizeof buf, "%s", extra);
        for (char* kv = strtok(buf, ":"); kv; kv = strtok(NULL, ":")) {
            char* eq = strchr(kv, '=');
            if (eq) *eq = 0;
            if (x265_param_parse(p, kv, eq ? eq + 1 : "1") != 0) { fprintf(stderr, "bad x265 param %s\n", kv); return 2; }
        }
    }
    x265_encoder* enc = x265_encoder_open(p);
    if (!enc) {
        fprintf(stderr, "x265_encoder_open failed\n");
        return 1;
    }
    // Parameter sets for the receiver (it never gets them over RF).
    x265_nal* nals;
    uint32_t nn;
    if (x265_encoder_headers(enc, &nals, &nn) >= 0) {
        uint32_t total = 0;
        for (uint32_t i = 0; i < nn; i++) total += nals[i].sizeBytes;
        fwrite("HDR", 1, 3, stderr);
        fwrite(&total, 4, 1, stderr);
        for (uint32_t i = 0; i < nn; i++) fwrite(nals[i].payload, 1, nals[i].sizeBytes, stderr);
        fflush(stderr);
    }

    const size_t fsz = (size_t)W * H * 3 / 2;
    uint8_t* cur = malloc(fsz);
    uint8_t* prev = calloc(1, fsz);
    uint8_t* out = malloc(1 << 20);
    x265_picture pic;
    x265_picture_init(p, &pic);
    pic.stride[0] = W;
    pic.stride[1] = pic.stride[2] = W / 2;
    pic.planes[0] = cur;
    pic.planes[1] = cur + W * H;
    pic.planes[2] = cur + W * H + W * H / 4;
    // Size model per frame kind: 0 = IDR, 1 = P, 2 = P carrying a refresh column.
    // x265 sweeps the refresh over the first ceil(W/16) frames of every keyint period.
    double cm[3] = {0, 0, 0};
    const int ctu = (int)p->maxCUSize;
    const int cols = (W + ctu - 1) / ctu;
    // Below ~20 the bits buy little at this size and the model can be fooled by static
    // scenes into very expensive refresh columns.
    const int QP_MIN = qpMinArg;
    int lastAny = 38, lastVbv = 38;

    for (int64_t n = 0;; n++) {
        char magic[4];
        uint32_t hdr[2];
        if (!read_all(magic, 4) || memcmp(magic, "FRM1", 4) || !read_all(hdr, 8) || !read_all(cur, fsz)) break;
        const double target = hdr[0];
        const int forceIdr = n == 0 || (hdr[1] & 1);
        const int kind = forceIdr ? 0 : (refresh && keyint > 0 && n >= keyint && n % keyint < cols) ? 2 : 1;
        // Complexity: IDR = luma gradient energy; refresh frame = motion SAD plus one
        // column's share of the intra cost; P = motion SAD vs the previous source.
        const double sad = forceIdr ? 0 : sad_p(cur, prev) + 0.05;
        const double cplx = kind == 0 ? grad_i(cur) + 0.5 : kind == 2 ? sad + (grad_i(cur) + 0.5) / cols : sad;
        // The first refresh frame has no history: its cost is mostly intra, so start
        // from the IDR model (same gradient-energy complexity term).
        const double c = cm[kind] ? cm[kind] : kind == 2 ? cm[0] : 0;
        int qp;
        if (c == 0) {
            qp = lastAny + (kind == 0 ? 6 : 0);
        } else {
            qp = 51;
            for (int q = QP_MIN; q <= 51; q++)
                if (c * cplx * pow(2, -q / 6.0) <= target) { qp = q; break; }
            if (qp < lastAny - maxDrop) qp = lastAny - maxDrop;  // recover quality gradually, from any frame kind
        }
        lastAny = qp;
        const double pred = c * cplx * pow(2, -qp / 6.0);
        // With bitrate/VBV in the tuning string, x265's own (row-level) rate control decides QP.
        // (A model-based latency guard on top was tried: forcing QP disturbs VBV and made it worse.)
        pic.forceqp = p->rc.rateControlMode == X265_RC_CQP ? qp + 1 : 0;
        pic.sliceType = forceIdr ? X265_TYPE_IDR : X265_TYPE_AUTO;
        pic.pts = n;
        x265_picture outpic;
        int r = x265_encoder_encode(enc, &nals, &nn, &pic, &outpic);
        if (r < 0) {
            fprintf(stderr, "encode failed\n");
            return 1;
        }
        // Learn and report with the QP actually used (x265's in VBV mode, unless forced).
        if (p->rc.rateControlMode != X265_RC_CQP && nn) { qp = (int)(outpic.frameData.qp + 0.5); lastVbv = qp; }
        uint32_t au = 0, vcl = 0;
        for (uint32_t i = 0; i < nn; i++) {
            if (nals[i].type == 35) continue;  // AUD
            memcpy(out + au, nals[i].payload, nals[i].sizeBytes);
            au += nals[i].sizeBytes;
            if (nals[i].type < 32) vcl += nals[i].sizeBytes;
        }
        // Learn the size model from the bytes that actually go on air (no parameter sets).
        if (vcl) {
            const double obs = vcl * pow(2, qp / 6.0) / cplx;
            cm[kind] = cm[kind] == 0 ? obs : exp(0.6 * log(cm[kind]) + 0.4 * log(obs));
        }
        const uint32_t type = kind == 0 ? 1 : kind == 2 ? 3 : 2;
        const uint32_t predb = (uint32_t)pred;
        fwrite("AU01", 1, 4, stdout);
        fwrite(&au, 4, 1, stdout);
        fwrite(&qp, 4, 1, stdout);
        fwrite(&type, 4, 1, stdout);
        fwrite(&predb, 4, 1, stdout);
        fwrite(out, 1, au, stdout);
        fflush(stdout);
        memcpy(prev, cur, fsz);
    }
    x265_encoder_close(enc);
    x265_param_free(p);
    return 0;
}
