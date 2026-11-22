/* H.265 decoding through an unmodified FFmpeg libavcodec, with conversion of
 * the decoded picture to BGRA for display. */
#include "decoder.h"

#include <stdlib.h>
#include <string.h>

#include "libavcodec/avcodec.h"

struct Decoder {
    AVCodecContext *codec;
    AVPacket *packet;
    AVFrame *frame;
    int have_picture;
};

void decoder_free(Decoder *d)
{
    if (!d)
        return;
    avcodec_free_context(&d->codec);
    av_packet_free(&d->packet);
    av_frame_free(&d->frame);
    free(d);
}

Decoder *decoder_new(void)
{
    Decoder *d = calloc(1, sizeof(*d));
    const AVCodec *hevc = avcodec_find_decoder(AV_CODEC_ID_HEVC);

    /* Damaged pictures are expected on a radio link; do not print each one. */
    av_log_set_level(AV_LOG_QUIET);
    if (!d || !hevc)
        goto fail;
    d->codec = avcodec_alloc_context3(hevc);
    d->packet = av_packet_alloc();
    d->frame = av_frame_alloc();
    if (!d->codec || !d->packet || !d->frame)
        goto fail;
    d->codec->thread_count = 1;
    /* Low delay: hand out each picture as soon as it is decoded, and keep
     * showing pictures after a loss instead of waiting for a keyframe. */
    d->codec->flags |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_OUTPUT_CORRUPT;
    d->codec->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
    if (avcodec_open2(d->codec, hevc, NULL) < 0)
        goto fail;
    return d;
fail:
    decoder_free(d);
    return NULL;
}

int decoder_send(Decoder *d, const uint8_t *data, int size)
{
    int ret = av_new_packet(d->packet, size);
    if (ret < 0)
        return ret;
    memcpy(d->packet->data, data, size);
    ret = avcodec_send_packet(d->codec, d->packet);
    av_packet_unref(d->packet);
    return ret;
}

int decoder_receive(Decoder *d, int *width, int *height)
{
    AVFrame *next = av_frame_alloc();
    int got = 0;

    if (!next)
        return 0;
    /* Keep only the newest picture. */
    while (avcodec_receive_frame(d->codec, next) >= 0) {
        if (next->format == AV_PIX_FMT_YUV420P || next->format == AV_PIX_FMT_YUVJ420P) {
            av_frame_unref(d->frame);
            av_frame_move_ref(d->frame, next);
            d->have_picture = got = 1;
        } else {
            av_frame_unref(next);
        }
    }
    av_frame_free(&next);
    if (got) {
        *width = d->frame->width;
        *height = d->frame->height;
    }
    return got;
}

static uint8_t clamp(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

void decoder_copy_bgra(const Decoder *d, uint8_t *out, int pitch)
{
    const AVFrame *f = d->frame;
    const int full = f->color_range == AVCOL_RANGE_JPEG || f->format == AV_PIX_FMT_YUVJ420P;
    const int bt709 = f->colorspace == AVCOL_SPC_BT709;
    /* 16.16 fixed point: R = y + rv*V, G = y - gu*U - gv*V, B = y + bu*U. */
    const int rv = bt709 ? 103206 : 91881, gu = bt709 ? 12276 : 22553, gv = bt709 ? 30679 : 46802,
              bu = bt709 ? 121609 : 116130;
    const int yscale = full ? 65536 : 76309, cscale = full ? 65536 : 74898, yoffset = full ? 0 : 16;

    if (!d->have_picture)
        return;
    for (int y = 0; y < f->height; y++) {
        const uint8_t *ly = f->data[0] + y * f->linesize[0];
        const uint8_t *lu = f->data[1] + (y / 2) * f->linesize[1];
        const uint8_t *lv = f->data[2] + (y / 2) * f->linesize[2];
        uint8_t *to = out + (size_t)y * pitch;
        for (int x = 0; x < f->width; x++) {
            const int luma = (ly[x] - yoffset) * yscale;
            const int u = (int)(((int64_t)(lu[x / 2] - 128) * cscale) >> 16), v = (int)(((int64_t)(lv[x / 2] - 128) * cscale) >> 16);
            to[4 * x + 0] = clamp((luma + bu * u) >> 16);
            to[4 * x + 1] = clamp((luma - gu * u - gv * v) >> 16);
            to[4 * x + 2] = clamp((luma + rv * v) >> 16);
            to[4 * x + 3] = 255;
        }
    }
}
