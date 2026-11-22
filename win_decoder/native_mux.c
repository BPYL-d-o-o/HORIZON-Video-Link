/* MP4 writer for the Windows desktop receiver's "Export video": the received H.265 access units (Annex-B, exactly as
 * they came over the air, no re-encoding) into an .mp4 with their own timestamps. Unmodified FFmpeg libavformat (the
 * mp4 muxer only); built into hz_decoder.dll by build_native.sh, not into the browser decoder. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "libavformat/avformat.h"

typedef struct {
    AVFormatContext *oc;
    AVStream *st;
    AVPacket *pkt;
    int64_t first, last;
} Mux;

int hz_mux_close(void *h);

/* path: UTF-8. extradata: the stream's VPS/SPS/PPS in Annex-B (start codes). Returns NULL on failure. */
void *hz_mux_open(const char *path, const uint8_t *extradata, int extradata_size, int width, int height)
{
    Mux *m = calloc(1, sizeof(*m));
    if (!m)
        return NULL;
    m->first = INT64_MIN;
    m->last = INT64_MIN;
    if (avformat_alloc_output_context2(&m->oc, NULL, "mp4", path) < 0 || !m->oc)
        goto fail;
    m->st = avformat_new_stream(m->oc, NULL);
    if (!m->st)
        goto fail;
    AVCodecParameters *p = m->st->codecpar;
    p->codec_type = AVMEDIA_TYPE_VIDEO;
    p->codec_id = AV_CODEC_ID_HEVC;
    p->width = width;
    p->height = height;
    p->extradata = av_mallocz(extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!p->extradata)
        goto fail;
    memcpy(p->extradata, extradata, extradata_size);
    p->extradata_size = extradata_size;
    m->st->time_base = (AVRational){1, 1000000};
    if (avio_open(&m->oc->pb, path, AVIO_FLAG_WRITE) < 0)
        goto fail;
    if (avformat_write_header(m->oc, NULL) < 0)
        goto fail;
    m->pkt = av_packet_alloc();
    if (!m->pkt)
        goto fail;
    return m;
fail:
    hz_mux_close(m);
    return NULL;
}

/* One access unit (Annex-B), its time in microseconds (any origin), key: it can be decoded on its own. */
int hz_mux_write(void *h, const uint8_t *data, int size, int64_t time_us, int key)
{
    Mux *m = h;
    if (!m || !m->pkt)
        return -1;
    if (m->first == INT64_MIN)
        m->first = time_us;
    int64_t t = av_rescale_q(time_us - m->first, (AVRational){1, 1000000}, m->st->time_base);
    if (t <= m->last)
        t = m->last + 1; /* strictly increasing, whatever the clock did */
    m->last = t;
    if (av_new_packet(m->pkt, size) < 0)
        return -1;
    memcpy(m->pkt->data, data, size);
    m->pkt->pts = m->pkt->dts = t;
    m->pkt->stream_index = m->st->index;
    if (key)
        m->pkt->flags |= AV_PKT_FLAG_KEY;
    int ret = av_interleaved_write_frame(m->oc, m->pkt);
    av_packet_unref(m->pkt);
    return ret;
}

/* Finishes the file (and frees everything, also after a failed open). Returns 0 when the file is complete. */
int hz_mux_close(void *h)
{
    Mux *m = h;
    int ret = -1;
    if (!m)
        return -1;
    if (m->oc && m->pkt)
        ret = av_write_trailer(m->oc);
    av_packet_free(&m->pkt);
    if (m->oc) {
        if (m->oc->pb)
            avio_closep(&m->oc->pb);
        avformat_free_context(m->oc);
    }
    free(m);
    return ret;
}
