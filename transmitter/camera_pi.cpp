// The Raspberry Pi 5's camera and H.265 encoder (camera.h): libcamera (its API is C++, hence this one C++ file),
// and x265 in its own process (hz_x265, x265_encoder.c; x265 is GPL, so hz_vtx does not link it), fed over pipes.
//
// ISP noise reduction on: sensor noise costs bits at ~60 kbit/s. Only the newest frame is kept.
// x265: zero latency and gradual intra refresh instead of keyframes. With constrained intra, temporal MVP off, one
// reference and no deblocking, a picture damaged by a lost packet is bit-exact again after the next refresh sweep.
// rdoq-level 2, psy-rdoq / psy-rd 1, early-skip off and TU depth 2 beat plain superfast at this bitrate; slower
// presets lose (they turn SAO on).
// --source FILE: a test video, converted once by ffmpeg (centre crop) into ~/.cache/horizon, looped at --fps.
#include "camera.h"

#include <errno.h>
#include <fcntl.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <libcamera/libcamera.h>
#include <signal.h>

extern char **environ;
using namespace libcamera;

static std::string stem_of(const std::string &path) { return path.substr(path.find_last_of('/') + 1); }

static uint64_t mono_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

namespace {
struct Pi {
    unsigned w = 0, h = 0;
    std::unique_ptr<CameraManager> cm;
    std::shared_ptr<Camera> cam;
    std::unique_ptr<CameraConfiguration> cfg;
    std::unique_ptr<FrameBufferAllocator> alloc;
    std::vector<std::unique_ptr<Request>> reqs;
    std::vector<std::pair<void *, size_t>> maps;
    std::map<const FrameBuffer *, std::vector<const uint8_t *>> planes;
    Stream *stream = nullptr;
    unsigned stride = 0;
    bool acquired = false, started = false;
    std::mutex mu;
    std::condition_variable cv;
    std::vector<uint8_t> latest, cur;  // packed I420, w x h
    uint64_t latest_pts = 0, count = 0, seen = 0;
    FILE *src = nullptr;
    uint64_t period_us = 0, next_us = 0;
    pid_t enc = 0;
    FILE *to_enc = nullptr, *from_enc = nullptr;
    std::vector<uint8_t> au;
    int64_t n = 0;

    void done(Request *r);
};

void Pi::done(Request *r) {
    if (r->status() == Request::RequestCancelled) return;
    const FrameBuffer *b = r->buffers().begin()->second;
    const std::vector<const uint8_t *> &pl = planes[b];
    uint64_t now = mono_us(), pts = now;
    std::optional<int64_t> ts = r->metadata().get(controls::SensorTimestamp);  // ns
    if (ts && *ts > 0 && now >= (uint64_t)*ts / 1000 && now - (uint64_t)*ts / 1000 < 1000000) pts = (uint64_t)*ts / 1000;
    {
        std::lock_guard<std::mutex> l(mu);
        uint8_t *d = latest.data();
        for (unsigned y = 0; y < h; y++, d += w) memcpy(d, pl[0] + (size_t)y * stride, w);
        for (unsigned k = 1; k <= 2; k++)
            for (unsigned y = 0; y < h / 2; y++, d += w / 2) memcpy(d, pl[k] + (size_t)y * (stride / 2), w / 2);
        latest_pts = pts;
        count++;
    }
    cv.notify_all();
    r->reuse(Request::ReuseBuffers);
    cam->queueRequest(r);
}

unsigned bit_depth(const PixelFormat &f) {  // "SBGGR10_CSI2P" -> 10
    std::string s = f.toString();
    size_t i = s.find_first_of("0123456789");
    return i == std::string::npos ? 0 : (unsigned)atoi(s.c_str() + i);
}

// The widest field of view among the modes that reach fps, then the most pixels: left to itself, libcamera picks by
// output size only and can land on a mode that cannot reach the frame rate.
std::optional<SensorConfiguration> pick_mode(Pi *c, double fps) {
    std::unique_ptr<CameraConfiguration> raw = c->cam->generateConfiguration({StreamRole::Raw});
    if (!raw) return std::nullopt;
    std::optional<SensorConfiguration> best;
    double best_fps = 0;
    uint64_t best_fov = 0, best_px = 0;
    for (const PixelFormat &f : raw->at(0).formats().pixelformats()) {
        unsigned bd = bit_depth(f);
        if (!bd) continue;
        for (const Size &s : raw->at(0).formats().sizes(f)) {
            std::unique_ptr<CameraConfiguration> t = c->cam->generateConfiguration({StreamRole::VideoRecording});
            t->at(0).size = Size(c->w, c->h);
            t->at(0).pixelFormat = formats::YUV420;
            SensorConfiguration sc;
            sc.bitDepth = bd;
            sc.outputSize = s;
            t->sensorConfig = sc;
            if (t->validate() == CameraConfiguration::Invalid || c->cam->configure(t.get())) continue;
            const ControlInfoMap &ci = c->cam->controls();
            auto fd = ci.find(&controls::FrameDurationLimits);
            auto cr = ci.find(&controls::ScalerCrop);
            if (fd == ci.end()) continue;
            double mfps = 1e6 / (double)fd->second.min().get<int64_t>();
            uint64_t fov = cr != ci.end() ? (uint64_t)cr->second.max().get<Rectangle>().size().width *
                                                cr->second.max().get<Rectangle>().size().height : 0;
            uint64_t px = (uint64_t)s.width * s.height;
            if (mfps < fps - 0.01) continue;
            if (!best || fov > best_fov || (fov == best_fov && px > best_px)) {
                best = sc; best_fps = mfps; best_fov = fov; best_px = px;
            }
        }
    }
    if (best) printf("camera: sensor mode %ux%u (max %.1f fps) for %.1f fps\n", best->outputSize.width,
                     best->outputSize.height, best_fps, fps);
    else printf("camera: no sensor mode reaches %.1f fps, using the default mode\n", fps);
    return best;
}

int open_camera(Pi *c, const opts_t *o) {
    c->cm = std::make_unique<CameraManager>();
    if (c->cm->start()) { fprintf(stderr, "hz: ERROR libcamera does not start\n"); return -1; }
    if (c->cm->cameras().empty()) { fprintf(stderr, "hz: ERROR no camera\n"); return -1; }
    c->cam = c->cm->cameras()[0];
    if (c->cam->acquire()) { fprintf(stderr, "hz: ERROR the camera is in use\n"); return -1; }
    c->acquired = true;
    std::optional<SensorConfiguration> mode = pick_mode(c, o->fps);
    c->cfg = c->cam->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration &sc = c->cfg->at(0);
    sc.size = Size(c->w, c->h);
    sc.pixelFormat = formats::YUV420;
    sc.bufferCount = 4;
    if (mode) c->cfg->sensorConfig = mode;
    if (c->cfg->validate() == CameraConfiguration::Invalid || sc.size != Size(c->w, c->h) ||
        sc.pixelFormat != formats::YUV420 || c->cam->configure(c->cfg.get())) {
        fprintf(stderr, "hz: ERROR the camera cannot deliver %ux%u YUV420\n", c->w, c->h);
        return -1;
    }
    c->stream = sc.stream();
    c->stride = sc.stride;
    c->alloc = std::make_unique<FrameBufferAllocator>(c->cam);
    if (c->alloc->allocate(c->stream) < 0) { fprintf(stderr, "hz: ERROR camera buffers\n"); return -1; }
    for (const std::unique_ptr<FrameBuffer> &b : c->alloc->buffers(c->stream)) {
        std::map<int, uint8_t *> fds;  // one mapping per dmabuf, the planes at their offsets
        size_t len = 0;
        for (const FrameBuffer::Plane &pl : b->planes()) len = std::max(len, (size_t)(pl.offset + pl.length));
        std::vector<const uint8_t *> pp;
        for (const FrameBuffer::Plane &pl : b->planes()) {
            int fd = pl.fd.get();
            if (!fds.count(fd)) {
                void *m = mmap(nullptr, len, PROT_READ, MAP_SHARED, fd, 0);
                if (m == MAP_FAILED) { fprintf(stderr, "hz: ERROR mmap: %s\n", strerror(errno)); return -1; }
                c->maps.push_back({m, len});
                fds[fd] = (uint8_t *)m;
            }
            pp.push_back(fds[fd] + pl.offset);
        }
        if (pp.size() != 3) { fprintf(stderr, "hz: ERROR the camera buffer has %zu planes, not 3\n", pp.size()); return -1; }
        c->planes[b.get()] = pp;
        std::unique_ptr<Request> r = c->cam->createRequest();
        if (!r || r->addBuffer(c->stream, b.get())) { fprintf(stderr, "hz: ERROR camera request\n"); return -1; }
        c->reqs.push_back(std::move(r));
    }
    c->cam->requestCompleted.connect(c, &Pi::done);
    ControlList ctl(c->cam->controls());
    int64_t period = (int64_t)(1e6 / o->fps);
    ctl.set(controls::FrameDurationLimits, std::span<const int64_t, 2>({period, period}));
    ctl.set(controls::draft::NoiseReductionMode, controls::draft::NoiseReductionModeHighQuality);
    if (c->cam->start(&ctl)) { fprintf(stderr, "hz: ERROR the camera does not start\n"); return -1; }
    c->started = true;
    for (std::unique_ptr<Request> &r : c->reqs) c->cam->queueRequest(r.get());
    printf("hz: camera %s, %ux%u YUV420 (stride %u)\n", c->cam->id().c_str(), c->w, c->h, c->stride);
    return 0;
}

int open_source(Pi *c, const opts_t *o) {
    std::string path = o->source, clip = path;
    size_t fsz = (size_t)c->w * c->h * 3 / 2;
    struct stat st;
    if (stat(path.c_str(), &st)) { fprintf(stderr, "hz: ERROR test video not found: %s\n", path.c_str()); return -1; }
    if (path.size() < 4 || strcasecmp(path.c_str() + path.size() - 4, ".yuv")) {
        char key[160];
        snprintf(key, sizeof key, "%s|%lld|%lld|%ux%u|%.3f", path.c_str(), (long long)st.st_size, (long long)st.st_mtime,
                 c->w, c->h, o->fps);
        uint32_t hsh = 2166136261u;
        for (const char *k = key; *k; k++) hsh = (hsh ^ (uint8_t)*k) * 16777619u;
        const char *home = getenv("HOME") ? getenv("HOME") : "/tmp";
        std::string dir = std::string(home) + "/.cache/horizon", stem = path.substr(path.find_last_of('/') + 1);
        stem = stem.substr(0, stem.find_last_of('.'));
        char name[256];
        snprintf(name, sizeof name, "/%s_%ux%u_%gfps_%08x.yuv", stem.c_str(), c->w, c->h, o->fps, hsh);
        clip = dir + name;
        if (stat(clip.c_str(), &st)) {
            mkdir((std::string(home) + "/.cache").c_str(), 0755);
            mkdir(dir.c_str(), 0755);
            char vf[200];
            snprintf(vf, sizeof vf, "fps=%g,scale=%u:%u:force_original_aspect_ratio=increase:flags=area,crop=%u:%u,"
                     "format=yuv420p", o->fps, c->w, c->h, c->w, c->h);
            std::string part = clip + ".part";
            printf("source: converting %s to %ux%u at %g fps (once; cached in %s)\n", path.c_str(), c->w, c->h, o->fps,
                   dir.c_str());
            const char *argv[] = {"ffmpeg", "-v", "error", "-nostdin", "-y", "-i", path.c_str(), "-an", "-sn", "-vf", vf,
                                  "-f", "rawvideo", part.c_str(), nullptr};
            pid_t pid;
            int status = -1;
            if (posix_spawnp(&pid, "ffmpeg", nullptr, nullptr, (char *const *)argv, environ)) {
                fprintf(stderr, "hz: ERROR ffmpeg is needed for test videos: sudo apt install ffmpeg\n");
                return -1;
            }
            waitpid(pid, &status, 0);
            if (!WIFEXITED(status) || WEXITSTATUS(status) || rename(part.c_str(), clip.c_str())) {
                fprintf(stderr, "hz: ERROR ffmpeg could not convert %s\n", path.c_str());
                return -1;
            }
        }
    }
    if (stat(clip.c_str(), &st) || st.st_size < (off_t)fsz || st.st_size % fsz) {
        fprintf(stderr, "hz: ERROR %s is not a whole number of %ux%u I420 frames\n", clip.c_str(), c->w, c->h);
        return -1;
    }
    c->src = fopen(clip.c_str(), "rb");
    if (!c->src) { fprintf(stderr, "hz: ERROR cannot read %s\n", clip.c_str()); return -1; }
    c->period_us = (uint64_t)(1e6 / o->fps);
    printf("source: %s, %lld frames (%.1f s at %g fps), looped\n", stem_of(path).c_str(), (long long)(st.st_size / fsz),
           st.st_size / fsz / o->fps, o->fps);
    return 0;
}

int open_encoder(Pi *c, const opts_t *o, unsigned kbps) {
    char exe[512], set[1024], w[16], h[16], fps[32], keyint[16];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 16);  // hz_x265 sits next to hz_vtx
    if (n <= 0) { fprintf(stderr, "hz: ERROR /proc/self/exe\n"); return -1; }
    exe[n] = 0;
    snprintf(strrchr(exe, '/') + 1, 16, "hz_x265");
    snprintf(set, sizeof set, "bitrate=%u:vbv-maxrate=%u:vbv-bufsize=%u:vbv-init=0.5:ctu=32:constrained-intra:"
             "temporal-mvp=0:ref=1:no-deblock:rdoq-level=2:psy-rdoq=1:psy-rd=1:early-skip=0:tu-intra-depth=2:"
             "tu-inter-depth=2%s%s", kbps, kbps, std::max(3u, kbps * 3 / 10), o->x265_extra[0] ? ":" : "",
             o->x265_extra);
    snprintf(w, sizeof w, "%u", c->w);
    snprintf(h, sizeof h, "%u", c->h);
    snprintf(fps, sizeof fps, "%.4f", o->fps);
    snprintf(keyint, sizeof keyint, "%u", o->keyint);
    int in[2], out[2];
    if (pipe(in) || pipe(out)) { fprintf(stderr, "hz: ERROR pipe\n"); return -1; }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, in[0], 0);
    posix_spawn_file_actions_adddup2(&fa, out[1], 1);
    posix_spawn_file_actions_addclose(&fa, in[1]);
    posix_spawn_file_actions_addclose(&fa, out[0]);
    const char *argv[] = {exe, w, h, fps, o->preset, keyint, set, nullptr};
    int rc = posix_spawn(&c->enc, exe, &fa, nullptr, (char *const *)argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    close(in[0]);
    close(out[1]);
    if (rc) { fprintf(stderr, "hz: ERROR cannot start %s: %s\n", exe, strerror(rc)); c->enc = 0; return -1; }
    c->to_enc = fdopen(in[1], "wb");
    c->from_enc = fdopen(out[0], "rb");
    signal(SIGPIPE, SIG_IGN);
    printf("hz: x265 (%s) %s, %u kbit/s, intra refresh every %u frames, no keyframes after the first\n", exe, o->preset,
           kbps, o->keyint);
    return 0;
}
}  // namespace

extern "C" int camera_open(camera_t *cam, const opts_t *o, unsigned kbps, int *fail) {
    memset(cam, 0, sizeof *cam);
    Pi *c = new Pi();
    cam->impl = c;
    cam->refresh = 1;
    cam->sensor_fps = (unsigned)(o->fps + 0.5);
    c->w = o->width;
    c->h = o->height;
    c->latest.resize((size_t)c->w * c->h * 3 / 2);
    c->cur.resize(c->latest.size());
    *fail = CAM_FAIL_NO_CAMERA;
    if (o->source ? open_source(c, o) : open_camera(c, o)) {
        if (!o->source && c->acquired) *fail = CAM_FAIL_CAMERA;
        return -1;
    }
    *fail = CAM_FAIL_ENCODER;
    return open_encoder(c, o, kbps);
}

extern "C" int camera_grab(camera_t *cam, volatile sig_atomic_t *stop, uint64_t *pts, uint64_t *drained) {
    Pi *c = (Pi *)cam->impl;
    if (c->src) {  // test video: one frame per period, looped
        uint64_t now = mono_us();
        if (!c->next_us) c->next_us = now;
        c->next_us += c->period_us;
        if (c->next_us > now) usleep((useconds_t)(c->next_us - now));
        else c->next_us = now;
        if (*stop) return 1;
        if (fread(c->cur.data(), 1, c->cur.size(), c->src) != c->cur.size()) {
            rewind(c->src);
            if (fread(c->cur.data(), 1, c->cur.size(), c->src) != c->cur.size()) return -1;
        }
        *pts = mono_us();
        return 0;
    }
    std::unique_lock<std::mutex> l(c->mu);
    for (;;) {
        uint64_t t_end = mono_us() + 2000000;
        while (c->count == c->seen) {  // the next frame (the camera runs at --fps: each frame is encoded on arrival)
            if (*stop) return 1;
            if (mono_us() > t_end) { fprintf(stderr, "hz: ERROR no camera frame for 2 s\n"); return -1; }
            c->cv.wait_for(l, std::chrono::milliseconds(50));
        }
        if (c->count - c->seen > 1) *drained += c->count - c->seen - 1;
        c->seen = c->count;
        c->cur.swap(c->latest);
        // A black first keyframe (exposure settling) throws x265's rate control off for many seconds: start with the
        // first frame that is not black, or after 2 s.
        if (c->n || c->count > 60) break;
        unsigned lo = 255, hi = 0;
        for (size_t i = 0; i < (size_t)c->w * c->h; i += 7) { lo = std::min<unsigned>(lo, c->cur[i]); hi = std::max<unsigned>(hi, c->cur[i]); }
        if (hi > 20 || hi - lo > 4) break;
    }
    *pts = c->latest_pts;
    return 0;
}

extern "C" void camera_drop(camera_t *cam) { (void)cam; }

extern "C" int camera_encode(camera_t *cam, const uint8_t **au, size_t *len, double *enc_ms) {
    Pi *c = (Pi *)cam->impl;
    uint64_t t0 = mono_us();
    uint32_t flags = c->n++ == 0, n = 0;
    char magic[4];
    if (fwrite("FRM1", 1, 4, c->to_enc) != 4 || fwrite(&flags, 4, 1, c->to_enc) != 1 ||
        fwrite(c->cur.data(), 1, c->cur.size(), c->to_enc) != c->cur.size() || fflush(c->to_enc) ||
        fread(magic, 1, 4, c->from_enc) != 4 || memcmp(magic, "AU01", 4) || fread(&n, 4, 1, c->from_enc) != 1) {
        fprintf(stderr, "hz: ERROR the encoder (hz_x265) stopped\n");
        return -1;
    }
    c->au.resize(n);
    if (n && fread(c->au.data(), 1, n, c->from_enc) != n) { fprintf(stderr, "hz: ERROR the encoder (hz_x265) stopped\n"); return -1; }
    *enc_ms = (mono_us() - t0) / 1000.0;
    *au = c->au.data();
    *len = c->au.size();
    return 0;
}

extern "C" void camera_done(camera_t *cam) { (void)cam; }

extern "C" void camera_close(camera_t *cam) {
    Pi *c = (Pi *)cam->impl;
    if (!c) return;
    if (c->started) c->cam->stop();
    if (c->cam) c->cam->requestCompleted.disconnect(c);
    c->reqs.clear();
    for (auto &m : c->maps) munmap(m.first, m.second);
    c->alloc.reset();
    if (c->acquired) c->cam->release();
    c->cam.reset();
    if (c->cm) c->cm->stop();
    if (c->to_enc) fclose(c->to_enc);  // the encoder ends at the end of its input
    if (c->from_enc) fclose(c->from_enc);
    if (c->enc) waitpid(c->enc, nullptr, 0);
    if (c->src) fclose(c->src);
    delete c;
    cam->impl = nullptr;
}
