"""Camera, file source and x265 encoder helpers for the Pi transmitter (live_sender.py).

  Camera      newest I420 frame from the Pi camera (picamera2), no queue
  FileSource  a test video instead of the camera (--source): converted once, cached, looped
  Encoder     the frame_enc process: one I420 frame in, one HEVC access unit out
  link_au     strips VPS/SPS/PPS/AUD/SEI from an access unit (the receiver keeps the parameter sets)
"""
import hashlib
import os
import struct
import subprocess
import sys
import threading
import time

import numpy as np

ANNEX_B = b"\0\0\0\1"


# ---------------------------------------------------------------- HEVC helpers
def nal_units(au):
    """Split Annex-B into NAL payloads (without start codes)."""
    out, i, n = [], 0, len(au)
    starts = []
    while i < n - 2:
        if au[i] == 0 and au[i + 1] == 0 and au[i + 2] == 1:
            starts.append(i + 3)
            i += 3
        else:
            i += 1
    for k, s in enumerate(starts):
        e = starts[k + 1] - 3 if k + 1 < len(starts) else n
        nal = au[s:e]
        if k + 1 < len(starts):
            nal = nal.rstrip(b"\0")
        out.append(nal)
    return out


def link_au(au):
    """Drop VPS/SPS/PPS/AUD/SEI (receiver holds the parameter sets); 4-byte start codes."""
    keep = [n for n in nal_units(au) if n and ((n[0] >> 1) & 63) not in (32, 33, 34, 35, 39, 40)]
    return b"".join(ANNEX_B + n for n in keep)


# ---------------------------------------------------------------- camera
class Camera:
    """Keeps only the newest I420 frame (no queue), like the Luckfox shared camera."""

    def __init__(self, w, h, fps):
        from picamera2 import Picamera2
        self.w, self.h = w, h
        self.cam = Picamera2()
        from libcamera import controls as lc
        # Sensor mode: left to picamera2 it picks by output size only, and can land on a mode that cannot
        # reach the frame rate (ov5647: 1920x1080 tops out at 32.8 fps, so 40 fps was silently 32.8).
        # Take the widest field of view among the modes that are fast enough, then the most pixels.
        sensor = {}
        try:
            ok = [m for m in self.cam.sensor_modes if m.get("fps", 0) >= fps - 0.01]
            if ok:
                m = max(ok, key=lambda m: (m["crop_limits"][2] * m["crop_limits"][3], m["size"][0] * m["size"][1]))
                sensor = {"output_size": m["size"], "bit_depth": m["bit_depth"]}
                print("camera: sensor mode %dx%d (max %.1f fps) for %.1f fps" % (m["size"][0], m["size"][1], m["fps"], fps), flush=True)
            else:
                print("camera: no sensor mode reaches %.1f fps, using the default mode" % fps, flush=True)
        except Exception as e:  # camera without mode information: keep the default
            print("camera: sensor mode selection skipped (%s)" % e, flush=True)
        cfg = self.cam.create_video_configuration(main={"size": (w, h), "format": "YUV420"}, sensor=sensor,
                                                  controls={"FrameDurationLimits": (int(1e6 / fps), int(1e6 / fps)),
                                                            # Sensor noise costs bits at 47 kbit/s; denoise in the ISP.
                                                            "NoiseReductionMode": lc.draft.NoiseReductionModeEnum.HighQuality},
                                                  buffer_count=4)
        self.cam.configure(cfg)
        self.cam.start()
        self.latest = None
        self.stamp = 0.0
        self.sensor_t = 0.0
        self.frames = 0
        self.cond = threading.Condition()
        self.lock = self.cond
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        w, h = self.w, self.h
        while True:
            req = self.cam.capture_request()
            try:
                a = req.make_array("main")  # (h*3/2, stride) uint8
                ts = req.get_metadata().get("SensorTimestamp")  # ns, CLOCK_MONOTONIC (= time.monotonic())
            finally:
                req.release()
            now = time.monotonic()
            # sensor time of the frame (as the Luckfox's PTS); arrival time if the camera gives none / another clock
            sensor_t = ts / 1e9 if ts and 0 <= now - ts / 1e9 < 1.0 else now
            y = a[:h, :w]
            # Chroma planes follow luma at half stride; picamera2 returns the full buffer rows.
            stride = a.shape[1]
            flat = a.reshape(-1)
            u = flat[h * stride: h * stride + (h // 2) * (stride // 2)].reshape(h // 2, stride // 2)[:, :w // 2]
            v0 = h * stride + (h // 2) * (stride // 2)
            v = flat[v0: v0 + (h // 2) * (stride // 2)].reshape(h // 2, stride // 2)[:, :w // 2]
            frame = np.concatenate([y.reshape(-1), u.reshape(-1), v.reshape(-1)]).tobytes()
            with self.cond:
                self.latest, self.stamp, self.sensor_t = frame, now, sensor_t
                self.frames += 1
                self.cond.notify_all()

    def wait_new(self, seen, timeout=0.25):
        """Block until a camera frame newer than count `seen` is in; -> (frame, stamp, count)."""
        with self.cond:
            self.cond.wait_for(lambda: self.frames != seen, timeout)
            return self.latest, self.stamp, self.frames

    def grab(self):
        with self.lock:
            return self.latest, self.stamp


class FileSource:
    """Test video instead of the camera: any file ffmpeg can read (or a raw I420 .yuv of the right size).

    The file is converted once to w x h at fps (centre crop to fill the picture, no stretching) into a
    cached raw I420 clip in CACHE_DIR, which is then looped: every run sends exactly the same frames, and
    nothing is decoded while transmitting. The cache is keyed by the file's size, modification time,
    the picture size and the frame rate, so a changed file is converted again."""

    CACHE_DIR = os.path.join(os.path.expanduser("~"), ".cache", "lora-vlink")

    def __init__(self, path, w, h, fps=30.0):
        self.size = w * h * 3 // 2
        self.path = self.prepare(path, w, h, fps)
        self.n = os.path.getsize(self.path) // self.size
        if self.n < 1:
            raise RuntimeError("test video %s has no complete %dx%d frame" % (path, w, h))
        self.f = open(self.path, "rb")
        self.frames = 0
        self.loops = 0
        print("source: %s, %d frames (%.1f s at %g fps), looped" % (os.path.basename(path), self.n, self.n / fps, fps),
              flush=True)

    @classmethod
    def prepare(cls, path, w, h, fps):
        """-> path of a raw I420 clip of w x h at fps for `path`; converts (ffmpeg) when not cached yet."""
        if not os.path.isfile(path):
            raise RuntimeError("test video not found: %s" % path)
        if path.lower().endswith(".yuv"):
            if os.path.getsize(path) % (w * h * 3 // 2):
                raise RuntimeError("%s is not a whole number of %dx%d I420 frames" % (path, w, h))
            return path
        st = os.stat(path)
        key = hashlib.sha1(("%s|%d|%d|%dx%d|%.3f" % (os.path.abspath(path), st.st_size, int(st.st_mtime), w, h, fps))
                           .encode()).hexdigest()[:10]
        stem = os.path.splitext(os.path.basename(path))[0]
        out = os.path.join(cls.CACHE_DIR, "%s_%dx%d_%gfps_%s.yuv" % (stem, w, h, fps, key))
        if os.path.isfile(out):
            return out
        os.makedirs(cls.CACHE_DIR, exist_ok=True)
        part = out + ".part"
        vf = ("fps=%g,scale=%d:%d:force_original_aspect_ratio=increase:flags=area,crop=%d:%d,format=yuv420p"
              % (fps, w, h, w, h))
        print("source: converting %s to %dx%d at %g fps (once; cached in %s)" % (path, w, h, fps, cls.CACHE_DIR),
              flush=True)
        t0 = time.monotonic()
        try:
            subprocess.run(["ffmpeg", "-v", "error", "-nostdin", "-y", "-i", path, "-an", "-sn", "-vf", vf,
                            "-f", "rawvideo", part], check=True)
        except FileNotFoundError:
            raise RuntimeError("ffmpeg is needed for test videos: sudo apt install ffmpeg") from None
        except subprocess.CalledProcessError as e:
            raise RuntimeError("ffmpeg could not convert %s (exit %d)" % (path, e.returncode)) from None
        os.replace(part, out)
        print("source: converted in %.0f s -> %s" % (time.monotonic() - t0, out), flush=True)
        return out

    def grab(self):
        data = self.f.read(self.size)
        if len(data) < self.size:  # end of the clip: loop
            self.f.seek(0)
            self.loops += 1
            data = self.f.read(self.size)
        self.frames += 1
        return data, time.monotonic()


# ---------------------------------------------------------------- encoder
class Encoder:
    def __init__(self, exe, w, h, fps, preset, keyint, refresh, x265=""):
        self.p = subprocess.Popen([exe, str(w), str(h), "%.4f" % fps, preset, str(keyint), str(refresh), x265],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        hdr = self.p.stderr.read(7)
        if hdr[:3] != b"HDR":
            raise RuntimeError("encoder did not report headers: %r" % hdr)
        self.headers = self.p.stderr.read(struct.unpack("<I", hdr[3:])[0])
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        for line in self.p.stderr:
            sys.stderr.write("enc: " + line.decode(errors="replace"))

    def encode(self, frame, target, idr=False):
        self.p.stdin.write(b"FRM1" + struct.pack("<II", int(target), 1 if idr else 0) + frame)
        self.p.stdin.flush()
        h = self.p.stdout.read(20)
        if h[:4] != b"AU01":
            raise RuntimeError("bad encoder reply")
        n, qp, kind, pred = struct.unpack("<IiII", h[4:])
        return self.p.stdout.read(n), qp, kind, pred
