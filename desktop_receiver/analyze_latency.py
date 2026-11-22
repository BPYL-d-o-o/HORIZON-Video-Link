"""Pairs the app's frame log (--frame-log: QPC times of each picture's last packet, decode and Present call) with
PresentMon's record of the same Present (measure_latency.cmd), and reports how the pictures reached the screen and
how long that took. Only PresentMon's display times are used: DXGI's own statistics are wrong in tearing mode.

    python analyze_latency.py [DIR]          (default %USERPROFILE%\\horizon-build\\latency)

"Display" is when Windows hands the frame to the screen (PresentMon), not when the panel shows it (scan-out and
pixel response are not included).
"""
import bisect
import csv
import os
import statistics
import sys
from collections import Counter


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p / 100 * len(v)))]


def analyze(d, name):
    frames_path, pm_path = os.path.join(d, f"frames_{name}.csv"), os.path.join(d, f"pm_{name}.csv")
    if not (os.path.exists(frames_path) and os.path.exists(pm_path)):
        return
    freq = 10_000_000
    frames = []
    for line in open(frames_path):
        if line.startswith("# qpc_frequency="):
            freq = int(line.split("=")[1])
        elif line[:1].isdigit():
            a, dec, p, e = (int(x) for x in line.split(",")[:4])
            frames.append((a, dec, p, e))
    pm = list(csv.DictReader(open(pm_path, encoding="utf-8-sig")))
    presents = sorted((int(r["QPCTime"]), r) for r in pm)
    keys = [q for q, _ in presents]
    ms = 1000.0 / freq
    packet, decoded, present, modes, dropped = [], [], [], Counter(), 0
    for a, dec, p, e in frames:
        i = bisect.bisect_left(keys, p)
        if i >= len(keys) or keys[i] > e:
            continue  # PresentMon was not recording then
        r = presents[i][1]
        if r["Dropped"] == "1" or r["msUntilDisplayed"] in ("", "NA"):
            dropped += 1
            continue
        shown = keys[i] + float(r["msUntilDisplayed"]) / ms
        packet.append((shown - a) * ms)
        decoded.append((shown - dec) * ms)
        present.append(float(r["msUntilDisplayed"]))
        modes[r["PresentMode"]] += 1
    print(f"== {name}: {len(packet)} pictures paired with PresentMon, {dropped} dropped")
    for m, n in modes.most_common():
        print(f"   {n:5d} x {m}")
    for label, v in (("last packet -> display", packet), ("decoded -> display", decoded), ("Present -> display", present)):
        if v:
            print(f"   {label:24s} median {statistics.median(v):6.2f}  p95 {pct(v, 95):6.2f}  p99 {pct(v, 99):6.2f}"
                  f"  max {max(v):6.2f} ms")


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.expanduser("~"), "horizon-build", "latency")
    names = sorted({f[len("frames_"):-4] for f in os.listdir(d) if f.startswith("frames_") and f.endswith(".csv")})
    for n in names:
        analyze(d, n)
