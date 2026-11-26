"""Receiver capture made by the Luckfox's own packet scheduler (scheduler.h, through test_sched_dump), with or without
block FEC, and with packets lost on the way: for checking the receivers offline (test/block_fec.sh).

The video byte stream comes from a real capture (its video packets in order, from the first access unit boundary on).
The transmitter is simulated slot by slot: one record per slot as the feeder takes them (a frame is pushed every
--frame-slots slots while at most --max-backlog bytes wait, as hz_vtx); a slot without a record carries the RP2350's
filler. Every slot then becomes a receiver line (radio_firmware/src/rx.c): "P ..." for a packet, "E ..." for one that
failed the radio CRC (a few damaged bytes, repaired by the Reed-Solomon code), "M ..." for a lost one.

    python transmitter/test/fec_capture.py TEST_SCHED_DUMP SOURCE.rxlog OUT.rxlog [--fec K,M[,D]] [--loss P] [--burst-rate P]
        [--burst N] [--damage P] [--ref REF.rxlog] [--source-lines N] [--seed S]

--ref writes the same packets with nothing lost or damaged. Prints EVENT lines and RESULT PASS|FAIL.
"""
import argparse
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "format"))
sys.path.insert(0, HERE)
import packet as S  # noqa: E402
from test_swfec import C  # noqa: E402

AUD = b"\x00\x00\x00\x01\x46\x01\x50"
FILLER_ENTRY = bytes([0xFF, 0xFF, 0xFF]) + bytes(S.ENTRY - 3)


def source_stream(path, max_lines):
    """The video byte stream of a capture: its video packets' symbols in id order (a lost one is skipped), cut to begin
    right after the first AUD (an access unit boundary) and to end with the last AUD. Its packets are checked with the
    link ID of the receiver that recorded it (its "C cfg ... link=N" line; the packets built here are unkeyed)."""
    syms, last = [], None
    with open(path, "rb") as f:
        for n, raw in enumerate(f):
            if n >= max_lines:
                break
            p = raw.decode("ascii", "replace").rstrip("\r\n").split(" ", 1)[-1].split(" ")
            if p[:2] == ["C", "cfg"]:
                S.LINK = next((int(w[5:]) for w in p if w.startswith("link=") and w[5:].isdigit()), S.LINK)
            if len(p) != 11 or p[0] not in ("P", "E"):
                continue
            try:
                e = bytes.fromhex(p[10])[2:]
            except ValueError:
                continue
            if len(e) != S.ENTRY or (p[0] == "P" and S.is_filler(e)):
                continue
            e = e if p[0] == "P" and S.check_entry(e) else S.repair_entry(e)
            if e is None or e[0] != 0:
                continue
            i = e[1] | e[2] << 8
            if i == last:
                continue
            last = i
            syms.append(bytes(e[S.HDR:S.HDR + S.SYM]))
    S.LINK = 0
    b = b"".join(syms)
    a, z = b.find(AUD), b.rfind(AUD)
    if a < 0 or z <= a:
        return b""
    return b[a + len(AUD):z + len(AUD)]


def frames(stream):
    out, i = [], 0
    while i < len(stream):
        j = stream.find(AUD, i)
        j = len(stream) if j < 0 else j + len(AUD)
        out.append(stream[i:j])
        i = j
    return out


def line(kind, seq, payload):
    return "%s %d 0 T -160 -158 40 120 0 -190 %s" % (kind, seq, payload.hex())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("exe")
    ap.add_argument("source")
    ap.add_argument("out")
    ap.add_argument("--fec", default="")
    ap.add_argument("--loss", type=float, default=0.0, help="random packet loss (fraction)")
    ap.add_argument("--burst-rate", type=float, default=0.0, help="chance per slot that a fade starts")
    ap.add_argument("--burst", type=int, default=4, help="packets lost in a row in a fade")
    ap.add_argument("--damage", type=float, default=0.0, help="packets that fail the radio CRC with 1..8 bad bytes")
    ap.add_argument("--ref", default=None)
    ap.add_argument("--source-lines", type=int, default=60000)
    ap.add_argument("--frame-slots", type=float, default=1.9, help="slots per camera frame (30 fps, 17.54 ms slots)")
    ap.add_argument("--max-backlog", type=int, default=4 * S.SYM)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    stream = source_stream(a.source, a.source_lines)
    if not stream:
        print("FATAL no access units in %s" % a.source)
        print("RESULT FAIL")
        return 1
    fr = frames(stream)
    c = C(a.exe)
    if c.cmd("S") != "OK":
        print("FATAL test_sched_dump")
        print("RESULT FAIL")
        return 1
    if a.fec:
        k = [int(x) for x in a.fec.split(",")] + [1]
        if c.cmd("F %d %d %d" % tuple(k[:3])) != "OK":
            print("FATAL bad --fec %s" % a.fec)
            print("RESULT FAIL")
            return 1
    rng = random.Random(a.seed)
    out = open(a.out, "w", newline="\n")
    ref = open(a.ref, "w", newline="\n") if a.ref else None
    t, seq, fi, pushed, backlog, next_frame, fade = 1795300000.0, 0, 0, 0, 0, 0.0, 0
    n = {"video": 0, "parity": 0, "filler": 0, "lost": 0, "damaged": 0}
    idle_after = 0
    while idle_after < 64:
        if fi < len(fr) and seq >= next_frame:
            next_frame += a.frame_slots
            if backlog <= a.max_backlog:
                assert c.cmd("P " + fr[fi].hex()) == "OK"
                backlog += len(fr[fi])
                pushed += 1
            fi += 1
        r = c.cmd("N")
        if r == "-":
            e = FILLER_ENTRY
            n["filler"] += 1
            idle_after = idle_after + 1 if fi >= len(fr) else 0
        else:
            e = bytes.fromhex(r)[2:-1]
            if e[0] == 0:
                n["video"] += 1
                backlog = max(0, backlog - S.SYM)
            else:
                n["parity"] += 1
        payload = bytes([seq & 0xFF, (seq >> 8) & 0xFF]) + e
        stamp = "%.4f " % t
        if ref:
            ref.write(stamp + line("P", seq, payload) + "\n")
        if fade == 0 and rng.random() < a.burst_rate:
            fade = a.burst
        lost = fade > 0 or rng.random() < a.loss
        fade = max(0, fade - 1)
        if lost:
            out.write(stamp + "M %d 0 T -200\n" % seq)
            n["lost"] += 1
        elif rng.random() < a.damage:
            d = bytearray(payload)
            for p in rng.sample(range(2, len(d)), rng.randrange(1, 9)):
                d[p] ^= rng.randrange(1, 256)
            out.write(stamp + line("E", seq, bytes(d)) + "\n")
            n["damaged"] += 1
        else:
            out.write(stamp + line("P", seq, payload) + "\n")
        seq += 1
        t += 0.01754
    out.close()
    if ref:
        ref.close()
    print("EVENT capture %s: %d slots, %d of %d frames pushed (the rest skipped: backlog), video %d, parity %d, "
          "filler %d, lost %d, damaged %d, fec %s"
          % (a.out, seq, pushed, len(fr), n["video"], n["parity"], n["filler"], n["lost"], n["damaged"], a.fec or "off"))
    print("RESULT PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
