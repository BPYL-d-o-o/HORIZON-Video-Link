"""Checks the Pi transmitter's format v4 scheduler (swtx.py):
  1. against the Luckfox's C scheduler (luckfox_vtx/hz_sched.h), record for record, byte for byte, with frames pushed
     and records drained in a random interleaving (so padding happens too), and the callsign blob / host records. The C
     test driver is compiled into a temporary directory (needs gcc; the Luckfox files are only read). Skipped with a
     note if gcc or luckfox_vtx/ is missing.
  2. the records carry the pushed stream unchanged (padding only where the queue ran empty), every entry passes the
     receiver's checks.
  3. speed of record building on this machine.

    python3 pi5_vtx/test_swtx.py
"""
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swtx  # noqa: E402
from swtx import S  # noqa: E402


def fail(msg):
    print("test_swtx: FAIL", msg)
    sys.exit(1)


def frames(rng, n):
    return [bytes(rng.randrange(256) for _ in range(rng.choice([20, 60, 230, 231, 232, 300, 462, 500, 900])))
            for _ in range(n)]


def entry_of(rec):
    if len(rec) != swtx.REC_LEN or rec[:2] != b"\xa5\x5a":
        fail("record framing")
    x = 0
    for b in rec[2:-1]:
        x ^= b
    if x != rec[-1]:
        fail("record xor")
    return rec[2:-1]


def against_c():
    root = os.path.join(HERE, "..", "luckfox_vtx")
    src = os.path.join(root, "test_sched_dump.c")
    if not (shutil.which("gcc") and os.path.exists(src)):
        print("test_swtx: C comparison skipped (needs gcc and luckfox_vtx/)")
        return
    tmp = tempfile.mkdtemp()
    exe = os.path.join(tmp, "test_sched_dump")
    subprocess.run(["gcc", "-O2", "-I", root, "-o", exe, src, "-lpthread", "-lm"], check=True)
    p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def c(line):
        p.stdin.write(line + "\n")
        p.stdin.flush()
        return p.stdout.readline().strip()

    rng = random.Random(7)
    assert c("S") == "OK"
    py = swtx.Scheduler()
    n = 0
    for f in frames(rng, 400):
        assert c("P " + f.hex()) == "OK"
        py.push(f)
        for _ in range(rng.randrange(0, 4)):
            a, b = c("N"), py.next_record()
            if (a == "-") != (b is None) or (b is not None and bytes.fromhex(a) != b):
                fail("record %d differs from the C scheduler" % n)
            n += 1
    for t in ("S57ABC", "S57ABC/P", "any text", "Č"):
        if c("K " + t) != S.encode_call(t).hex():
            fail("callsign blob %r" % t)
    p.stdin.close()
    p.wait()
    shutil.rmtree(tmp, ignore_errors=True)
    print("test_swtx: %d records identical to the C scheduler, callsign blobs identical" % n)


def stream_check():
    rng = random.Random(3)
    sch = swtx.Scheduler()
    sent, pushed = bytearray(), bytearray()
    pads = 0
    for f in frames(rng, 600):
        sch.push(f)
        pushed += f
        for _ in range(rng.randrange(0, 4)):
            r = sch.next_record()
            if r is None:
                if sch.backlog():
                    fail("no record although bytes wait")
                break
            e = entry_of(r)
            if not S.check_entry(e) or e[0] != 0:
                fail("entry check")
            sym = e[S.HDR:S.HDR + S.SYM]
            take = min(S.SYM, len(pushed) - len(sent))
            if sym != bytes(pushed[len(sent):len(sent) + take]) + bytes(S.SYM - take):
                fail("stream bytes")
            pads += take < S.SYM
            sent += sym[:take]
    rec = swtx.call_record(S.encode_call("S57ABC"))
    if rec != bytes.fromhex("a51d0586e27b9306") + bytes([0x5A ^ 5 ^ 0x86 ^ 0xE2 ^ 0x7B ^ 0x93 ^ 0x06]):
        fail("callsign record")
    if swtx.ident_record(2) != b"\xa5\x1e\x02\xfd":
        fail("ident record")
    print("test_swtx: stream carried unchanged (%d bytes, %d packets padded), host records ok" % (len(sent), pads))


def speed():
    sch = swtx.Scheduler()
    sch.push(bytes(range(256)) * 400)
    t = time.perf_counter()
    for _ in range(300):
        sch.next_record()
    print("test_swtx: record build %.2f ms" % ((time.perf_counter() - t) / 300 * 1000))


if __name__ == "__main__":
    against_c()
    stream_check()
    speed()
    print("test_swtx: OK")
