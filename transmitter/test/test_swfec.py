"""Checks the C transmitter side of the format (format/packet.h, scheduler.h) against the format's reference (format/packet.py):
CRC-16, RS(253,237) parity and the callsign blob byte for byte, that the records carry the video stream in order
(a packet is only padded when nothing else waits), 16-bit ids, and that damaged packets are repaired. Block FEC
(hz_sched_set_fec): every parity packet equals format/block_fec.py's parity over the video packets of its block,
interleaving and blocks closed early included.

    python3 transmitter/test/test_swfec.py transmitter/build/test_sched_dump
"""
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "format"))
import packet as S  # noqa: E402


class C:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def cmd(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()


def fail(msg):
    print("test_swfec: FAIL", msg)
    sys.exit(1)


def check_record(r):
    rec = bytes.fromhex(r)
    if len(rec) != 2 + S.ENTRY + 1 or rec[0] != 0xA5 or rec[1] != 0x5A:
        fail("record framing")
    x = 0
    for b in rec[2:-1]:
        x ^= b
    if x != rec[-1]:
        fail("record xor")
    return rec[2:-1]


def main():
    c = C(sys.argv[1])
    rng = random.Random(1)
    # 1. CRC-16 and RS parity, C vs Python
    for _ in range(200):
        b = bytes(rng.randrange(256) for _ in range(rng.choice([0, 1, 50, S.RS_DATA])))
        if int(c.cmd("C " + b.hex()), 16) != S.crc16(b):
            fail("crc16")
        if b and bytes.fromhex(c.cmd("R " + b.hex())) != S.rs_parity(b):
            fail("rs parity")
    # 2. callsign blob, C vs Python
    for t in ("S57ABC", "S57ABC/P", "W1AW-12 XY", "a", "lower case", "X" * 32, "Č" * 20):
        want = S.encode_call(t).hex()
        if c.cmd("K " + t) != want:
            fail("callsign blob %r" % t)
        if S.decode_call(bytes.fromhex(want)) != t:
            fail("callsign round trip %r" % t)
    # 3. record stream: pushed in random chunks, drained in between; each record = the next min(231, waiting) bytes
    assert c.cmd("S") == "OK"
    stream = bytes(rng.randrange(256) for _ in range(S.SYM * 3000 + 77))
    mirror, ents, i, pads, nid = bytearray(), [], 0, 0, 0
    while i < len(stream):
        n = rng.randrange(1, 700)
        assert c.cmd("P " + stream[i:i + n].hex()) == "OK"
        mirror += stream[i:i + n]
        i += n
        for _ in range(rng.randrange(0, 5)):
            r = c.cmd("N")
            if r == "-":
                if mirror:
                    fail("no record although bytes wait")
                break
            e = check_record(r)
            take = min(S.SYM, len(mirror))
            if not S.check_entry(e) or e[0] != 0 or e[1] | e[2] << 8 != nid & 0xFFFF:
                fail("entry header / crc")
            if e[S.HDR:S.HDR + S.SYM] != bytes(mirror[:take]) + bytes(S.SYM - take):
                fail("symbol is not the next stream bytes")
            pads += take < S.SYM
            del mirror[:take]
            nid += 1
            ents.append(e)
    print("test_swfec: %d records, stream in order, %d padded (queue ran dry)" % (len(ents), pads))
    # 4. 16-bit id wrap
    assert c.cmd("S") == "OK" and c.cmd("I 65530") == "OK"
    assert c.cmd("P " + bytes(S.SYM * 12).hex()) == "OK"
    ids = [(lambda e: e[1] | e[2] << 8)(check_record(c.cmd("N"))) for _ in range(12)]
    if ids != [(65530 + i) & 0xFFFF for i in range(12)]:
        fail("id wrap %s" % ids)
    print("test_swfec: id wrap ok")
    # 5. damaged packets from the C encoder are repaired (8 random byte errors, or 16 known erasures)
    ok = 0
    for e in ents[:200]:
        b = bytearray(e)
        for p in rng.sample(range(S.ENTRY), 8):
            b[p] ^= rng.randrange(1, 256)
        r = S.repair_entry(b)
        ok += r is not None and r[:S.RS_DATA] == e[:S.RS_DATA]
        b = bytearray(e)
        pos = rng.sample(range(S.ENTRY), 16)
        for p in pos:
            b[p] ^= rng.randrange(1, 256)
        r = S.repair_entry(b, pos)
        ok += r is not None and r[:S.RS_DATA] == e[:S.RS_DATA]
    if ok != 400:
        fail("repair of damaged C packets %d/400" % ok)
    # 6. block FEC: the parity against the reference (format/block_fec.py)
    check_block_fec(c, rng)
    print("test_swfec: OK")


def check_block_fec(c, rng):
    import block_fec
    if c.cmd("F 0 1 1") != "BAD" or c.cmd("F 17 1 1") != "BAD" or c.cmd("F 4 2 9") != "BAD":
        fail("block FEC accepts bad k, m, d")
    for fec in ((16, 2, 1), (12, 3, 1), (8, 2, 3), (1, 1, 1), (16, 16, 8), (5, 4, 2)):
        assert c.cmd("S") == "OK" and c.cmd("F %d %d %d" % fec) == "OK"
        video, parity = {}, []   # id16 -> symbol; (entry) of every parity packet
        n = early = 0
        for _ in range(1500):
            if rng.random() < 0.4:
                assert c.cmd("P " + bytes(rng.randrange(256) for _ in range(rng.randrange(1, 900))).hex()) == "OK"
            for _ in range(rng.choice((1, 1, 2, 3, 20))):   # 20: idle slots, a group may close early
                r = c.cmd("N")
                if r != "-":
                    e = check_record(r)
                    if not S.check_entry(e):
                        fail("block FEC %s: entry crc" % (fec,))
                    if e[0] == 0x00:  # video
                        video[e[1] | e[2] << 8] = bytes(e[S.HDR:S.HDR + S.SYM])
                    elif e[0] & 0x0F == S.PARITY:
                        parity.append(e)
                n += 1
        for e in parity:   # its block: k video packets from id16 on, d apart (interleaving); parity index j
            if e[0] >> 4 != fec[2] - 1:
                fail("block FEC %s: parity type" % (fec,))
            first, k, j = e[1] | e[2] << 8, (e[3] >> 4) + 1, e[3] & 15
            early += k < fec[0]
            syms = [video.get((first + t * fec[2]) & 0xFFFF) for t in range(k)]
            if None in syms:
                fail("block FEC %s: parity for packets never sent" % (fec,))
            if bytes(e[S.HDR:S.HDR + S.SYM]) != bytes(block_fec.encode(syms, fec[1])[j]):
                fail("block FEC %s: parity differs from format/block_fec.py" % (fec,))
        if not parity or (fec[0] > 1 and not early):
            fail("block FEC %s: parity %d, early-closed %d" % (fec, len(parity), early))
        print("test_swfec: block FEC %d+%d x%d: %d slots, %d parity packets (%d for blocks closed early), as "
              "format/block_fec.py" % (fec + (n, len(parity), early)))


if __name__ == "__main__":
    main()
