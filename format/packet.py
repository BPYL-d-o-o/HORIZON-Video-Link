"""HORIZON over-the-air format (pure Python + numpy): the reference the transmitters and receivers are checked against.
Layout: format/README.md. Bit-identical counterpart of format/packet.h (transmitter/test/test_swfec.py).

  * check_entry(): CRC-16 must match, also for packets the radio accepted (its CRC lets bad ones through now and then).
  * repair_entry(): a packet that FAILED the radio CRC, corrected by the in-packet RS code.
  * combine(): damaged copies of the same packet from several receivers; bytes where they disagree are erasures.
  * encode_call() / decode_call(): the callsign blob.
"""
import struct

import numpy as np

PAYLOAD = 255
ENTRY = 253
HDR = 4
RS_PAR = 16
SYM = ENTRY - HDR - 2 - RS_PAR    # 231
CRC_AT = HDR + SYM                # 235
RS_DATA = CRC_AT + 2              # 237
FILLER = 0xFF
PARITY = 0x02                     # optional block FEC parity packet (format/block_fec.py)

# ------------------------------------------------------------------ GF(256), polynomial 0x11d
_EXP = [0] * 512
_LOG = [0] * 256
_x = 1
for _i in range(255):
    _EXP[_i] = _x
    _LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
for _i in range(255, 512):
    _EXP[_i] = _EXP[_i - 255]

# MUL[c] = table "multiply by c" for numpy fancy indexing: MUL[c][vec]
MUL = np.zeros((256, 256), dtype=np.uint8)
for _c in range(1, 256):
    for _v in range(1, 256):
        MUL[_c, _v] = _EXP[_LOG[_c] + _LOG[_v]]


def gmul(a, b):
    return 0 if a == 0 or b == 0 else _EXP[_LOG[a] + _LOG[b]]


def ginv(a):
    return _EXP[255 - _LOG[a]]


def gpow(x, p):
    return 1 if p == 0 else (0 if x == 0 else _EXP[(_LOG[x] * p) % 255])


# ------------------------------------------------------------------ CRC-16/MODBUS (poly 0x8005 reflected, init 0xFFFF)
_CRC16 = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (0xA001 ^ (_c >> 1)) if _c & 1 else _c >> 1
    _CRC16.append(_c)


def crc16(data):
    c = 0xFFFF
    for b in data:
        c = _CRC16[(c ^ b) & 0xFF] ^ (c >> 8)
    return c


# The link ID packets are checked with (0 = unkeyed, as raw TX taps). A receiver sets it from "C cfg ... link=N".
LINK = 0


def crc_ok(e, link=None):
    link = LINK if link is None else link
    return crc16(bytes(e[:CRC_AT])) ^ link == struct.unpack_from("<H", bytes(e), CRC_AT)[0]


# ------------------------------------------------------------------ Reed-Solomon RS(253,237), errors + erasures
def _poly_mul(p, q):
    r = [0] * (len(p) + len(q) - 1)
    for j, qj in enumerate(q):
        if qj:
            for i, pi in enumerate(p):
                if pi:
                    r[i + j] ^= _EXP[_LOG[pi] + _LOG[qj]]
    return r


def _poly_eval(p, x):
    y = p[0]
    for c in p[1:]:
        y = gmul(y, x) ^ c
    return y


def _poly_scale(p, x):
    return [gmul(c, x) for c in p]


def _poly_add(p, q):
    r = [0] * max(len(p), len(q))
    for i, c in enumerate(p):
        r[i + len(r) - len(p)] = c
    for i, c in enumerate(q):
        r[i + len(r) - len(q)] ^= c
    return r


_GEN = [1]
for _i in range(RS_PAR):
    _GEN = _poly_mul(_GEN, [1, _EXP[_i]])


def rs_parity(msg):
    """RS parity (16 bytes) of msg (bytes-like), = hz_rs_encode()."""
    r = [0] * RS_PAR
    g = _GEN[1:]
    for b in msg:
        f = b ^ r[0]
        r = r[1:] + [0]
        if f:
            lf = _LOG[f]
            for j in range(RS_PAR):
                if g[j]:
                    r[j] ^= _EXP[_LOG[g[j]] + lf]
    return bytes(r)


def _syndromes(cw):
    # S_i = c(alpha^i), i = 0..15, with a leading 0 (the classic layout of the algorithm below)
    return [0] + [_poly_eval(cw, _EXP[i]) for i in range(RS_PAR)]


def _forney_syndromes(synd, pos, n):
    fs = list(synd[1:])
    for p in pos:
        x = gpow(2, n - 1 - p)
        for j in range(len(fs) - 1):
            fs[j] = gmul(fs[j], x) ^ fs[j + 1]
    return fs


def _error_locator(synd, nsym, erase_count):
    err_loc, old_loc = [1], [1]
    shift = len(synd) - nsym if len(synd) > nsym else 0
    for i in range(nsym - erase_count):
        k = i + shift
        delta = synd[k]
        for j in range(1, len(err_loc)):
            delta ^= gmul(err_loc[-(j + 1)], synd[k - j])
        old_loc = old_loc + [0]
        if delta:
            if len(old_loc) > len(err_loc):
                new_loc = _poly_scale(old_loc, delta)
                old_loc = _poly_scale(err_loc, ginv(delta))
                err_loc = new_loc
            err_loc = _poly_add(err_loc, _poly_scale(old_loc, delta))
    while err_loc and err_loc[0] == 0:
        del err_loc[0]
    if (len(err_loc) - 1 - erase_count) * 2 + erase_count > nsym:
        return None
    return err_loc


def _find_errors(err_loc, n):
    errs = len(err_loc) - 1
    pos = [n - 1 - i for i in range(n) if _poly_eval(err_loc, gpow(2, i)) == 0]
    return pos if len(pos) == errs else None


def _correct_errata(cw, synd, err_pos):
    n = len(cw)
    coef_pos = [n - 1 - p for p in err_pos]
    loc = [1]
    for p in coef_pos:
        loc = _poly_mul(loc, _poly_add([1], [gpow(2, p), 0]))
    # error evaluator: (synd reversed * loc) mod x^(len(loc))
    s_rev = synd[::-1]
    prod = _poly_mul(s_rev, loc)
    ev = prod[len(prod) - len(loc):]
    ev = ev[::-1]
    xs = [_EXP[p % 255] for p in coef_pos]  # X_i = alpha^(position as power)
    out = list(cw)
    for i, xi in enumerate(xs):
        xi_inv = ginv(xi)
        prime = 1
        for j, xj in enumerate(xs):
            if j != i:
                prime = gmul(prime, 1 ^ gmul(xi_inv, xj))
        if prime == 0:
            return None
        y = _poly_eval(ev[::-1], xi_inv)
        y = gmul(xi, y)  # X^(1 - fcr), fcr = 0
        out[err_pos[i]] ^= gmul(y, ginv(prime))
    return out


def rs_decode(cw, erasures=()):
    """cw: 253 ints. Returns the corrected codeword (list) or None if uncorrectable."""
    cw = list(cw)
    erasures = sorted(set(erasures))
    if len(erasures) > RS_PAR:
        return None
    for p in erasures:
        cw[p] = 0
    synd = _syndromes(cw)
    if max(synd) == 0:
        return cw
    fs = _forney_syndromes(synd, erasures, len(cw))
    loc = _error_locator(fs, RS_PAR, len(erasures))
    if loc is None:
        return None
    errs = _find_errors(loc[::-1], len(cw))
    if errs is None:
        return None
    out = _correct_errata(cw, synd, erasures + errs)
    if out is None or max(_syndromes(out)) != 0:
        return None
    return out


# ------------------------------------------------------------------ packet entry checks
def is_filler(e):
    """The TX RP2350's own packets: idle filler (FF FF FF) and command ACK / power announcement (FF FF FE). They carry
    no CRC-16 of their own (the radio CRC covers them)."""
    return e[0] == FILLER and e[1] == 0xFF and e[2] in (0xFF, 0xFE)


def check_entry(e):
    """Entry (253 bytes) that passed the radio CRC: True if its CRC-16 matches."""
    return len(e) == ENTRY and crc_ok(e)


def repair_entry(e, erasures=()):
    """Entry from a packet that FAILED the radio CRC: corrected bytes (bytes) or None.
    Accepted only if both the RS code decodes and the CRC-16 matches."""
    if len(e) != ENTRY:
        return None
    if not erasures and crc_ok(e):
        return bytes(e)  # the radio CRC failed only in the 2-byte seq / its own field
    out = rs_decode(e, erasures)
    if out is None:
        return None
    b = bytes(out)
    return b if crc_ok(b) else None


def combine(copies):
    """Several damaged copies of the same entry (different receivers): first each one alone, then the bytes where they
    disagree as erasures (if <= 16), applied to each copy. Returns corrected bytes or None."""
    for c in copies:
        r = repair_entry(c)
        if r is not None:
            return r
    if len(copies) < 2:
        return None
    a = np.frombuffer(bytes(copies[0]), dtype=np.uint8)
    diff = np.zeros(ENTRY, dtype=bool)
    for c in copies[1:]:
        diff |= a != np.frombuffer(bytes(c), dtype=np.uint8)
    pos = [int(i) for i in np.nonzero(diff)[0]]
    if not pos or len(pos) > RS_PAR:
        return None
    for c in copies:
        r = repair_entry(c, pos)
        if r is not None:
            return r
    return None


# ------------------------------------------------------------------ identification (callsign) blob = hz_call_encode()
CALL_ALPHA = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/- "   # codes 1..39 (0 = padding)
CALL_MAX = 112          # blob bytes (16 uplink parts of 7)
CALL_TEXT_MAX = 32      # characters a callsign may have


def encode_call(text):
    """Callsign / any text -> blob (bytes). Packed (16 bits per 3 characters) if every character is in CALL_ALPHA,
    else raw UTF-8. Raises ValueError if empty or too long."""
    if not text:
        raise ValueError("empty callsign")
    if all(c in CALL_ALPHA for c in text) and len(text) <= 127:
        out = bytearray([0x80 | len(text)])
        codes = [CALL_ALPHA.index(c) + 1 for c in text]
        codes += [0] * (-len(codes) % 3)
        for i in range(0, len(codes), 3):
            out += struct.pack("<H", codes[i] * 1600 + codes[i + 1] * 40 + codes[i + 2])
    else:
        raw = text.encode("utf-8")
        if len(raw) > min(127, CALL_MAX - 1):
            raise ValueError("callsign too long (%d bytes, max %d)" % (len(raw), min(127, CALL_MAX - 1)))
        out = bytearray([len(raw)]) + raw
    if len(out) > CALL_MAX:
        raise ValueError("callsign too long")
    return bytes(out)


def call_blob_len(blob):
    """Bytes the blob uses according to its header (0 if inconsistent)."""
    if not blob:
        return 0
    n = blob[0] & 0x7F
    ln = 1 + 2 * ((n + 2) // 3) if blob[0] & 0x80 else 1 + n
    return ln if ln <= len(blob) else 0


def decode_call(blob):
    """Blob -> text (None if malformed)."""
    ln = call_blob_len(blob)
    if not ln:
        return None
    n = blob[0] & 0x7F
    if not blob[0] & 0x80:
        return blob[1:1 + n].decode("utf-8", errors="replace")
    out = []
    for i in range(1, ln, 2):
        w = blob[i] | blob[i + 1] << 8
        for c in (w // 1600, (w // 40) % 40, w % 40):
            if c:
                out.append(CALL_ALPHA[c - 1] if c <= len(CALL_ALPHA) else "?")
    return "".join(out[:n])


class IdentAssembler:
    """Ident packets ("I seq part parts hex20" lines from the receiver RP2350) -> callsign text once all parts of one
    identification are in (parts are sent in consecutive command slots)."""

    def __init__(self):
        self.parts = {}
        self.n = None
        self.last_seq = None

    def add(self, seq, part, parts, data):
        if self.n != parts or (self.last_seq is not None and (seq - self.last_seq) & 0xFFFF > 64 * (parts + 1)) or part == 0:
            self.parts = {}
        self.n, self.last_seq = parts, seq
        self.parts[part] = bytes(data)
        if len(self.parts) == parts:
            blob = b"".join(self.parts[i] for i in range(parts))
            self.parts = {}
            return decode_call(blob)
        return None
