"""Optional block FEC across packets (systematic Cauchy Reed-Solomon over GF(256), polynomial 0x11d).

The Pi transmitter's (pi5_vtx/swtx.py); the Luckfox's luckfox_vtx/hz_fec.h and the receivers' ports give the same parity.

After every k video packets (a block) the transmitter sends m parity packets; any k of the k+m packets of a block give
back all k. Video packets are sent unchanged (systematic), so the receiver only waits for parity after a packet of the
block was lost. Parity packet (swfec entry): type PARITY (0x02), id16 = the block's first video packet id,
aux = (k - 1) << 4 | j (j = parity index; k, m <= 16), symbol = parity j over the k video symbols. A block that is still
open when the transmitter runs out of video is closed early with k < the configured block size, so a recovery never
waits for idle time.
"""
import numpy as np

# GF(256), polynomial 0x11d
EXP = np.zeros(512, dtype=np.uint8)
LOG = np.zeros(256, dtype=np.int32)
_x = 1
for _i in range(255):
    EXP[_i] = _x
    LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
EXP[255:510] = EXP[:255]


def gmul(a, b):
    if a == 0 or b == 0:
        return 0
    return int(EXP[LOG[a] + LOG[b]])


def ginv(a):
    return int(EXP[255 - LOG[a]])


def gmul_vec(c, v):
    """c (scalar) * v (uint8 array)."""
    if c == 0:
        return np.zeros_like(v)
    out = EXP[(LOG[v] + LOG[c]) % 255]
    out[v == 0] = 0
    return out


def cauchy(k, m):
    """m x k Cauchy matrix: C[j][i] = 1 / (x_j + y_i), x_j = k + j, y_i = i (all distinct)."""
    return [[ginv((k + j) ^ i) for i in range(k)] for j in range(m)]


def encode(symbols, m):
    """symbols: list of k equal-length bytes -> list of m parity bytes."""
    k = len(symbols)
    C = cauchy(k, m)
    src = [np.frombuffer(s, np.uint8) for s in symbols]
    out = []
    for j in range(m):
        acc = np.zeros_like(src[0])
        for i in range(k):
            acc ^= gmul_vec(C[j][i], src[i])
        out.append(acc.tobytes())
    return out


def decode(k, m, have, parity):
    """have: {i: bytes} received sources (0..k-1); parity: {j: bytes} received parities (0..m-1).
    -> {i: bytes} for every missing source, or None when too much is missing."""
    miss = [i for i in range(k) if i not in have]
    if not miss:
        return {}
    rows = sorted(parity)[:len(miss)]
    if len(rows) < len(miss):
        return None
    C = cauchy(k, m)
    n = len(miss)
    L = len(next(iter(parity.values())))
    # right-hand side: parity_j minus the known sources' contribution
    rhs = []
    for j in rows:
        acc = np.frombuffer(parity[j], np.uint8).copy()
        for i, s in have.items():
            acc ^= gmul_vec(C[j][i], np.frombuffer(s, np.uint8))
        rhs.append(acc)
    A = [[C[j][i] for i in miss] for j in rows]
    # Gauss-Jordan over GF(256)
    for col in range(n):
        piv = next(r for r in range(col, n) if A[r][col])
        A[col], A[piv] = A[piv], A[col]
        rhs[col], rhs[piv] = rhs[piv], rhs[col]
        inv = ginv(A[col][col])
        A[col] = [gmul(inv, a) for a in A[col]]
        rhs[col] = gmul_vec(inv, rhs[col])
        for r in range(n):
            if r != col and A[r][col]:
                f = A[r][col]
                A[r] = [a ^ gmul(f, b) for a, b in zip(A[r], A[col])]
                rhs[r] = rhs[r] ^ gmul_vec(f, rhs[col])
    return {miss[r]: rhs[r].tobytes() for r in range(n)}


