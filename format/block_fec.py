"""Block FEC across packets: the reference (format/README.md "Block FEC across packets").

format/block_fec.h and the receivers' ports must give the same parity.
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
