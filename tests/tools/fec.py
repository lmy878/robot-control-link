"""Systematic MDS erasure FEC over GF(2^8).

Models the block FEC that WFB-ng applies below the RCLP transport: a block of
``k`` data packets is expanded to ``n = k + m`` packets (``k`` systematic data +
``m`` parity). Any ``k`` of the ``n`` packets let the receiver rebuild the ``k``
data packets (MDS / maximum-distance-separable).

Parity is generated with a Cauchy matrix over GF(256), which is provably MDS.
This is an *erasure* decoder: lost packets are known from the sequence gaps,
which is exactly the regime of packet-level FEC over a lossy link.
"""

_FIELD_POLY = 0x11D

_EXP = [0] * 512
_LOG = [0] * 256


def _init_tables():
    x = 1
    for i in range(255):
        _EXP[i] = x
        _LOG[x] = i
        x <<= 1
        if x & 0x100:
            x ^= _FIELD_POLY
    for i in range(255, 512):
        _EXP[i] = _EXP[i - 255]


_init_tables()


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def gf_inv(a):
    if a == 0:
        raise ZeroDivisionError("inverse of 0 in GF(256)")
    return _EXP[255 - _LOG[a]]


def _scale(block, f):
    return bytearray(gf_mul(f, b) for b in block)


def _axpy(dst, src, f):
    for i in range(len(dst)):
        dst[i] ^= gf_mul(f, src[i])


class RSBlock:
    """Systematic (n=k+m, k) Cauchy MDS erasure code over GF(256)."""

    def __init__(self, k, m):
        if k < 1 or m < 1 or (k + m) > 256:
            raise ValueError("require 1 <= k, 1 <= m, k+m <= 256")
        self.k = k
        self.m = m
        self.n = k + m
        self._g = self._generator()

    def _generator(self):
        g = [[0] * self.k for _ in range(self.n)]
        for i in range(self.k):
            g[i][i] = 1
        for r in range(self.m):
            y = self.k + r
            for j in range(self.k):
                g[self.k + r][j] = gf_inv(j ^ y)  # Cauchy: 1/(x_j + y_r)
        return g

    def parity(self, data_blocks):
        """Return ``m`` parity blocks (bytearrays) for ``k`` equal-length data blocks."""
        if len(data_blocks) != self.k:
            raise ValueError("expected %d data blocks" % self.k)
        length = len(data_blocks[0])
        out = []
        for r in range(self.m):
            row = self._g[self.k + r]
            acc = bytearray(length)
            for j in range(self.k):
                coef = row[j]
                if coef:
                    _axpy(acc, data_blocks[j], coef)
            out.append(acc)
        return out

    def recover(self, received):
        """Rebuild the ``k`` data blocks from any ``>= k`` received packets.

        ``received`` maps packet index (0..k-1 data, k..k+m-1 parity) to a
        bytearray. Returns a list of ``k`` data blocks, or ``None`` if there are
        fewer than ``k`` packets (unrecoverable).
        """
        if len(received) < self.k:
            return None
        idxs = sorted(received.keys())[: self.k]
        length = len(received[idxs[0]])

        mat = []
        rhs = []
        for idx in idxs:
            mat.append(list(self._g[idx]))
            rhs.append(bytearray(received[idx]))

        if not self._solve(mat, rhs):
            return None
        return rhs

    @staticmethod
    def _solve(mat, rhs):
        k = len(mat)
        for col in range(k):
            piv = None
            for r in range(col, k):
                if mat[r][col] != 0:
                    piv = r
                    break
            if piv is None:
                return False
            mat[col], mat[piv] = mat[piv], mat[col]
            rhs[col], rhs[piv] = rhs[piv], rhs[col]

            inv = gf_inv(mat[col][col])
            mat[col] = [gf_mul(inv, x) for x in mat[col]]
            rhs[col] = _scale(rhs[col], inv)

            for r in range(k):
                if r == col:
                    continue
                f = mat[r][col]
                if f == 0:
                    continue
                mat[r] = [mat[r][c] ^ gf_mul(f, mat[col][c]) for c in range(k)]
                _axpy(rhs[r], rhs[col], f)
        return True
