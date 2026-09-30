"""Exactness of the native unpackers against a NumPy reference. No camera needed.

    uv run --no-sync python tests/test_unpack.py     (or pytest, if installed)

The Rust paths are hand-optimised (NEON for the 8-bit unpack, a fused
LUT + histogram pass for the ISP), so every change to them should be checked
bit-for-bit here. Random packed bytes exercise every 10-bit code and every
LSB pattern.
"""
import numpy as np

from rcam import _native, isp

W, H = 1280, 800


def _frames():
    rng = np.random.default_rng(0)
    yield "noise", rng.integers(0, 256, W * H * 10 // 8, dtype=np.uint8).tobytes()
    # A dark, narrow-histogram scene like a short exposure (many equal bins).
    v = rng.normal(85, 8, (H, W)).clip(0, 1023).astype(np.uint16).reshape(-1, 4)
    packed = np.empty((v.shape[0], 5), np.uint8)
    packed[:, :4] = v >> 2
    packed[:, 4] = (v[:, 0] & 3) | (v[:, 1] & 3) << 2 | (v[:, 2] & 3) << 4 | (v[:, 3] & 3) << 6
    yield "dark", packed.tobytes()


def _unpack10(buf: bytes) -> np.ndarray:
    g = np.frombuffer(buf, np.uint8).reshape(-1, 5).astype(np.uint16)
    return np.stack([(g[:, k] << 2) | ((g[:, 4] >> (2 * k)) & 3) for k in range(4)],
                    axis=1).reshape(H, W)


def _hist(v10: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Every 4th row, pixels 0 and 2 of each group, PiSP 15x15 zone weights."""
    zw, zh = (W // 15) & ~1, (H // 15) & ~1
    ox, oy = ((W - 15 * zw) // 2) & ~1, ((H - 15 * zh) // 2) & ~1
    xs = np.array([x for x in range(W) if x % 4 in (0, 2)])
    zx = np.where(xs < ox, 15, np.minimum((xs - ox) // zw, 15))
    grid = weights.reshape(15, 15)
    hist = np.zeros(1024, np.int64)
    for y in range(oy, H, 1):
        if y % 4:
            continue
        zy = min((y - oy) // zh, 15)
        if zy >= 15:
            continue
        ok = zx < 15
        np.add.at(hist, v10[y, xs[ok]], grid[zy][zx[ok]].astype(np.int64))
    return hist


def test_unpack_u8_exact():
    for name, buf in _frames():
        got = np.frombuffer(_native.unpack_u8(buf, W, H), np.uint8).reshape(H, W)
        assert (got == _unpack10(buf) >> 2).all(), name


def test_unpack_isp_exact():
    for name, buf in _frames():
        v10 = _unpack10(buf)
        for dg in (1.0, 2.7):
            lut = isp.build_lut(dg, gamma="pisp")
            px, hb = _native.unpack_isp(buf, W, H, lut.tobytes(),
                                        isp.CENTRE_WEIGHTED.tobytes())
            px = np.frombuffer(px, np.uint8).reshape(H, W)
            assert (px == lut[v10]).all(), f"{name} dg={dg} pixels"
            hist = np.frombuffer(hb, "<u4")
            assert (hist == _hist(v10, isp.CENTRE_WEIGHTED)).all(), f"{name} dg={dg} hist"


if __name__ == "__main__":
    test_unpack_u8_exact()
    test_unpack_isp_exact()
    print("ok")
