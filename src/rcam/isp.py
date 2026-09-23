"""A software port of the Raspberry Pi ISP's mono pipeline for the OV9281.

On a Pi, libcamera does not show you the raw sensor output. For the OV9281
mono tuning (``ov9281_mono.json``) the Pi ISP:

1. subtracts the black level (4096/65536 -> 64 codes at 10 bit),
2. multiplies by the AGC's digital gain,
3. maps through the tuning file's gamma curve (``rpi.contrast``),

and runs the AGC (``rpi.agc``) on every frame to pick exposure time, analogue
gain and digital gain. There is no colour, no lens shading table
(``rpi.alsc`` has ``n_iter: 0``) and no sharpening in that tuning, so those
three pixel stages plus the AGC are what makes a Pi frame look the way it does.

Steps 1-3 are folded into one 1024-entry lookup table applied during the
Rust Y10P unpack (no extra cost over the plain unpack), which also returns a
centre-weighted histogram of the raw codes for the AGC. :class:`Agc` is a
line-by-line port of ``AgcChannel`` from raspberrypi/libcamera
(``src/ipa/rpi/controller/rpi/agc_channel.cpp``), using the PiSP (Pi 5)
statistics path, where the AGC meters from the weighted histogram.

Not ported: spatial/temporal denoise (``rpi.denoise``) - it softens noise but
does not change brightness or contrast.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

# -- tuning (raspberrypi/libcamera src/ipa/rpi/{pisp,vc4}/data/ov9281_mono.json)
BLACK_LEVEL_16 = 4096            # rpi.black_level, 16-bit pipeline units

# rpi.contrast gamma_curve, (x, y) pairs in 16-bit units.
GAMMA_PISP = np.array([
    0, 0, 512, 2518, 1024, 5033, 1536, 7175, 2048, 9309, 2560, 10814, 3072, 12312,
    3584, 13773, 4096, 15225, 4608, 16566, 5120, 17899, 5632, 19221, 6144, 20534,
    6656, 21684, 7168, 22826, 7680, 24024, 8192, 25212, 9216, 27251, 10240, 29167,
    11264, 30947, 12288, 32696, 13312, 34309, 14336, 35849, 15360, 37194, 16384,
    38445, 17408, 39598, 18432, 40732, 19456, 41717, 20480, 42687, 22528, 44343,
    24576, 45871, 26624, 47222, 28672, 48441, 30720, 49460, 32768, 50470, 34816,
    51476, 36864, 52480, 38912, 53382, 40960, 54294, 43008, 55155, 45056, 56035,
    47104, 56920, 49152, 57824, 51200, 58737, 53248, 59666, 55296, 60604, 57344,
    61558, 59392, 62529, 61440, 63516, 63488, 64519, 65535, 65535,
], dtype=np.float64).reshape(-1, 2)
GAMMA_VC4 = np.array([
    0, 0, 1024, 5040, 2048, 9338, 3072, 12356, 4096, 15312, 5120, 18051, 6144,
    20790, 7168, 23193, 8192, 25744, 9216, 27942, 10240, 30035, 11264, 32005,
    12288, 33975, 13312, 35815, 14336, 37600, 15360, 39168, 16384, 40642, 18432,
    43379, 20480, 45749, 22528, 47753, 24576, 49621, 26624, 51253, 28672, 52698,
    30720, 53796, 32768, 54876, 36864, 57012, 40960, 58656, 45056, 59954, 49152,
    61183, 53248, 62355, 57344, 63419, 61440, 64476, 65535, 65535,
], dtype=np.float64).reshape(-1, 2)
GAMMA = {"pisp": GAMMA_PISP, "vc4": GAMMA_VC4}

# rpi.agc metering_modes.centre-weighted, PiSP 15x15 grid (row-major).
CENTRE_WEIGHTED = np.array([
    0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
    0, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 0,
    1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
    1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 1, 1,
    1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1,
    1, 1, 2, 2, 3, 3, 3, 4, 3, 3, 3, 2, 2, 1, 1,
    1, 1, 2, 2, 3, 3, 4, 4, 4, 3, 3, 2, 2, 1, 1,
    1, 1, 2, 2, 3, 3, 3, 4, 3, 3, 3, 2, 2, 1, 1,
    1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 1, 1,
    1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
    1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
    0, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 0,
    0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
], dtype=np.uint8)
METERING_MODES = {"centre-weighted": CENTRE_WEIGHTED,
                  "average": np.ones(225, np.uint8)}

# rpi.agc exposure_modes: (exposure time us, analogue gain) stages.
EXPOSURE_MODES = {
    "normal": ([100, 15000, 30000, 60000, 120000], [1.0, 2.0, 3.0, 4.0, 8.0]),
    "short": ([100, 5000, 10000, 20000, 30000], [1.0, 2.0, 4.0, 6.0, 8.0]),
    "long": ([1000, 30000, 60000, 90000, 120000], [1.0, 2.0, 4.0, 6.0, 12.0]),
}
# libcamera's AeExposureMode enum order.
EXPOSURE_MODE_IDS = {0: "normal", 1: "short", 2: "long"}

# rpi.agc y_target / constraint_modes.normal (a single LOWER bound).
Y_TARGET = ([0, 1000, 10000], [0.16, 0.165, 0.17])
CONSTRAINTS = [("LOWER", 0.98, 1.0, 0.4)]     # (bound, q_lo, q_hi, y_target)

# rpi.lux
LUX_REF = dict(shutter_us=2000.0, gain=1.0, lux=800.0, y=20000.0)

# AgcConfig defaults (not overridden by the ov9281 tuning).
SPEED = 0.2
STARTUP_FRAMES = 10
FAST_REDUCE_THRESHOLD = 0.4
STABLE_REGION = 0.02
MAX_DIGITAL_GAIN = 4.0
EV_GAIN_Y_TARGET_LIMIT = 0.9
DEFAULT_EXPOSURE_US = 1000.0
DEFAULT_GAIN = 1.0

# CamHelperOv9281 / the sensor driver.
FRAME_INTEGRATION_DIFF = 25          # min lines between frame length and exposure
MIN_ANALOGUE_GAIN = 1.0
MAX_ANALOGUE_GAIN = 255 / 16
CONTROL_DELAY = 2                    # frames before exposure/gain take effect


def build_lut(digital_gain: float = 1.0, *, gamma: str | np.ndarray | None = "pisp",
              brightness: float = 0.0, contrast: float = 1.0,
              black_level: int = BLACK_LEVEL_16) -> np.ndarray:
    """10-bit raw code -> 8-bit output, as the Pi ISP would render it.

    ``brightness`` (-1..1) and ``contrast`` (0..32) are libcamera's controls,
    applied to the gamma curve exactly as ``applyManualContrast`` does.
    ``gamma=None`` gives a linear output (black level + digital gain only).
    """
    x = np.arange(1024, dtype=np.float64) * 64.0             # 10 -> 16 bit
    x = np.clip((x - black_level) * (65535.0 / (65535.0 - black_level)), 0, None)
    x = np.clip(x * digital_gain, 0, 65535)
    if gamma is not None:
        curve = GAMMA[gamma] if isinstance(gamma, str) else np.asarray(gamma)
        y = np.interp(x, curve[:, 0], curve[:, 1])
    else:
        y = x
    if brightness != 0.0 or contrast != 1.0:
        y = np.clip((y - 32768) * contrast + 32768 + brightness * 65536, 0, 65535)
    return np.clip(np.round(y / 257.0), 0, 255).astype(np.uint8)


class Histogram:
    """The AGC's view of a frame: weighted counts over normalised Y in 0..1.

    Built from the raw-code histogram with the black level removed, which is
    where the Pi frontend gathers its statistics.
    """

    def __init__(self, raw_counts: np.ndarray, black_level: int = BLACK_LEVEL_16):
        bl = black_level / 64.0
        codes = np.arange(raw_counts.size, dtype=np.float64)
        self.values = np.clip((codes - bl) / (1023.0 - bl), 0.0, 1.0)
        self.counts = raw_counts.astype(np.float64)
        self.total = self.counts.sum()

    def mean(self, gain: float = 1.0) -> float:
        """Mean Y after ``gain``, saturating at 1 (``computeInitialY``)."""
        if self.total == 0:
            return 0.0
        return float((np.minimum(self.values * gain, 1.0) * self.counts).sum() / self.total)

    def inter_quantile_mean(self, q_lo: float, q_hi: float) -> float:
        """Mean Y of the samples between two quantiles (``interQuantileMean``)."""
        if self.total == 0:
            return 0.0
        cum = np.concatenate(([0.0], np.cumsum(self.counts))) / self.total
        lo, hi = cum[:-1], cum[1:]
        # Fraction of each bin's mass that falls inside [q_lo, q_hi].
        overlap = np.clip(np.minimum(hi, q_hi) - np.maximum(lo, q_lo), 0, None)
        w = overlap.sum()
        return float((overlap * self.values).sum() / w) if w > 0 else 0.0


@dataclasses.dataclass
class AgcStatus:
    exposure_us: float
    analogue_gain: float
    digital_gain: float
    total_exposure: float          # exposure_us * analogue * digital
    lux: float = 400.0


class Agc:
    """``AgcChannel`` from the Pi's libcamera IPA, fed with our histograms.

    Call :meth:`process` once per frame with that frame's histogram and the
    exposure/gain it was actually captured with; it returns the exposure time,
    analogue gain and digital gain to request next.

    picamera2 semantics: ``fixed_exposure_us`` / ``fixed_gain`` pin one half
    of the exposure while the AGC keeps driving the other (that is what
    setting ``ExposureTime`` or ``AnalogueGain`` does on a Pi with AE on).
    """

    def __init__(self, *, max_exposure_us: float, min_exposure_us: float = 1.0):
        self.min_exposure_us = min_exposure_us
        self.max_exposure_us = max_exposure_us
        self.enabled = True
        self.fixed_exposure_us = 0.0
        self.fixed_gain = 0.0
        self.ev = 1.0                      # linear; ExposureValue control is log2
        self.exposure_mode = "normal"
        self.metering_mode = "centre-weighted"
        self.flicker_period_us = 0.0
        self.frame_count = 0
        self.lux = 400.0
        self._filtered_total = 0.0
        self.status = AgcStatus(DEFAULT_EXPOSURE_US, DEFAULT_GAIN, 1.0,
                                DEFAULT_EXPOSURE_US * DEFAULT_GAIN)

    @property
    def weights(self) -> np.ndarray:
        return METERING_MODES[self.metering_mode]

    # -- limits (limitExposureTime / limitGain) ----------------------------
    def _limit_exposure(self, t: float) -> float:
        return t if not t else min(max(t, self.min_exposure_us), self.max_exposure_us)

    @staticmethod
    def _limit_gain(g: float) -> float:
        return g if not g else min(max(g, MIN_ANALOGUE_GAIN),
                                   MAX_ANALOGUE_GAIN * MAX_DIGITAL_GAIN)

    # -- one frame ---------------------------------------------------------
    def process(self, hist: Histogram, exposure_us: float, analogue_gain: float) -> AgcStatus:
        """Run the AGC on one frame's statistics.

        ``exposure_us``/``analogue_gain`` are what that frame was *captured*
        with (not what was last requested) - the Pi gets these from its
        DelayedControls; the caller tracks them the same way.
        """
        self.frame_count += 1
        self.lux = self._compute_lux(hist, exposure_us, analogue_gain)
        if not self.enabled and self.fixed_exposure_us == 0.0:
            self.fixed_exposure_us = exposure_us    # AE off freezes the current values
        if not self.enabled and self.fixed_gain == 0.0:
            self.fixed_gain = analogue_gain
        fixed_t = self._limit_exposure(self.fixed_exposure_us)
        fixed_g = self._limit_gain(self.fixed_gain)

        current_no_dg = exposure_us * analogue_gain
        gain, target_y = self._compute_gain(hist)
        target = self._compute_target_exposure(current_no_dg, gain, fixed_t, fixed_g)
        self._filter_exposure(target, fixed_t and fixed_g)
        total_no_dg = self._filtered_total
        # applyDigitalGain: "desaturate" - when overexposed, drop the real
        # exposure hard and let digital gain hold the brightness meanwhile.
        if target_y > FAST_REDUCE_THRESHOLD and gain < math.sqrt(target_y):
            total_no_dg *= FAST_REDUCE_THRESHOLD
        self.status = self._divide_up(total_no_dg, fixed_t, fixed_g)
        self.status.lux = self.lux
        return self.status

    def _compute_lux(self, hist: Histogram, exposure_us: float, gain: float) -> float:
        y16 = hist.mean() * 65536.0
        if exposure_us <= 0 or gain <= 0 or y16 <= 0:
            return self.lux
        return (LUX_REF["lux"] * (LUX_REF["shutter_us"] / exposure_us)
                * (LUX_REF["gain"] / gain) * (y16 / LUX_REF["y"]))

    def _compute_gain(self, hist: Histogram) -> tuple[float, float]:
        target_y = float(np.interp(self.lux, *Y_TARGET))
        target_y = min(EV_GAIN_Y_TARGET_LIMIT, target_y * self.ev)
        gain = 1.0
        for _ in range(8):
            extra = min(10.0, target_y / (hist.mean(gain) + 0.001))
            gain *= extra
            if extra < 1.01:
                break
        for bound, q_lo, q_hi, y in CONSTRAINTS:
            cy = min(EV_GAIN_Y_TARGET_LIMIT, y * self.ev)
            iqm = hist.inter_quantile_mean(q_lo, q_hi)
            new_gain = cy / iqm if iqm > 0 else 10.0
            if (bound == "LOWER" and new_gain > gain) or (bound == "UPPER" and new_gain < gain):
                gain, target_y = new_gain, cy
        return gain, target_y

    def _compute_target_exposure(self, current_no_dg, gain, fixed_t, fixed_g) -> float:
        if fixed_t and fixed_g:
            return fixed_t * fixed_g
        shutters, gains = EXPOSURE_MODES[self.exposure_mode]
        max_t = self._limit_exposure(fixed_t or shutters[-1])
        max_g = self._limit_gain(fixed_g or gains[-1])
        return min(current_no_dg * gain, max_t * max_g)

    def _filter_exposure(self, target: float, fully_fixed: bool):
        speed, stable = SPEED, STABLE_REGION
        if fully_fixed or self.frame_count <= STARTUP_FRAMES:
            speed, stable = 1.0, 0.0
        f = self._filtered_total
        if not f:
            self._filtered_total = target
        elif f * (1 - stable) < target < f * (1 + stable):
            pass
        else:
            if 0.8 * target < f < 1.2 * target:
                speed = math.sqrt(speed)
            self._filtered_total = speed * target + f * (1 - speed)

    def _divide_up(self, value: float, fixed_t: float, fixed_g: float) -> AgcStatus:
        shutters, gains = EXPOSURE_MODES[self.exposure_mode]
        t = self._limit_exposure(fixed_t or shutters[0])
        g = self._limit_gain(fixed_g or gains[0])
        if t * g < value:
            for stage in range(1, len(gains)):
                if not fixed_t:
                    st = self._limit_exposure(shutters[stage])
                    if st * g >= value:
                        t = value / g
                        break
                    t = st
                if not fixed_g:
                    if gains[stage] * t >= value:
                        g = value / t
                        break
                    g = self._limit_gain(gains[stage])
        if not fixed_t and not fixed_g and self.flicker_period_us:
            periods = int(t // self.flicker_period_us)
            if periods:
                nt = periods * self.flicker_period_us
                g *= t / nt
                t = nt
        ag = min(g, MAX_ANALOGUE_GAIN)
        no_dg = ag * t
        dg = min(max(self._filtered_total / no_dg, 1.0), MAX_DIGITAL_GAIN)
        self._filtered_total = no_dg * dg
        return AgcStatus(t, ag, dg, self._filtered_total)
