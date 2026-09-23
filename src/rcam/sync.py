"""Frame-phase alignment for two free-running OV9281 sensors.

Each camera module self-clocks off its own 24 MHz oscillator and there is no
FSIN/trigger wiring between them, so the two sensors free-run with an arbitrary
phase offset: measured cold on the Dragon Q6A, -14.9 ms out of a 33.3 ms frame
period, i.e. very nearly anti-phase. Two frames carrying the same index were
exposed 15 ms apart, and no amount of capturing "at the same time" on the host
can undo that - the offset is already in the pixels.

What *can* be steered is the sensors' cadence. Frame period is
``(height + vertical_blanking) x line_time``, so inflating ``vertical_blanking``
for a moment stretches one frame and permanently retards that sensor's phase.
One camera is picked as the reference and the other is walked onto it:

    ref ---|--------|--------|--------|--------|--------|      free-running
    adj ------|--------|-----------------|--------|-----|      one stretched frame
                                          ^ phase now aligned

Measured against the kernel's own frame timestamps this converges to about
+/-100 us, which is the measurement noise floor - roughly 0.3% of a frame period,
down from 45%.

The two crystals also differ by ~50 ppm, so alignment decays at ~3 ms/minute and
has to be topped up during a long recording; :meth:`FrameSync.resync_if_needed`
does that from timestamps the caller already has, without stealing frames.

Both the measurement and the correction rest on the V4L2 buffer timestamp
(``ts-monotonic``, ``ts-src-eof``) that ``Camera.capture_meta()`` exposes. The
old approach of stamping ``time.monotonic_ns()`` in Python after the frame comes
back carries 1.6-1.9 ms of scheduling jitter, which is larger than the residual
we are trying to correct, so it cannot close this loop.

Example
-------
    from rcam import Camera, FrameSync

    cams = [Camera("CAM2").configure(...).start(), Camera("CAM3")...]
    sync = FrameSync(cams[0], cams[1])
    sync.align()                      # before recording starts
    ...
    sync.resync_if_needed(ts_ref, ts_adj)   # periodically, from recorded stamps
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .camera import Camera


@dataclass
class PhaseReport:
    """Outcome of one phase measurement."""

    phase_us: float
    """Offset of the adjusted sensor from the reference, wrapped to +/-T/2.

    Negative means the adjusted camera exposes *earlier* than the reference.
    """
    jitter_us: float
    """Circular standard deviation of the per-frame offsets."""
    drift_ppm: float
    """Rate the phase is walking, from a fit over the measurement window."""
    n: int
    """Frame pairs the estimate is built from."""
    dropped: tuple[int, int]
    """Frames the sensors produced that never reached us, per camera."""

    def __str__(self) -> str:
        return (
            f"phase {self.phase_us:+8.1f} us  jitter {self.jitter_us:6.1f} us  "
            f"drift {self.drift_ppm:+6.1f} ppm  n={self.n}  dropped={list(self.dropped)}"
        )


def wrap(offset_us: float, period_us: float) -> float:
    """Fold an offset into (-T/2, +T/2].

    Free-running sensors make frame *pairing* arbitrary up to whole periods: at
    33.3 ms/frame an offset of 30 ms is really -3.3 ms with the pairing off by
    one. Only the wrapped value is physically meaningful.
    """
    return (offset_us + period_us / 2) % period_us - period_us / 2


def _circular_stats(offsets_us, period_us: float) -> tuple[float, float]:
    """Mean and standard deviation of offsets that live on a circle of size T.

    Averaging raw offsets would be wrong for the same reason :func:`wrap` is
    needed - values just either side of the wrap point would average to the
    opposite phase. Averaging them as unit vectors does not have that failure.
    """
    if not offsets_us:
        return 0.0, float("inf")
    scale = 2 * math.pi / period_us
    cos_mean = sum(math.cos(o * scale) for o in offsets_us) / len(offsets_us)
    sin_mean = sum(math.sin(o * scale) for o in offsets_us) / len(offsets_us)
    mean = math.atan2(sin_mean, cos_mean) / scale
    r = math.hypot(cos_mean, sin_mean)
    std = math.sqrt(-2 * math.log(r)) / scale if 0 < r < 1 else 0.0
    return mean, std


def phase_from_timestamps(ts_ref_ns, ts_adj_ns, period_us: float) -> PhaseReport:
    """Phase between two cameras from frame timestamps the caller already holds.

    Takes no frames of its own, so it is safe to call mid-recording on the
    timestamps a capture thread has been writing. Both sequences are paired by
    index; the pairing being off by whole frames does not matter, since the
    result is wrapped into a single period either way.
    """
    n = min(len(ts_ref_ns), len(ts_adj_ns))
    if n < 2:
        return PhaseReport(0.0, float("inf"), 0.0, n, (0, 0))
    t_ref = [t / 1000.0 for t in ts_ref_ns[-n:]]  # ns -> us
    t_adj = [t / 1000.0 for t in ts_adj_ns[-n:]]
    offsets = [wrap(b - a, period_us) for a, b in zip(t_ref, t_adj)]
    mean, std = _circular_stats(offsets, period_us)

    # Drift: fit the residual about the circular mean against time. Re-wrapping
    # the residual keeps the fit sane when the offsets straddle +/-T/2, and the
    # residual stays small (tens of us) over any sane window, so it never wraps
    # a second time.
    span_us = t_ref[-1] - t_ref[0]
    drift_ppm = 0.0
    if span_us > 0:
        resid = [wrap(o - mean, period_us) for o in offsets]
        t0 = t_ref[0]
        xs = [t - t0 for t in t_ref]
        x_bar = sum(xs) / n
        y_bar = sum(resid) / n
        sxx = sum((x - x_bar) ** 2 for x in xs)
        if sxx > 0:
            slope = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, resid)) / sxx
            drift_ppm = slope * 1e6
    return PhaseReport(mean, std, drift_ppm, n, (0, 0))


class FrameSync:
    """Walks one sensor's frame phase onto another's.

    ``ref`` is left alone; only ``adj`` is retimed, so anything else keyed to
    the reference camera's cadence keeps working. Both cameras must already be
    ``start()``ed on the native backend (the v4l2-ctl fallback exposes no buffer
    timestamps and cannot be synced).
    """

    def __init__(self, ref: Camera, adj: Camera):
        if ref._cap is None or adj._cap is None:
            raise RuntimeError(
                "FrameSync needs both cameras started on the native backend "
                "(rcam._native); the v4l2-ctl fallback has no frame timestamps"
            )
        self.ref = ref
        self.adj = adj
        # Cached because every read is a v4l2-ctl subprocess and neither value
        # changes while streaming at a fixed rate.
        self.line_time_us = adj.line_time_us()
        self.period_us = adj.frame_period_us()
        self.nudges = 0

    # -- measurement -------------------------------------------------------
    def flush(self, n: int = 6) -> None:
        """Drop queued frames on both cameras, in parallel.

        Has to be parallel: draining one camera while the other sits idle lets
        the idle one's four V4L2 buffers overflow, so it would come back with a
        fresh backlog the moment we turned to it.
        """
        threads = [
            threading.Thread(target=lambda c=c: [c.capture_meta() for _ in range(n)])
            for c in (self.ref, self.adj)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def measure(self, n: int = 45, flush: bool = True) -> PhaseReport:
        """Grab ``n`` frames from each camera and report the phase between them.

        Pixels are never copied out - only the buffer header is read - so this
        costs one frame period per sample and nothing else.
        """
        if flush:
            self.flush()
        stamps: dict[int, list[tuple[int, int]]] = {0: [], 1: []}

        def collect(slot: int, cam: Camera) -> None:
            out = stamps[slot]
            for _ in range(n):
                out.append(cam.capture_meta())

        threads = [
            threading.Thread(target=collect, args=(i, c))
            for i, c in enumerate((self.ref, self.adj))
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        report = phase_from_timestamps(
            [ts for ts, _ in stamps[0]], [ts for ts, _ in stamps[1]], self.period_us
        )
        # A sequence counter that advances by more than one means the sensor
        # made a frame the driver could not hand us.
        dropped = tuple(
            sum(
                b - a - 1
                for (_, a), (_, b) in zip(stamps[s], stamps[s][1:])
                if b - a > 1
            )
            for s in (0, 1)
        )
        report.dropped = dropped
        return report

    # -- correction --------------------------------------------------------
    def nudge(self, phase_us: float) -> float:
        """Retard ``adj`` by whatever brings ``phase_us`` to zero.

        Only delays are possible - a sensor can be slowed by adding blanking but
        not hurried - so an adjusted camera that is running *late* is corrected
        the long way round, by delaying it almost a full period. That costs one
        stretched frame interval and no dropped frames.
        """
        delay_us = (-phase_us) % self.period_us
        applied = self.adj.nudge_phase(delay_us, line_time_us=self.line_time_us)
        if applied:
            self.nudges += 1
        return applied

    def align(
        self,
        tol_us: float = 200.0,
        max_iters: int = 6,
        n: int = 45,
        verbose: bool = True,
    ) -> PhaseReport:
        """Iterate measure/nudge until the phase is within ``tol_us``.

        Closed-loop because the correction is open-loop imprecise: the stretched
        vblank is held by a sleep, so it may cover one frame or two, and the
        overshoot is only known afterwards from the timestamps. Two or three
        rounds normally reach the ~100 us measurement floor.
        """
        report = self.measure(n)
        if verbose:
            print(f"  start   {report}")
        for i in range(max_iters):
            if abs(report.phase_us) <= tol_us:
                break
            self.nudge(report.phase_us)
            report = self.measure(n)
            if verbose:
                print(f"  iter {i + 1}  {report}")
        if verbose:
            verdict = (
                "aligned" if abs(report.phase_us) <= tol_us else "NOT within tolerance"
            )
            print(
                f"  {verdict}: {report.phase_us:+.1f} us "
                f"({abs(report.phase_us) / self.period_us * 100:.2f}% of the "
                f"{self.period_us / 1000:.2f} ms frame period) after {self.nudges} nudges"
            )
        return report

    def resync_if_needed(
        self, ts_ref_ns, ts_adj_ns, threshold_us: float = 1000.0
    ) -> PhaseReport | None:
        """Top up the alignment mid-recording, from timestamps already captured.

        Takes no frames, so it can run while capture threads own the cameras.
        Returns the measured phase when a correction was applied, else ``None``.

        The threshold trades residual skew against disturbance: every nudge puts
        one long interval into the recording. At the ~50 ppm the two crystals
        differ by, a 1 ms threshold fires roughly every 20 s.

        Keep it well above the ~150 us measurement jitter. Set near or below it
        and the loop chases noise - it fires constantly and the phase wanders
        over a wider range than leaving it alone would have.
        """
        report = phase_from_timestamps(ts_ref_ns, ts_adj_ns, self.period_us)
        if report.n < 2 or abs(report.phase_us) <= threshold_us:
            return None
        self.nudge(report.phase_us)
        return report
