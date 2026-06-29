"""A small picamera2-style wrapper around the Dragon Q6A CAMSS pipeline for the
Waveshare OV9281 (mono global shutter).

Why not libcamera/picamera2: this board has no Qualcomm libcamera pipeline
handler, so libcamera falls back to the generic "simple" handler whose software
debayer rejects the OV9281's mono R8/R10 stream. For a manual-exposure mono
machine-vision sensor we go straight to V4L2: sensor controls on the subdev,
frames from the CAMSS RDI (format Y10P, MIPI-packed 10-bit), unpacked to NumPy.

Example
-------
    from rcam import Camera
    cam = Camera("CAM2")
    cam.configure(size=(1280, 800), bit_depth=8)
    cam.set_controls({"ExposureLines": 600, "AnalogueGain": 4.0, "VFlip": True})
    cam.start()
    frame = cam.capture_array()      # HxW uint8 (or uint16 if bit_depth=10)
    cam.stop()
"""
from __future__ import annotations

import shutil
import subprocess
from typing import Any

import numpy as np

from .topology import CaptureChain, Sensor, parse

# Sensor subdev V4L2 control names (from `v4l2-ctl --list-ctrls` on the ov9281).
_RAW_CTRLS = {
    "exposure", "analogue_gain", "vertical_blanking", "horizontal_blanking",
    "horizontal_flip", "vertical_flip",
}
# Fixed sensor characteristics (read-only controls report these).
_PIXEL_RATE = 160_000_000  # Hz


def _run(cmd: list[str]) -> str:
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def list_cameras(media: str = "/dev/media0") -> list[str]:
    """Return friendly camera labels present in the graph, e.g. ['CAM2','CAM3']."""
    sensors, _ = parse(media)
    return [_label(s) for s in sensors]


def _label(s: Sensor) -> str:
    # csiphy2 -> CAM2, csiphy3 -> CAM3 (matches the board's silkscreen).
    return f"CAM{s.csiphy_id}"


class Camera:
    """One OV9281. Resolve by label ('CAM2'/'CAM3'), index (0/1), or i2c id."""

    def __init__(self, cam: str | int = 0, *, media: str = "/dev/media0"):
        self.media = media
        sensors, chains = parse(media)
        if not sensors:
            raise RuntimeError(
                "No ov9281 sensors in the media graph. Is the driver loaded? "
                "Run: sudo modprobe ov9282"
            )
        self.sensor = self._resolve(cam, sensors)
        # Assign a downstream capture chain by this sensor's order, so two
        # cameras get distinct csid/vfe/video and can stream simultaneously.
        order = sensors.index(self.sensor)
        if order >= len(chains):
            raise RuntimeError("no free capture chain for this camera")
        self.chain: CaptureChain = chains[order]

        self.width = 1280
        self.height = 800
        self.bit_depth = 8
        self._proc: subprocess.Popen | None = None
        self._frame_bytes = self.width * self.height * 10 // 8

    @staticmethod
    def _resolve(cam: str | int, sensors: list[Sensor]) -> Sensor:
        if isinstance(cam, int):
            return sensors[cam]
        key = cam.strip().upper()
        for s in sensors:
            if _label(s) == key or s.i2c == cam or s.name == cam:
                return s
        raise ValueError(f"camera {cam!r} not found; have {[_label(s) for s in sensors]}")

    @property
    def label(self) -> str:
        return _label(self.sensor)

    # -- configuration -----------------------------------------------------
    def configure(self, size: tuple[int, int] = (1280, 800), *, bit_depth: int = 8):
        """Set capture resolution and output depth (8 -> uint8, 10 -> uint16)."""
        if bit_depth not in (8, 10):
            raise ValueError("bit_depth must be 8 or 10")
        self.width, self.height = size
        self.bit_depth = bit_depth
        self._frame_bytes = self.width * self.height * 10 // 8
        return self

    # -- sensor controls (picamera2-ish names) -----------------------------
    def set_controls(self, controls: dict[str, Any], *, settle: bool = True):
        """Apply sensor controls. Accepts friendly aliases or raw V4L2 names:

        ExposureLines : int  exposure in sensor lines (1..1797 @ 800 rows)
        ExposureTime  : float exposure in microseconds (converted to lines)
        AnalogueGain  : float 1.0..16.0 (mapped to the sensor's 16..255 code)
        HFlip / VFlip : bool
        FrameRate     : float target fps (sets vertical_blanking)
        ...plus any raw control: exposure, analogue_gain, vertical_blanking, etc.

        While streaming, the V4L2 queue holds a few already-captured frames, so a
        control change only shows up after them. With ``settle=True`` (default)
        those stale frames are drained so the next capture_array() reflects the
        new settings.
        """
        pairs: list[str] = []
        for key, val in controls.items():
            pairs += self._translate(key, val)
        if pairs:
            _run(["v4l2-ctl", "-d", self.sensor.subdev,
                  "--set-ctrl", ",".join(pairs)])
        if settle and self._proc is not None:
            self.flush()
        return self

    def flush(self, n: int = 6):
        """Discard ``n`` queued frames (use after changing controls mid-stream)."""
        for _ in range(n):
            self.capture_buffer()
        return self

    def _translate(self, key: str, val: Any) -> list[str]:
        k = key.strip()
        if k in _RAW_CTRLS:
            return [f"{k}={int(val)}"]
        kl = k.lower()
        if kl == "exposurelines":
            return [f"exposure={int(val)}"]
        if kl == "exposuretime":               # microseconds -> lines
            return [f"exposure={max(1, round(float(val) / self._line_time_us()))}"]
        if kl in ("analoguegain", "gain"):
            code = int(round(float(val) * 16))
            return [f"analogue_gain={min(255, max(16, code))}"]
        if kl == "hflip":
            return [f"horizontal_flip={int(bool(val))}"]
        if kl == "vflip":
            return [f"vertical_flip={int(bool(val))}"]
        if kl == "framerate":
            return [f"vertical_blanking={self._vblank_for_fps(float(val))}"]
        raise KeyError(f"unknown control {key!r}")

    def get_control(self, name: str) -> int:
        out = _run(["v4l2-ctl", "-d", self.sensor.subdev, "--get-ctrl", name])
        return int(out.split(":")[1])

    def _line_time_us(self) -> float:
        hblank = self.get_control("horizontal_blanking")
        return (self.width + hblank) / _PIXEL_RATE * 1e6

    def _vblank_for_fps(self, fps: float) -> int:
        line_len = self.width + self.get_control("horizontal_blanking")
        total_lines = _PIXEL_RATE / line_len / fps
        return max(110, round(total_lines - self.height))

    # -- streaming ---------------------------------------------------------
    def _setup_pipeline(self):
        m, phy, ch = self.media, self.sensor.csiphy, self.chain
        # Link only this camera's chain (no global reset -> the other camera's
        # pipeline keeps running for simultaneous capture).
        _run(["media-ctl", "-d", m, "-l", f"'{phy}':1 -> '{ch.csid}':0 [1]"])
        _run(["media-ctl", "-d", m, "-l", f"'{ch.csid}':1 -> '{ch.rdi}':0 [1]"])
        fmt = f"[fmt:Y10_1X10/{self.width}x{self.height}]"
        for pad in (f"'{self.sensor.name}':0", f"'{phy}':0",
                    f"'{ch.csid}':0", f"'{ch.rdi}':0"):
            _run(["media-ctl", "-d", m, "-V", f"{pad} {fmt}"])

    def start(self):
        if self._proc is not None:
            return self
        if shutil.which("v4l2-ctl") is None:
            raise RuntimeError("v4l2-ctl not found (install v4l-utils)")
        self._setup_pipeline()
        _run(["v4l2-ctl", "-d", self.chain.video,
              "-v", f"width={self.width},height={self.height},pixelformat=Y10P"])
        # Continuous raw stream to stdout (omit --stream-count => stream forever);
        # we read one frame at a time. stderr is kept for error diagnostics.
        self._stderr = subprocess.PIPE
        self._proc = subprocess.Popen(
            ["v4l2-ctl", "-d", self.chain.video, "--stream-mmap",
             "--stream-to=-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        return self

    def capture_buffer(self) -> bytes:
        """Read one raw Y10P frame (packed) as bytes (handles partial pipe reads)."""
        if self._proc is None:
            raise RuntimeError("call start() first")
        chunks = bytearray()
        while len(chunks) < self._frame_bytes:
            chunk = self._proc.stdout.read(self._frame_bytes - len(chunks))
            if not chunk:
                rc = self._proc.poll()
                raise RuntimeError(f"stream ended early (v4l2-ctl exit={rc}); "
                                   "check pipeline / that another process isn't "
                                   "holding the video node")
            chunks += chunk
        return bytes(chunks)

    def capture_array(self) -> np.ndarray:
        """Read one frame, unpacked to a HxW NumPy array.

        bit_depth=8  -> uint8  (the high byte of each pixel; fastest, OpenCV-ready)
        bit_depth=10 -> uint16 (full 10-bit value, 0..1023)
        """
        return self.unpack(self.capture_buffer())

    def unpack(self, buf: bytes) -> np.ndarray:
        """Unpack MIPI Y10P (4 px per 5 bytes) to a HxW array."""
        g = np.frombuffer(buf, np.uint8).reshape(-1, 5)
        if self.bit_depth == 8:
            return g[:, :4].reshape(self.height, self.width).copy()
        hi = g[:, :4].astype(np.uint16) << 2
        lo = g[:, 4]
        lsb = np.stack([(lo >> (2 * i)) & 0x3 for i in range(4)], axis=1)
        return (hi | lsb).reshape(self.height, self.width)

    def stop(self):
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        return self

    # -- context manager ---------------------------------------------------
    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
