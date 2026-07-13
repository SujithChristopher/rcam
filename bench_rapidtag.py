"""AprilTag (36h11) detection FPS benchmark using rapidtag for the OV9281 cameras.

Same measurement structure as bench_apriltag.py (which uses cv2.aruco), but with
detection done by rapidtag (https://pypi.org/project/rapidtag/) instead:
  * capture-only   - capture_array() (Rust mmap read + unpack, no detect)
  * capture+detect - capture_array() -> rapidtag.detect_markers_batch()

rapidtag's detect_markers_batch() uses flat (frame x scale) parallelism with the
GIL released, and is designed to be called with all cameras' frames at once - a
single frame, a dual-camera pair, or a larger batch all keep the cores busy
without nested threading. So unlike bench_apriltag.py (one ArucoDetector per
camera thread), here each camera's capture runs on its own thread (writing into
a single-slot "latest frame" buffer) while ONE thread does the batched detect
across all cameras' most recent frames. This is substantially faster than
calling detect_markers per-camera-thread (1.4x-2x observed on 2 cameras,
depending on thermal/system state).

Runs at the default frame length and at minimum vertical_blanking (max sensor
fps). Frames are read, detected, and discarded - nothing is stored. The mean
tags/frame and detect-rate columns confirm the tag is actually in view.

    sudo modprobe ov9282
    uv run python bench_rapidtag.py
"""
from __future__ import annotations

import subprocess
import threading
import time

import rapidtag

from rcam import Camera, list_cameras

DUR = 4.0          # seconds per measurement
WARMUP = 0.5       # seconds discarded before timing
DICT = "DICT_APRILTAG_36h11"


def set_vblank(cam: Camera, vblank: int):
    # exposure=600 lines gives reliable 36h11 detection here (mean ~45/255) while
    # staying below the min-blanking frame length (800+110=910), so it never caps fps.
    subprocess.run(["v4l2-ctl", "-d", cam.sensor.subdev,
                    "--set-ctrl", f"vertical_blanking={vblank},exposure=600"],
                   check=True, capture_output=True)


class Counter:
    __slots__ = ("frames", "tags")

    def __init__(self):
        self.frames = 0
        self.tags = 0


def read_loop(cam: Camera, stop: threading.Event, ctr: Counter):
    while not stop.is_set():
        cam.capture_array()
        ctr.frames += 1


def bench_capture_only(cams: list[Camera]) -> list[tuple[float, float]]:
    stop = threading.Event()
    counters = [Counter() for _ in cams]
    threads = [threading.Thread(target=read_loop, args=(c, stop, ctr))
               for c, ctr in zip(cams, counters)]
    for c in cams:
        c.flush(4)
    for t in threads:
        t.start()
    time.sleep(WARMUP)
    base = [ctr.frames for ctr in counters]
    t0 = time.perf_counter()
    time.sleep(DUR)
    dt = time.perf_counter() - t0
    stop.set()
    for t in threads:
        t.join()
    return [((ctr.frames - b) / dt, 0.0) for ctr, b in zip(counters, base)]


class Latest:
    __slots__ = ("frame", "lock")

    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()


def capture_loop(cam: Camera, stop: threading.Event, latest: Latest):
    while not stop.is_set():
        frame = cam.capture_array()
        with latest.lock:
            latest.frame = frame


def bench_capture_detect(cams: list[Camera]) -> list[tuple[float, float]]:
    """Per-camera capture threads feed a single batched-detect loop.

    Detect throughput (not capture throughput) is the bottleneck here, so the
    reported fps is batches/sec (same for every camera, since they're detected
    together) - see module docstring for why batching beats per-camera calls.
    """
    stop = threading.Event()
    for c in cams:
        c.flush(4)
    latests = [Latest() for _ in cams]
    threads = [threading.Thread(target=capture_loop, args=(c, stop, l))
               for c, l in zip(cams, latests)]
    for t in threads:
        t.start()
    time.sleep(WARMUP)
    while any(l.frame is None for l in latests):
        time.sleep(0.001)

    batches = 0
    tags = [0] * len(cams)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < DUR:
        frames = []
        for l in latests:
            with l.lock:
                frames.append(l.frame)
        for i, (_corners, ids) in enumerate(rapidtag.detect_markers_batch(frames, DICT)):
            tags[i] += len(ids)
        batches += 1
    dt = time.perf_counter() - t0
    stop.set()
    for t in threads:
        t.join()
    return [(batches / dt, t / batches if batches else 0.0) for t in tags]


def row(label: str, results: list[tuple[float, float]]):
    fps = [r[0] for r in results]
    per = "  ".join(f"{f:6.1f}" for f in fps)
    tags = "  ".join(f"{t:4.1f}" for _, t in results)
    print(f"  {label:<24} {per:<18}  total={sum(fps):6.1f} fps   tags/frame={tags}")


def main():
    labels = list_cameras()
    print(f"cameras: {labels}\n")
    if not labels:
        print("none detected - run: sudo modprobe ov9282")
        return

    # vblank gates frame length; set it explicitly each pass (sensor controls
    # persist across runs, so we must not rely on "whatever was left").
    for tag, vblank in (("default blanking (vblank=1022)", 1022),
                        ("min blanking (vblank=110, max fps)", 110)):
        print(f"=== {tag} ===")
        for scope, sel in (("single", labels[:1]), ("parallel", labels)):
            cams = [Camera(l).start() for l in sel]
            for c in cams:
                set_vblank(c, vblank)
            try:
                results = bench_capture_only(cams)
            finally:
                for c in cams:
                    c.stop()
            row(f"capture-only {scope}", results)

        for scope, sel in (("single", labels[:1]), ("parallel", labels)):
            cams = [Camera(l).start() for l in sel]
            for c in cams:
                set_vblank(c, vblank)
            try:
                results = bench_capture_detect(cams)
            finally:
                for c in cams:
                    c.stop()
            row(f"capture+detect(batch) {scope}", results)
        print()


if __name__ == "__main__":
    main()
