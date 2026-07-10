"""AprilTag (36h11) detection FPS benchmark for the OV9281 cameras.

Measures, single-camera and both-in-parallel, at 1280x800 uint8:
  * capture-only   - capture_array() (Rust mmap read + unpack, no detect)
  * capture+detect - capture_array() -> cv2.aruco.ArucoDetector.detectMarkers()

Both the Rust capture and OpenCV's detectMarkers() release the GIL, so two
cameras on two threads run capture+detect truly in parallel. Each thread owns
its own ArucoDetector (detectors are not safe to share across threads).

Runs at the default frame length and at minimum vertical_blanking (max sensor
fps). Frames are read, detected, and discarded - nothing is stored. The mean
tags/frame and detect-rate columns confirm the tag is actually in view.

    sudo modprobe ov9282
    uv run python bench_apriltag.py
"""
from __future__ import annotations

import subprocess
import threading
import time

import cv2

from rcam import Camera, list_cameras

DUR = 4.0          # seconds per measurement
WARMUP = 0.5       # seconds discarded before timing


def make_detector() -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


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


def read_loop(cam: Camera, detect: bool, stop: threading.Event, ctr: Counter):
    # Per-thread detector: ArucoDetector is not documented thread-safe.
    detector = make_detector() if detect else None
    while not stop.is_set():
        frame = cam.capture_array()          # HxW uint8, OpenCV-ready gray
        if detector is not None:
            _corners, ids, _rej = detector.detectMarkers(frame)
            if ids is not None:
                ctr.tags += len(ids)
        ctr.frames += 1


def bench(cams: list[Camera], detect: bool) -> list[tuple[float, float]]:
    """Threaded capture(+detect) for DUR seconds; return (fps, tags/frame) per cam."""
    stop = threading.Event()
    counters = [Counter() for _ in cams]
    threads = [threading.Thread(target=read_loop, args=(c, detect, stop, ctr))
               for c, ctr in zip(cams, counters)]
    for c in cams:
        c.flush(4)                           # discard warmup/queue
    for t in threads:
        t.start()
    time.sleep(WARMUP)
    base = [(ctr.frames, ctr.tags) for ctr in counters]
    t0 = time.perf_counter()
    time.sleep(DUR)
    dt = time.perf_counter() - t0
    stop.set()
    for t in threads:
        t.join()
    out = []
    for ctr, (bf, bt) in zip(counters, base):
        df = ctr.frames - bf
        out.append((df / dt, (ctr.tags - bt) / df if df else 0.0))
    return out


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
        for detect, name in ((False, "capture-only"), (True, "capture+detect")):
            for scope, sel in (("single", labels[:1]), ("parallel", labels)):
                cams = [Camera(l).start() for l in sel]
                for c in cams:
                    set_vblank(c, vblank)
                try:
                    results = bench(cams, detect)
                finally:
                    for c in cams:
                        c.stop()
                row(f"{name} {scope}", results)
        print()


if __name__ == "__main__":
    main()
