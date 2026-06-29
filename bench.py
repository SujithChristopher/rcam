"""FPS benchmark for the OV9281 cameras at full resolution (1280x800).

Measures, single-camera and both-in-parallel:
  * ground truth  - `v4l2-ctl` streaming (kernel pipeline, ~no userspace work)
  * rcam raw      - capture_buffer() (read Y10P, no unpack)
  * rcam unpack8  - capture_array() bit_depth=8 (NumPy unpack to uint8)

at the default frame length and at minimum vertical_blanking (max sensor fps).
Frames are read and discarded - nothing is stored.

    sudo modprobe ov9282
    uv run python bench.py
"""
from __future__ import annotations

import re
import subprocess
import threading
import time

from rcam import Camera, list_cameras

DUR = 4.0          # seconds per measurement
WARMUP = 0.5       # seconds discarded before timing


def set_vblank(cam: Camera, vblank: int):
    subprocess.run(["v4l2-ctl", "-d", cam.sensor.subdev,
                    "--set-ctrl", f"vertical_blanking={vblank},exposure=300"],
                   check=True, capture_output=True)


def ground_truth(videos: list[str], seconds: float) -> dict[str, float]:
    """Run a v4l2-ctl stream per video node concurrently; parse reported fps."""
    n = int(seconds * 130) + 30
    procs = {v: subprocess.Popen(
        ["v4l2-ctl", "-d", v, "--stream-mmap", f"--stream-count={n}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) for v in videos}
    out = {v: p.communicate()[0] for v, p in procs.items()}
    fps = {}
    for v, text in out.items():
        vals = [float(m) for m in re.findall(r"([\d.]+) fps", text)]
        fps[v] = max(vals) if vals else float("nan")
    return fps


def read_loop(cam: Camera, unpack: bool, stop: threading.Event, counter: list[int]):
    grab = cam.capture_array if unpack else cam.capture_buffer
    while not stop.is_set():
        grab()
        counter[0] += 1


def bench_python(cams: list[Camera], unpack: bool) -> list[float]:
    """Threaded read from each camera for DUR seconds; return per-cam fps."""
    stop = threading.Event()
    counters = [[0] for _ in cams]
    threads = [threading.Thread(target=read_loop, args=(c, unpack, stop, ctr))
               for c, ctr in zip(cams, counters)]
    for c in cams:
        c.flush(4)                       # discard warmup/queue
    for t in threads:
        t.start()
    time.sleep(WARMUP)
    base = [c[0] for c in counters]
    t0 = time.perf_counter()
    time.sleep(DUR)
    dt = time.perf_counter() - t0
    stop.set()
    for t in threads:
        t.join()
    return [(c[0] - b) / dt for c, b in zip(counters, base)]


def row(label: str, fps: list[float]):
    total = sum(fps)
    per = "  ".join(f"{f:6.1f}" for f in fps)
    print(f"  {label:<22} {per:<18}  total={total:6.1f} fps")


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
        # --- ground truth (kernel pipeline) ---
        for scope, sel in (("single (CAM2)", labels[:1]), ("parallel (all)", labels)):
            cams = [Camera(l) for l in sel]
            for c in cams:
                c._setup_pipeline()
                subprocess.run(["v4l2-ctl", "-d", c.chain.video, "-v",
                                "width=1280,height=800,pixelformat=Y10P"],
                               check=True, capture_output=True)
                if vblank:
                    set_vblank(c, vblank)
            gt = ground_truth([c.chain.video for c in cams], DUR)
            row(f"ground-truth {scope}", list(gt.values()))

        # --- rcam python path ---
        for unpack, name in ((False, "rcam-raw"), (True, "rcam-unpack8")):
            for scope, sel in (("single", labels[:1]), ("parallel", labels)):
                cams = [Camera(l).start() for l in sel]
                if vblank:
                    for c in cams:
                        set_vblank(c, vblank)
                try:
                    fps = bench_python(cams, unpack)
                finally:
                    for c in cams:
                        c.stop()
                row(f"{name} {scope}", fps)
        print()


if __name__ == "__main__":
    main()
