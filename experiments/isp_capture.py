"""Experiment: does Pi-style processing (rcam.isp) help real synced capture?

Records the same scene back to back in each processing mode, on both cameras,
phase-aligned with FrameSync and with *identical* sensor exposure/gain, so the
only variable between runs is what happens to the pixels after readout. Every
frame is kept (raw uint8 stack + per-frame metadata), then ``analyze`` runs
rapidtag over all of them offline so detection cost cannot drop frames during
recording.

    # record 10 s per mode at 2.5 ms / 2x (the default A/B: raw vs pisp)
    uv run --no-sync python experiments/isp_capture.py record
    # other settings / modes
    uv run --no-sync python experiments/isp_capture.py record \\
        --modes raw pisp pisp-auto --exposure 1500 --gain 4 --seconds 20
    # detection + sync stats for a recording
    uv run --no-sync python experiments/isp_capture.py analyze recordings/<stamp>

Modes: ``raw`` (the old high-byte unpack), ``pisp`` / ``vc4`` (Pi gamma and
black level with exposure and gain pinned to --exposure/--gain), and
``pisp-auto`` (the Pi AGC free to choose - note the two cameras may then pick
different exposures, which shifts their exposure midpoints apart).

Output layout::

    recordings/<stamp>/
      meta.json                    settings, size, cameras, per-run sync report
      <mode>/<CAM>.u8              N x H x W uint8 frames, back to back
      <mode>/<CAM>.csv             seq, ts_ns, exposure_us, analogue_gain, digital_gain
      <mode>/<CAM>.png             first frame, for eyeballing
      summary.json                 written by `analyze`
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import polars as pl

from rcam import Camera, FrameSync, list_cameras

W, H = 1280, 800
DICT = "DICT_APRILTAG_36h11"
MODES = ("raw", "pisp", "vc4", "pisp-auto")
RESYNC_EVERY_S = 2.0


def _open(label: str, mode: str, exposure_us: float, gain: float) -> Camera:
    cam = Camera(label)
    isp = None if mode == "raw" else mode.removesuffix("-auto")
    cam.configure((W, H), isp=isp)
    if mode.endswith("-auto"):
        cam.set_controls({"AeEnable": True}, settle=False)
    else:
        # Same keys for every mode: on raw they go to the sensor, with the ISP
        # on they pin the AGC - either way the sensor ends up identical.
        cam.set_controls({"ExposureTime": exposure_us, "AnalogueGain": gain}, settle=False)
    return cam.start()


def record_mode(mode: str, labels: list[str], out: Path, args) -> dict:
    cams = [_open(l, mode, args.exposure, args.gain) for l in labels]
    try:
        # Let the AGC (pisp-auto) converge / the pinned values latch.
        for c in cams:
            for _ in range(args.warmup):
                c.capture_array()
        sync = None
        report = None
        if len(cams) == 2 and not args.no_sync:
            sync = FrameSync(cams[0], cams[1])
            print(f"[{mode}] aligning {labels[1]} onto {labels[0]}")
            report = sync.align(verbose=True)
            for c in cams:            # drain what queued up during align
                c.flush(4)

        out.mkdir(parents=True, exist_ok=True)
        rows: list[list[tuple]] = [[] for _ in cams]
        stop = threading.Event()

        def loop(i: int):
            cam, rec = cams[i], rows[i]
            with open(out / f"{labels[i]}.u8", "wb") as f:
                while not stop.is_set():
                    frame, ts, seq = cam.capture_array_meta()
                    f.write(frame.tobytes())
                    if cam._agc is not None:
                        m = cam._metadata
                        rec.append((seq, ts, m["ExposureTime"], m["AnalogueGain"],
                                    m["DigitalGain"]))
                    else:
                        rec.append((seq, ts, args.exposure, args.gain, 1.0))
                    if len(rec) == 1:
                        cv2.imwrite(str(out / f"{labels[i]}.png"), frame)

        threads = [threading.Thread(target=loop, args=(i,)) for i in range(len(cams))]
        t0 = time.monotonic()
        for t in threads:
            t.start()
        resyncs = 0
        while time.monotonic() - t0 < args.seconds:
            time.sleep(RESYNC_EVERY_S)
            if sync is not None and len(rows[0]) > 30 and len(rows[1]) > 30:
                ref = [r[1] for r in rows[0][-30:]]
                adj = [r[1] for r in rows[1][-30:]]
                if sync.resync_if_needed(ref, adj) is not None:
                    resyncs += 1
        stop.set()
        for t in threads:
            t.join()
        dt = time.monotonic() - t0

        for label, rec in zip(labels, rows):
            pl.DataFrame(rec, schema=["seq", "ts_ns", "exposure_us", "analogue_gain",
                                      "digital_gain"], orient="row"
                         ).write_csv(out / f"{label}.csv")
            print(f"[{mode}] {label}: {len(rec)} frames in {dt:.1f} s "
                  f"({len(rec) / dt:.1f} fps), exposure {rec[-1][2]} us, "
                  f"gain {rec[-1][3]:.2f}, dg {rec[-1][4]:.2f}")
        return {"phase_us": None if report is None else report.phase_us,
                "resyncs": resyncs, "seconds": dt}
    finally:
        for c in cams:
            c.stop()


def cmd_record(args):
    labels = args.cams or list_cameras()
    if not labels:
        raise SystemExit("no cameras (is ov9282 loaded?)")
    root = Path(args.out) / time.strftime("%Y%m%d-%H%M%S")
    meta = {"size": [W, H], "cameras": labels, "exposure_us": args.exposure,
            "gain": args.gain, "modes": args.modes, "runs": {}}
    for mode in args.modes:
        meta["runs"][mode] = record_mode(mode, labels, root / mode, args)
        (root / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"\nrecorded -> {root}\nnext: uv run --no-sync python "
          f"experiments/isp_capture.py analyze {root}")


def _detect_all(path: Path, batch: int) -> tuple[np.ndarray, list[set], float, float]:
    import rapidtag

    frames = np.memmap(path, np.uint8, "r").reshape(-1, H, W)
    counts = np.zeros(len(frames), np.int32)
    ids: list[set] = []
    t_det = 0.0
    for s in range(0, len(frames), batch):
        chunk = [np.ascontiguousarray(f) for f in frames[s:s + batch]]
        t0 = time.perf_counter()
        res = rapidtag.detect_markers_batch(chunk, DICT)
        t_det += time.perf_counter() - t0
        for j, (_c, i) in enumerate(res):
            got = set() if i is None else set(np.ravel(i).tolist())
            counts[s + j] = len(got)
            ids.append(got)
    return counts, ids, t_det / max(1, len(frames)), float(frames[::10].mean())


def cmd_analyze(args):
    root = Path(args.dir)
    meta = json.loads((root / "meta.json").read_text())
    summary: dict = {}
    for mode in meta["modes"]:
        d = root / mode
        per_cam = {}
        for label in meta["cameras"]:
            df = pl.read_csv(d / f"{label}.csv")
            counts, ids, det_s, mean8 = _detect_all(d / f"{label}.u8", args.batch)
            seq = df["seq"].to_numpy()
            seen = set().union(*ids) if ids else set()
            per_cam[label] = {
                "frames": len(counts),
                "dropped": int(np.clip(np.diff(seq) - 1, 0, None).sum()),
                "mean8": round(mean8, 1),
                "detect_rate": round(float((counts > 0).mean()), 4),
                "tags_per_frame": round(float(counts.mean()), 3),
                "all_ids_rate": round(float(np.mean([len(s) == len(seen) for s in ids])), 4)
                                if seen else 0.0,
                "ids_seen": sorted(seen),
                "detect_ms": round(det_s * 1e3, 2),
                "exposure_us": [int(df["exposure_us"].min()), int(df["exposure_us"].max())],
                "gain": [float(df["analogue_gain"].min()), float(df["analogue_gain"].max())],
                "_ids": ids, "_ts": df["ts_ns"].to_numpy(),
            }
        # Stereo: frames paired by nearest timestamp; a pair is only useful for
        # triangulation if the same tag is found in both views.
        if len(meta["cameras"]) == 2:
            a, b = (per_cam[l] for l in meta["cameras"])
            idx = np.clip(np.searchsorted(b["_ts"], a["_ts"]), 1, len(b["_ts"]) - 1)
            idx -= (a["_ts"] - b["_ts"][idx - 1]) < (b["_ts"][idx] - a["_ts"])
            skew_us = np.abs(a["_ts"] - b["_ts"][idx]) / 1e3
            n = min(len(a["_ids"]), len(idx))
            shared = [len(a["_ids"][k] & b["_ids"][idx[k]]) for k in range(n)
                      if idx[k] < len(b["_ids"])]
            per_cam["stereo"] = {
                "skew_us_median": round(float(np.median(skew_us)), 1),
                "skew_us_p95": round(float(np.percentile(skew_us, 95)), 1),
                "shared_tags_per_pair": round(float(np.mean(shared)), 3),
                "pairs_with_shared_tag": round(float(np.mean(np.array(shared) > 0)), 4),
            }
        for v in per_cam.values():
            v.pop("_ids", None)
            v.pop("_ts", None)
        summary[mode] = per_cam

    (root / "summary.json").write_text(json.dumps(summary, indent=2))
    cols = ["frames", "dropped", "mean8", "detect_rate", "tags_per_frame",
            "all_ids_rate", "detect_ms", "exposure_us", "gain"]
    print(f"{'mode':10s} {'cam':6s} " + " ".join(f"{c:>14s}" for c in cols))
    for mode, per_cam in summary.items():
        for label in meta["cameras"]:
            r = per_cam[label]
            print(f"{mode:10s} {label:6s} " + " ".join(f"{str(r[c]):>14s}" for c in cols))
        if "stereo" in per_cam:
            s = per_cam["stereo"]
            print(f"{mode:10s} stereo  skew median {s['skew_us_median']} us, "
                  f"p95 {s['skew_us_p95']} us; shared tags/pair "
                  f"{s['shared_tags_per_pair']}, pairs with a shared tag "
                  f"{s['pairs_with_shared_tag'] * 100:.1f}%")
    print(f"\n-> {root / 'summary.json'}")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--modes", nargs="+", choices=MODES, default=["raw", "pisp"])
    r.add_argument("--cams", nargs="+", help="default: all detected")
    r.add_argument("--exposure", type=float, default=2500.0, help="us (pinned modes)")
    r.add_argument("--gain", type=float, default=2.0, help="analogue gain (pinned modes)")
    r.add_argument("--seconds", type=float, default=10.0)
    r.add_argument("--warmup", type=int, default=30, help="frames discarded first")
    r.add_argument("--no-sync", action="store_true", help="skip FrameSync alignment")
    r.add_argument("--out", default="recordings")
    r.set_defaults(func=cmd_record)
    a = sub.add_parser("analyze")
    a.add_argument("dir")
    a.add_argument("--batch", type=int, default=8)
    a.set_defaults(func=cmd_analyze)
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
