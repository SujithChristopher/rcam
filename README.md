# rcam

picamera2-style Python access to the two **Waveshare OV9281** mono global-shutter
cameras on the **Radxa Dragon Q6A** (QCS6490 CAMSS).

Hardware bring-up (device-tree overlay, driver, EFI DTB deploy) lives in
[`ov9281/`](ov9281/README.md). This package is the userspace API on top of it.

## Why not libcamera / picamera2

This board has no Qualcomm libcamera pipeline handler, so libcamera falls back to
the generic *simple* handler whose software debayer rejects the OV9281's mono
R8/R10 stream (`Unsupported input format R8`). For a manual-exposure mono
machine-vision sensor that ISP/3A machinery adds nothing, so `rcam` talks to
V4L2 directly: **sensor controls on the subdev**, **frames from the CAMSS RDI**
(format `Y10P`, MIPI-packed 10-bit) unpacked into NumPy.

## Backends

Capture has two interchangeable backends, picked automatically:

- **native (Rust)** — a small PyO3 extension (`rcam._native`) that mmaps the
  CAMSS multiplanar video node and runs `DQBUF` + Y10P unpack + `QBUF` with the
  **GIL released**, so two cameras on two threads capture truly in parallel.
  Built with `uv sync` (maturin build backend). Preferred when present.
- **subprocess** — pipes frames from `v4l2-ctl --stream-mmap`; the fallback when
  the extension isn't built. Works, but the GIL/IPC caps parallel throughput.

Pass `Camera(..., native=False)` to force the subprocess path.

At full resolution (1280x800) with minimum blanking the native path matches the
raw kernel pipeline and scales linearly across both cameras — **~121 fps single,
~241 fps aggregate in parallel**, with 8-bit unpacking essentially free. Re-run
the table with `uv run python bench.py`.

## Setup

```bash
sudo modprobe ov9282        # out-of-tree driver; once per boot (see note below)
uv sync                     # installs rcam (editable) into .venv
uv run python main.py       # captures cam2.png + cam3.png
```

## Usage

```python
from rcam import Camera, list_cameras

print(list_cameras())                       # ['CAM2', 'CAM3']

with Camera("CAM2") as cam:                  # or Camera(0) / Camera("18-0060")
    cam.configure(size=(1280, 800), bit_depth=8)   # 8 -> uint8, 10 -> uint16
    cam.set_controls({
        "ExposureLines": 600,    # or "ExposureTime": 5000  (microseconds)
        "AnalogueGain":  4.0,    # 1.0 .. 16.0
        "VFlip": True,
    })
    frame = cam.capture_array()              # HxW NumPy array, ready for cv2
```

Both cameras stream **simultaneously** (independent CSID/VFE chains):

```python
a, b = Camera("CAM2").start(), Camera("CAM3").start()
fa, fb = a.capture_array(), b.capture_array()
a.stop(); b.stop()
```

### Controls

Set on the sensor subdev. Friendly aliases (left) map to raw V4L2 controls:

| alias | raw control | range |
|-------|-------------|-------|
| `ExposureLines` | `exposure` | 1 - 1797 lines |
| `ExposureTime` (us) | `exposure` | converted via line time |
| `AnalogueGain` / `Gain` | `analogue_gain` | 1.0 - 16.0 (code 16 - 255) |
| `HFlip` / `VFlip` | `horizontal_flip` / `vertical_flip` | bool |
| `FrameRate` | `vertical_blanking` | fps target |

Raw control names also pass straight through, e.g.
`set_controls({"vertical_blanking": 2000})`.

> While streaming, the V4L2 queue holds a few already-captured frames, so a
> control change only appears after them. `set_controls(..., settle=True)`
> (the default) drains those stale frames; call `cam.flush()` to drain manually.

## Notes / gotchas

- **Format is Y10P only.** The RDI advertises `GREY`/`Y10` but only `Y10P`
  actually streams; `rcam` unpacks it (8-bit fast path, or full 10-bit uint16).
- **Driver auto-load:** `ov9282` is out-of-tree and not loaded at boot. Either
  `sudo modprobe ov9282` each boot, or add `/etc/modules-load.d/ov9282.conf`
  containing `ov9282` (persistent).
- **After a kernel/image update** the EFI boot DTB reverts to stock (cameras
  disappear). Re-run `ov9281/scripts/deploy_efi_dtb.sh` and reboot.
- **i2c bus numbers are not stable** across boots; `rcam` discovers sensors by
  walking the media graph, so this doesn't matter.

## Layout

```
src/rcam/
  topology.py   discover sensors + capture chains from `media-ctl -p`
  camera.py     Camera class: configure / set_controls / capture_array
rust/lib.rs     native V4L2 mmap backend (rcam._native, built by maturin)
main.py         demo: snapshot from every camera
bench.py        full-res FPS benchmark (single + parallel)
ov9281/         hardware bring-up (overlay, driver, deploy scripts)
```
