"""Demo: capture one frame from every OV9281 and save a PNG preview.

    sudo modprobe ov9282        # one-time per boot (out-of-tree driver)
    uv run python main.py
"""
import cv2

from rcam import Camera, list_cameras


def main():
    labels = list_cameras()
    print(f"Cameras detected: {labels or 'none (is ov9282 loaded?)'}")
    for label in labels:
        with Camera(label) as cam:
            cam.set_controls({"ExposureLines": 600, "AnalogueGain": 4.0})
            frame = cam.capture_array()          # HxW uint8
            out = f"{label.lower()}.png"
            cv2.imwrite(out, frame)
            print(f"{label}: {frame.shape} {frame.dtype} "
                  f"min={frame.min()} mean={frame.mean():.1f} max={frame.max()} -> {out}")


if __name__ == "__main__":
    main()
