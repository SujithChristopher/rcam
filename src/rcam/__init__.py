"""rcam - picamera2-style access to the OV9281 cameras on the Radxa Dragon Q6A.

    from rcam import Camera, list_cameras
    print(list_cameras())                 # ['CAM2', 'CAM3']
    with Camera("CAM2") as cam:
        cam.set_controls({"ExposureLines": 600, "AnalogueGain": 4.0})
        frame = cam.capture_array()       # HxW uint8 numpy

Requires the out-of-tree driver loaded:  sudo modprobe ov9282
"""
from .camera import Camera, list_cameras
from .topology import parse as parse_topology

__all__ = ["Camera", "list_cameras", "parse_topology"]
