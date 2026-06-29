"""Parse the CAMSS media graph (``media-ctl -p``) to discover OV9281 sensors
and the capture chains they can be routed through.

The Dragon Q6A CAMSS exposes a crossbar: any CSIPHY can feed any CSID, and each
CSID feeds one VFE which exposes rdi0/1/2 video nodes. Our dual-OV9281 overlay
wires the sensors to fixed CSIPHYs (CAM2->csiphy2, CAM3->csiphy3) via an
IMMUTABLE link; everything downstream of the CSIPHY we choose ourselves.
"""
from __future__ import annotations

import re
import shutil
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class Sensor:
    name: str          # e.g. "ov9281 18-0060"
    subdev: str        # e.g. "/dev/v4l-subdev28" (where exposure/gain live)
    csiphy: str        # e.g. "msm_csiphy2" (entity it is linked to)

    @property
    def i2c(self) -> str:
        # "ov9281 18-0060" -> "18-0060"
        return self.name.split()[-1]

    @property
    def csiphy_id(self) -> int:
        return int(re.search(r"(\d+)$", self.csiphy).group(1))


@dataclass(frozen=True)
class CaptureChain:
    """A downstream path: csid -> vfeN_rdi0 (subdev) -> vfeN_video0 (/dev/videoX).

    ``rdi`` is the subdev we set links/formats on; ``video`` is the V4L2 capture
    node owned by the matching ``msm_vfeN_video0`` entity.
    """
    csid: str          # "msm_csid0"
    rdi: str           # "msm_vfe0_rdi0"
    video: str         # "/dev/video0"


def _media_print(media: str) -> str:
    if shutil.which("media-ctl") is None:
        raise RuntimeError("media-ctl not found (install v4l-utils)")
    return subprocess.run(
        ["media-ctl", "-d", media, "-p"],
        check=True, capture_output=True, text=True,
    ).stdout


def parse(media: str = "/dev/media0") -> tuple[list[Sensor], list[CaptureChain]]:
    """Return (sensors, capture_chains) discovered from the live media graph."""
    text = _media_print(media)
    blocks = re.split(r"\n(?=- entity )", text)

    sensors: list[Sensor] = []
    vfe_video0: dict[int, str] = {}       # vfe index -> "/dev/video0" (rdi0 node)
    csid_idx: set[int] = set()

    for blk in blocks:
        m = re.search(r"- entity \d+: (\S[^\(]*?) \(", blk)
        if not m:
            continue
        ent = m.group(1).strip()

        if ent.startswith("ov9281"):
            sd = re.search(r"device node name (/dev/v4l-subdev\d+)", blk)
            phy = re.search(r'-> "(msm_csiphy\d+)"', blk)
            if sd and phy:
                sensors.append(Sensor(ent, sd.group(1), phy.group(1)))
            continue
        # The capture node lives on a separate "msm_vfeN_video0" entity; rdi0
        # maps to video0 of the same VFE.
        vm = re.fullmatch(r"msm_vfe(\d+)_video0", ent)
        if vm:
            vid = re.search(r"device node name (/dev/video\d+)", blk)
            if vid:
                vfe_video0[int(vm.group(1))] = vid.group(1)
            continue
        cm = re.fullmatch(r"msm_csid(\d+)", ent)
        if cm:
            csid_idx.add(int(cm.group(1)))

    # Pair csidN -> vfeN_rdi0 (subdev) -> vfeN_video0 (/dev/video) by index.
    chains = [
        CaptureChain(f"msm_csid{i}", f"msm_vfe{i}_rdi0", vfe_video0[i])
        for i in sorted(csid_idx) if i in vfe_video0
    ]

    sensors.sort(key=lambda s: s.csiphy_id)
    return sensors, chains
