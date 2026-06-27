# OV9281 on the Radxa Dragon Q6A (QCS6490)

Brings up a **Waveshare/Arducam OV9281** (1 MP mono, global-shutter, Pi-style
MIPI CSI module) on the Radxa Dragon Q6A using the mainline Qualcomm **CAMSS**
driver. End result: a `/dev/video0` that streams **Y10 mono 1280×800** frames.

Status: **working** — verified by capturing frames and viewing the preview.

---

## TL;DR rebuild (fresh image)

```bash
cd ov9281

# 1. Build + install the sensor driver (ov9282 covers OV9281); not in stock kernel
make -C module
sudo install -D -m0644 module/ov9282.ko /lib/modules/$(uname -r)/updates/ov9282.ko
sudo depmod -a

# 2. Merge the overlay into the REAL boot DTB and deploy it (see "Boot mechanism")
sudo scripts/deploy_efi_dtb.sh
sudo reboot

# 3. After reboot: configure pipeline + capture
scripts/capture.sh
python3 scripts/y10p_to_pgm.py captures/ov9281_1280x800_y10p.raw   # -> .pgm preview
```

---

## Hardware facts (verified on-device)

- **Board:** Radxa Dragon Q6A, SoC `qcm6490` (sc7280-class). CAMSS supports it
  (`qcom,sc7280-camss`); CCI controllers `cci@ac4a000` (cci0) / `cci@ac4b000` (cci1)
  already exist in the base DTB.
- **Sensor:** OV9281, I²C address **0x60**, confirmed by reading chip-id reg
  `0x300A` → `0x92 0x81`. A companion/EEPROM sits at **0x70** (=0xa1, ignore).
- **The module self-clocks.** The 15-pin Pi CSI connector has **no host-MCLK
  pin** (per datasheet) — the board has its own 24 MHz oscillator, enabled by the
  connector's POWER-EN line. So a DT *virtual* `fixed-clock` is correct; do **not**
  chase a `camcc` MCLK for this module.
- **Verified wiring of the connector in use** (cross-wired vs Radxa's labels):
  - sensor I²C + data: **`cci1_i2c0`** / **CSIPHY2** (`port@2`), 2 data lanes
  - power-enable: **gpio78** (driving it high powers the sensor; gpio77 alone did
    not — we assert both to be safe)
  - link frequency 400 MHz, formats Y10/Y8 mono.

> ⚠️ Linux i²c **adapter numbers are not stable across boots** (`i2c-18`/`i2c-20`
> swap). Always resolve via `/sys/bus/i2c/devices/i2c-N/of_node` → `i2c-bus@0/@1`,
> or just `media-ctl -d /dev/media0 -p | grep ov9281`.

---

## Boot mechanism — the key gotcha

This image boots via **systemd-boot (UEFI/edk2)**, **not** U-Boot. The
`extlinux.conf` / `u-boot-update` machinery is present but **vestigial and
ignored** — editing `fdtoverlays` / `U_BOOT_FDT` there does nothing.

The DTB that actually boots is the one named on the systemd-boot loader entry:

```
/boot/efi/loader/entries/RadxaOS-<ver>.conf
   devicetree /RadxaOS/<ver>/qcs6490-radxa-dragon-q6a.dtb   <-- THIS file (on the EFI vfat partition)
   devicetree-overlay                                       <-- empty
```

We don't rely on firmware overlay application (the firmware control FDT has no
`__symbols__`, so `devicetree-overlay` / U-Boot `fdtoverlays` can't resolve
fixups). Instead we **pre-merge** our overlay into that boot DTB with
`fdtoverlay` and write it back. `scripts/deploy_efi_dtb.sh` does this and keeps
`*.orig` (pristine) and `*.bak` backups. `OF_CONFIGFS` is not built, so there is
no runtime/configfs overlay path either.

Recovery if a bad DTB stops boot: serial console on `ttyMSM0` (or fastboot/EDL),
then `sudo scripts/revert_efi_dtb.sh` (restores `*.orig`) and reboot.

---

## Capture pipeline

`ov9281 (cci1_i2c0) → msm_csiphy2 → msm_csid0 → msm_vfe0_rdi0 → /dev/video0`

`scripts/capture.sh` resets links, enables `csiphy2→csid0→vfe0_rdi0`, propagates
`Y10_1X10/1280x800` across every pad, sets `/dev/video0` to `Y10P`, and streams.
A frame is `1280*800*10/8 = 1,280,000` bytes.

Quick view: `y10p_to_pgm.py` unpacks the upper 8 bits to a greyscale **PGM**
(no numpy/PIL needed). For PNG, `sudo apt install imagemagick` then
`convert x.pgm x.png`.

---

## Files

| Path | Purpose |
|------|---------|
| `module/ov9282.c`, `module/Makefile` | out-of-tree OV9281/OV9282 driver (v6.18 source) |
| `overlay/qcs6490-radxa-dragon-q6a-ov9281-active.dtso` | **the working overlay** (cci1_i2c0/csiphy2, gpio77+78, virtual clock) |
| `overlay/...-cam1-ov9281.dtso` | CAM1 variant (real `CAM_CC_MCLK0`) — for a module that needs host MCLK |
| `overlay/...-cam2/cam3-ov9281.dtso` | earlier per-label variants (kept for reference) |
| `scripts/deploy_efi_dtb.sh` / `revert_efi_dtb.sh` | merge overlay into the EFI boot DTB / restore |
| `scripts/capture.sh` | configure CAMSS pipeline + capture frames |
| `scripts/y10p_to_pgm.py` | raw Y10P → PGM preview |
| `scripts/install.sh` | **deprecated** (U-Boot path; does not work on this image) |

---

## Dead-ends (don't repeat these)

1. **U-Boot `fdtoverlays` / `fdt` in extlinux.conf** — ignored; board uses
   systemd-boot. Wasted two reboots. Edit the EFI DTB instead.
2. **Routing a `camcc` MCLK / moving to CAM1** — unnecessary; the module
   self-clocks (no MCLK pin on the connector).
3. **Trusting `i2c-18`/`i2c-20` numbers across boots** — they swap; map via
   `of_node`.
4. **Assuming CAM2 label ⇒ gpio77 power-enable** — this connector needed
   **gpio78**; the sensor stayed unpowered until gpio78 was driven.

---

## Notes / next steps

- The module is built for kernel `6.18.2-4-qcom`. To survive kernel upgrades,
  package it with DKMS (the `.c` + `Makefile` are DKMS-ready). A kernel update
  also regenerates the EFI boot DTB, so `deploy_efi_dtb.sh` must be re-run.
- For a friendlier API, libcamera's `simple` pipeline + soft-ISP can be layered
  on; for a mono sensor most uses are fine with raw V4L2 as above.
