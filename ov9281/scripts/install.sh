#!/usr/bin/env bash
# Install the OV9281 driver + device-tree overlay on the Radxa Dragon Q6A.
# Reversible: see uninstall.sh. Requires a reboot to take effect.
set -euo pipefail

KVER="$(uname -r)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DTBO="qcs6490-radxa-dragon-q6a-cam2-ov9281.dtbo"   # change to cam3 variant if needed

echo ">>> 1. Building + installing ov9282 module"
make -C "$HERE/module" >/dev/null
sudo install -D -m 0644 "$HERE/module/ov9282.ko" "/lib/modules/$KVER/updates/ov9282.ko"
sudo depmod -a "$KVER"

echo ">>> 2. Deploying overlay to /boot/dtbo/ (enabled)"
sudo install -D -m 0644 "$HERE/overlay/$DTBO" "/boot/dtbo/$DTBO"
# Register in managed.list so rsetup is aware of it (idempotent)
if ! grep -qx "$DTBO" /boot/dtbo/managed.list 2>/dev/null; then
	echo "$DTBO" | sudo tee -a /boot/dtbo/managed.list >/dev/null
fi

echo ">>> 3. Regenerating boot config (u-boot-update)"
sudo u-boot-update

echo ">>> Done. Verify the fdtoverlays line was added:"
grep -n fdtoverlays /boot/extlinux/extlinux.conf || echo "  (no fdtoverlays line - check U_BOOT_FDT_OVERLAYS_DIR)"
echo ">>> Reboot, then run scripts/bringup.sh"
