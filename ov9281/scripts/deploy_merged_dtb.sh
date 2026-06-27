#!/usr/bin/env bash
# Make U-Boot boot a PRE-MERGED dtb (base + OV9281 overlay), bypassing U-Boot's
# overlay engine entirely (its control FDT lacks __symbols__, so fdtoverlays
# can't resolve our fixups).
#
# Reversible with revert_merged_dtb.sh. Keeps a backup of the boot config.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KVER="$(uname -r)"
BASE="/usr/lib/linux-image-$KVER/qcom/qcs6490-radxa-dragon-q6a.dtb"
DTBO="$HERE/overlay/qcs6490-radxa-dragon-q6a-cam2-ov9281.dtbo"
MERGED_DST="/boot/qcs6490-radxa-dragon-q6a-ov9281.dtb"

echo ">>> 1. Re-merge overlay into base -> $MERGED_DST"
fdtoverlay -i "$BASE" -o /tmp/ov9281-merged.dtb "$DTBO"
sudo install -m 0644 /tmp/ov9281-merged.dtb "$MERGED_DST"

echo ">>> 2. Disable the standalone overlay so u-boot-update emits NO fdtoverlays line"
if [ -e "/boot/dtbo/$(basename "$DTBO")" ]; then
	sudo mv "/boot/dtbo/$(basename "$DTBO")" "/boot/dtbo/$(basename "$DTBO").disabled"
fi
sudo sed -i "\|^$(basename "$DTBO")$|d" /boot/dtbo/managed.list 2>/dev/null || true

echo ">>> 3. Point U-Boot at the merged dtb (backup first)"
sudo cp -n /etc/default/u-boot /etc/default/u-boot.bak.ov9281 || true
# set or replace U_BOOT_FDT
if grep -qE '^\s*#?\s*U_BOOT_FDT=' /etc/default/u-boot; then
	sudo sed -i "s|^\s*#\?\s*U_BOOT_FDT=.*|U_BOOT_FDT=\"$MERGED_DST\"|" /etc/default/u-boot
else
	echo "U_BOOT_FDT=\"$MERGED_DST\"" | sudo tee -a /etc/default/u-boot >/dev/null
fi

echo ">>> 4. Regenerate boot config"
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak.ov9281
sudo u-boot-update

echo ">>> Result — expect an 'fdt $MERGED_DST' line and NO 'fdtoverlays':"
grep -nE "fdt|fdtoverlays" /boot/extlinux/extlinux.conf
echo ">>> If that looks right, reboot. Recovery notes in README (serial console)."
