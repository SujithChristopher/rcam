#!/usr/bin/env bash
# THE correct path for this board: it boots via systemd-boot (UEFI), which loads
# the DTB named on the loader entry's `devicetree` line from the EFI partition:
#   /boot/efi/RadxaOS/<ver>/qcs6490-radxa-dragon-q6a.dtb
# extlinux/u-boot is vestigial here and ignored. We merge the OV9281 overlay
# directly into that boot DTB (backup kept). Reversible via revert_efi_dtb.sh.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KVER="$(uname -r)"
EFIDTB="/boot/efi/RadxaOS/$KVER/qcs6490-radxa-dragon-q6a.dtb"
DTBO="$HERE/overlay/qcs6490-radxa-dragon-q6a-ov9281-active.dtbo"  # verified working wiring

[ -f "$EFIDTB" ] || { echo "!! boot dtb not found: $EFIDTB" >&2; exit 1; }

echo ">>> Backing up boot DTB (plain cp: EFI is vfat, no ownership)"
[ -f "$EFIDTB.orig" ] || cp "$EFIDTB" "$EFIDTB.orig"   # pristine, keep forever
cp "$EFIDTB" "$EFIDTB.bak"                             # rolling backup

echo ">>> Merging overlay into the pristine boot DTB"
fdtoverlay -i "$EFIDTB.orig" -o /tmp/ov9281-efi-merged.dtb "$DTBO"

echo ">>> Verifying merge (sensor node present)"
dtc -I dtb -O dts /tmp/ov9281-efi-merged.dtb 2>/dev/null | grep -q 'ovti,ov9281' || {
	echo "!! merge missing sensor node, aborting" >&2; exit 2; }

cp /tmp/ov9281-efi-merged.dtb "$EFIDTB"
sync
echo ">>> Done. Boot DTB now includes the OV9281. Reboot to apply."
echo ">>> (Pristine original preserved at $EFIDTB.orig)"
