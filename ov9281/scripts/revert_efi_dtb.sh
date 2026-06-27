#!/usr/bin/env bash
# Restore the pristine boot DTB on the EFI partition.
set -euo pipefail
KVER="$(uname -r)"
EFIDTB="/boot/efi/RadxaOS/$KVER/qcs6490-radxa-dragon-q6a.dtb"
if [ -f "$EFIDTB.orig" ]; then
	cp -a "$EFIDTB.orig" "$EFIDTB"; sync
	echo ">>> Restored pristine boot DTB. Reboot to apply."
else
	echo "!! No $EFIDTB.orig backup found." >&2; exit 1
fi
