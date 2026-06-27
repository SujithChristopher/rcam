#!/usr/bin/env bash
# Revert everything install.sh did.
set -uo pipefail
KVER="$(uname -r)"
DTBO="qcs6490-radxa-dragon-q6a-cam2-ov9281.dtbo"

sudo rm -f "/lib/modules/$KVER/updates/ov9282.ko"
sudo depmod -a "$KVER"
sudo rm -f "/boot/dtbo/$DTBO"
sudo sed -i "\|^$DTBO$|d" /boot/dtbo/managed.list 2>/dev/null || true
sudo u-boot-update
echo ">>> Reverted. Reboot to fully disable the overlay."
