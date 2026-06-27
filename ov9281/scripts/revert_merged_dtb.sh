#!/usr/bin/env bash
# Revert deploy_merged_dtb.sh: stop loading the merged dtb, restore stock boot.
set -uo pipefail
MERGED_DST="/boot/qcs6490-radxa-dragon-q6a-ov9281.dtb"

# Clear U_BOOT_FDT (restore backup if present)
if [ -f /etc/default/u-boot.bak.ov9281 ]; then
	sudo cp /etc/default/u-boot.bak.ov9281 /etc/default/u-boot
else
	sudo sed -i 's|^U_BOOT_FDT=.*|#U_BOOT_FDT=""|' /etc/default/u-boot
fi
sudo rm -f "$MERGED_DST"
sudo u-boot-update
echo ">>> Reverted to stock boot config. Reboot to apply."
echo ">>> (Overlay file left disabled in /boot/dtbo/; module install untouched.)"
