#!/usr/bin/env bash
# Post-reboot bring-up + test capture for the OV9281 on CAMSS.
# Discovers the media topology rather than hard-coding entity names, since the
# CAMSS entity naming is kernel/SoC specific.
set -uo pipefail

W=${W:-1280}; H=${H:-800}; CODE=${CODE:-Y10}   # sensor mbus: Y10_1X10
PIXFMT=${PIXFMT:-Y10}                           # v4l2 capture pixelformat
OUT=${OUT:-/tmp/ov9281_frame.raw}

echo ">>> Loading modules"
sudo modprobe qcom-camss 2>/dev/null
sudo modprobe ov9282 2>/dev/null

echo ">>> Sensor probe status (dmesg):"
sudo dmesg | grep -iE "ov928|csiphy|csid|vfe|camss" | tail -20

MDEV=$(ls /dev/media* 2>/dev/null | head -1)
if [ -z "$MDEV" ]; then
	echo "!! No /dev/media* node. CAMSS did not instantiate - check overlay/dmesg." >&2
	exit 1
fi
echo ">>> Using $MDEV"

echo ">>> Media topology:"
media-ctl -d "$MDEV" -p

# --- Discover entities -------------------------------------------------------
SENSOR=$(media-ctl -d "$MDEV" -p 2>/dev/null | grep -oE 'ov9281[^ ]*[0-9]+-[0-9a-f]+' | head -1)
[ -z "$SENSOR" ] && SENSOR=$(media-ctl -d "$MDEV" -e "ov9281" 2>/dev/null | head -1)
echo ">>> Sensor entity: ${SENSOR:-NONE}"

if [ -z "$SENSOR" ]; then
	echo "!! Sensor subdev not found in media graph. Stop here and inspect topology above." >&2
	exit 1
fi

cat <<EOF

>>> NEXT (manual, topology-dependent) STEPS:
    The CAMSS graph is sensor -> csiphy -> csid -> vfe -> /dev/videoN (RDI).
    From the topology above, set the same Y${CODE#Y}/${W}x${H} format on every
    pad along the active pipe, enable the links, then capture, e.g.:

      media-ctl -d $MDEV -V '"$SENSOR":0 [fmt:${CODE}_1X10/${W}x${H}]'
      # repeat -V for each csiphy/csid/vfe pad shown above
      # enable links with: media-ctl -d $MDEV -l '"src":pad->"sink":pad[1]'

      v4l2-ctl -d /dev/videoN \\
        --set-fmt-video=width=${W},height=${H},pixelformat=${PIXFMT} \\
        --stream-mmap --stream-count=10 --stream-to=${OUT}

    Then view a frame (Y10 packed -> use ImageMagick/ffmpeg or a quick Python
    unpack). See README.md "Viewing frames".
EOF
