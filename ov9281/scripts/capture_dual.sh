#!/usr/bin/env bash
# Configure CAMSS for BOTH OV9281 cameras and capture from each.
# Verified on the Dragon Q6A with the dual-ov9281 overlay merged into the EFI DTB:
#   CAM2: ov9281 (cci1_i2c0) -> csiphy2 -> csid0 -> vfe0_rdi0 -> /dev/video0
#   CAM3: ov9281 (cci1_i2c1) -> csiphy3 -> csid1 -> vfe1_rdi0 -> /dev/video3
# Both sensors stream Y10 mono 1280x800.
#
# NOTE: the ov9282 driver is out-of-tree and not auto-loaded unless you install
# /etc/modules-load.d/ov9282.conf (see install.sh). Run: sudo modprobe ov9282
# i2c bus numbers (18/20) can change across boots; the sensor *names* are stable:
#   media-ctl -d /dev/media0 -p | grep ov9281
set -uo pipefail

M=${M:-/dev/media0}
W=${W:-1280}; H=${H:-800}
N=${N:-30}
SENS2=${SENS2:-"$(media-ctl -d "$M" -p 2>/dev/null | grep -oE 'ov9281 [0-9]+-0060' | sort -u | sed -n 1p)"}
SENS3=${SENS3:-"$(media-ctl -d "$M" -p 2>/dev/null | grep -oE 'ov9281 [0-9]+-0060' | sort -u | sed -n 2p)"}
OUTDIR=${OUTDIR:-captures}
mkdir -p "$OUTDIR"

echo ">>> CAM2 sensor: ${SENS2:-<none>}   CAM3 sensor: ${SENS3:-<none>}"
[ -n "$SENS2" ] && [ -n "$SENS3" ] || { echo "!! need two ov9281 subdevs; is ov9282 loaded?" >&2; exit 1; }

echo ">>> reset + build both pipelines"
media-ctl -d "$M" -r
media-ctl -d "$M" -l "'msm_csiphy2':1 -> 'msm_csid0':0 [1]"
media-ctl -d "$M" -l "'msm_csid0':1 -> 'msm_vfe0_rdi0':0 [1]"
media-ctl -d "$M" -l "'msm_csiphy3':1 -> 'msm_csid1':0 [1]"
media-ctl -d "$M" -l "'msm_csid1':1 -> 'msm_vfe1_rdi0':0 [1]"

echo ">>> propagate Y10_1X10/${W}x${H} across both pipes"
for PAD in "'$SENS2':0" "'msm_csiphy2':0" "'msm_csid0':0" "'msm_vfe0_rdi0':0" \
           "'$SENS3':0" "'msm_csiphy3':0" "'msm_csid1':0" "'msm_vfe1_rdi0':0"; do
	media-ctl -d "$M" -V "$PAD [fmt:Y10_1X10/${W}x${H}]"
done

for pair in "video0:cam2" "video3:cam3"; do
	VID="/dev/${pair%%:*}"; TAG="${pair##*:}"
	OUT="$OUTDIR/${TAG}_${W}x${H}_y10p.raw"
	echo ">>> capture $N frames from $TAG ($VID) -> $OUT"
	v4l2-ctl -d "$VID" -v width="$W",height="$H",pixelformat=Y10P
	v4l2-ctl -d "$VID" --stream-mmap --stream-count="$N" --stream-to="$OUT"
	echo "    wrote $(stat -c%s "$OUT") bytes ($((W*H*10/8))/frame)"
done
echo ">>> previews:  python3 scripts/y10p_to_pgm.py $OUTDIR/cam2_${W}x${H}_y10p.raw $W $H"
