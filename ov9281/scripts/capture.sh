#!/usr/bin/env bash
# Configure the CAMSS pipeline for the OV9281 and capture frames.
# Verified working on the Dragon Q6A: ov9281 (cci1_i2c0) -> csiphy2 -> csid0
# -> vfe0_rdi0 -> /dev/video0, Y10 mono 1280x800.
set -uo pipefail

M=${M:-/dev/media0}
SENS=${SENS:-"ov9281 18-0060"}      # NOTE: i2c bus number can change across boots;
                                     # check: media-ctl -d $M -p | grep ov9281
W=${W:-1280}; H=${H:-800}
VID=${VID:-/dev/video0}
N=${N:-30}
OUT=${OUT:-captures/ov9281_${W}x${H}_y10p.raw}
mkdir -p "$(dirname "$OUT")"

echo ">>> Reset + link ov9281 -> csiphy2 -> csid0 -> vfe0_rdi0"
media-ctl -d "$M" -r
media-ctl -d "$M" -l "'msm_csiphy2':1 -> 'msm_csid0':0 [1]"
media-ctl -d "$M" -l "'msm_csid0':1 -> 'msm_vfe0_rdi0':0 [1]"

echo ">>> Propagate Y10_1X10/${W}x${H} across the pipe"
for PAD in "'$SENS':0" "'msm_csiphy2':0" "'msm_csid0':0" "'msm_vfe0_rdi0':0"; do
	media-ctl -d "$M" -V "$PAD [fmt:Y10_1X10/${W}x${H}]"
done

echo ">>> Capture $N frames (Y10P) -> $OUT"
v4l2-ctl -d "$VID" -v width="$W",height="$H",pixelformat=Y10P
v4l2-ctl -d "$VID" --stream-mmap --stream-count="$N" --stream-to="$OUT"

echo ">>> Wrote $(stat -c%s "$OUT") bytes ($((W*H*10/8)) per frame)"
echo ">>> Make an 8-bit preview PGM:  python3 scripts/y10p_to_pgm.py $OUT $W $H"
