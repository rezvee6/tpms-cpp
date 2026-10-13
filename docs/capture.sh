#!/usr/bin/env bash
# Regenerates the README screenshots and GIF from scripted demo scenes.
# Needs: Google Chrome, ImageMagick, ffmpeg, and the demo served on :8765
#   python3 -m http.server 8765 --directory web
set -euo pipefail
cd "$(dirname "$0")/img"

CHROME="${CHROME:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"
BASE="${BASE:-http://localhost:8765/}"

shot() {  # shot <out.png> <width> <height> <query>
  "$CHROME" --headless=new --hide-scrollbars --force-device-scale-factor=2 \
    --window-size="$2,$3" --virtual-time-budget=6000 --screenshot="$1" "${BASE}?$4" 2>/dev/null
}

# Stills.
shot hero.png        1440 1000 'seed=1&t=19&ev=12:FL:p0.7&pause=1'
shot all-four.png    1440 1000 'seed=11&t=18&ev=11:ALL:p0.7&pause=1&sel=1'
shot calibrating.png 1440 1000 'seed=2&t=2.5&pause=1'
shot slow-leak.png   1440 1000 'seed=5&scenario=3&t=27&ev=12:RL:l0.7&pause=1&sel=2'
shot cats-eyes.png   1440 1000 'seed=4&cats=2&t=16&ev=10:RL:p0.72&pause=1&sel=2'
shot mobile.png       390 1700 'seed=1&t=19&ev=12:FL:p0.7&pause=1'

# Close-up of the resonance panel during a puncture (2x pixel coordinates).
shot tmp.png 1440 1000 'seed=1&t=19&ev=12:FL:p0.7&pause=1'
magick tmp.png -crop 780x700+2078+275 +repage spectra.png

# Animated puncture: one scripted frame per 15 simulated seconds.
rm -rf frames && mkdir frames
i=0
for t in $(seq 11.5 0.25 19); do
  shot "frames/$(printf %03d $i).png" 1440 780 "seed=1&t=$t&ev=12:FL:l1.75&pause=1"
  i=$((i + 1))
done
ffmpeg -loglevel error -y -framerate 6 -i frames/%03d.png \
  -vf "scale=1100:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
  puncture.gif
rm -rf frames tmp.png

# Downscale stills to a sane README size.
for f in hero all-four calibrating slow-leak cats-eyes spectra; do
  magick "$f.png" -resize '1600x>' -strip "$f.png"
done
magick mobile.png -resize '600x>' -strip mobile.png
ls -la
