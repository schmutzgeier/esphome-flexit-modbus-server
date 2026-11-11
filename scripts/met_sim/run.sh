#!/bin/sh
# Renders the weather picture of packages/code/met for mocked weathers, with the wave heights, the forecast's marks
# and the animations, on a PC. packages/code/met is built as it is, next to overlay_mock.h in place of the panel's
# overlay.h, and marks.h with its font draws the forecast over the picture.
# Writes to build/met_sim: a PNG and an animated GIF for each case, and stills.png with all of them.
#   scripts/met_sim/run.sh [met_sim's options, like --only heath --seconds 20]
# Needs g++ and python3 with Pillow.
set -e
root=$(cd "$(dirname "$0")/../.." && pwd)
build=$root/build/met_sim
src=$build/src
rm -rf "$src" "$build"/out
mkdir -p "$src/met" "$src/panel" "$build/out"
cp "$root"/packages/code/met/*.h "$root"/packages/code/met/*.cpp "$src/met/"
cp "$root"/packages/code/panel/*.h "$src/panel/"  # picture_plan.h for the picture's parts, marks.h, and what they need
cp "$root/packages/code/panel/marks_font.cpp" "$src/panel/"
cp "$root/scripts/met_sim/overlay_mock.h" "$src/panel/overlay.h"
# the files of the heath, if this tree has them
heath=
[ -f "$src/met/met_heath.cpp" ] && heath=$src/met/met_heath.cpp
# the FreeRTOS that overlay_script.h asks the stack's start of
g++ -std=gnu++20 -O2 -Wall -Wextra -Wno-unused-parameter -I"$root/scripts/met_sim/stub" -I"$src/met" \
  "$root/scripts/met_sim/met_sim.cpp" \
  "$src"/met/met_scene.cpp "$src"/met/met_raster.cpp $heath "$src"/met/met_forecast.cpp \
  "$src"/met/met_sky_math.cpp "$src"/met/met_sprites.cpp "$src"/panel/marks_font.cpp -o "$build/met_sim"
"$build/met_sim" "$build/out" "$@"
python3 "$root/scripts/met_sim/render.py" "$build/out"
