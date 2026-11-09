// The heath, Ground G_HEATH: the cabin on the heath under the mountain from the SVG layers in scripts/met_art
// (met_art.h), with the bay by the Beaufort scale and Oceanforecast's waves, the people, the dog, the birch through
// the year, a reindeer and a walrus. met_scene.cpp and met_raster.cpp hand it their parts of the scene where it's
// the heath. Plain C++ without ESPHome, see met_scene.h.
#pragma once
#include "met_font.h"
#include "met_scene.h"

namespace flexit::panel::met {

// met_scene.cpp's stamps, for the heath's things
Stamp *stamp(Scene &s, int x, int y, int w, int h, StampLayer layer, uint32_t c);
void stamp_rows(Stamp *p, const uint32_t *rows);
void blob(Scene &s, int cx, int cy, int rx, int ry, StampLayer layer, uint32_t c);
void box(Scene &s, int x, int y, int w, int h, StampLayer layer, uint32_t c);
void flag_norway(Scene &s, int x, int y, int dir, uint32_t red, uint32_t white, uint32_t blue);
// Text in a font of met_font.h, y the top of its line
void ov_text(Scene &s, int x, int y, const char *t, const Font &f);
int ov_text_width(const char *t, const Font &f);

// ---- for build()

// ground() for the heath, seed the place's: the land for the animations, the landscape's colours for the light,
// the season and the weather, the bay, the fog, the moon's maria, and where the cabin's things are
void heath_ground(Scene &s, const Now &n, uint32_t seed);
// The rest of life() for the heath, after the birds and bats: the chimney's smoke, the flag, the people and the
// animals
void heath_life(Scene &s, const Now &n, Rng &rng);
// holidays() for the heath
void heath_holidays(Scene &s, const Now &n, Rng &rng);
// After overlay(): its outline in the heath's darkest colour, and the wave height in the bay
void heath_overlay(Scene &s, const Now &n);

// ---- for row() and palette()

// The artboard's moon with maria, a glow and a halo, in the sky's row y
void heath_moon_row(const Scene &s, int y, uint8_t *out);
// The heath's row y over the sky, the clouds and the stamps in it: the wind's streaks, the landscape with
// the bay and the fog, the smoke, the flagpole and the people
void heath_row(const Scene &s, int y, uint8_t *out);
// The heath's colour slots row() may draw
void heath_drawn(const Scene &s, Colours &u);
// The heath's land as the picture's parts, farthest first: the mountain, the bay, the far land and the near ground,
// each in front of the animations farther off than it, then the cabin with the spruce and the birch, and the pole on
// flag days, in front of what walks on the near ground. Each its box and its slots. How many, at most HEATH_PARTS.
struct HeathPart {
  int16_t x0, y0, x1, y1;  // x1 and y1 just past it
  Colours slots;
  uint8_t depth;  // a Depth
};
inline constexpr int HEATH_PARTS = 6;
int heath_parts(const Scene &s, HeathPart *out);
// The open water in the bay for what swims or sails on it: columns l to r along row y, open from 8 rows above it to
// 2 under, the nearest such row with at least min px of it above the wave height. False if there's none.
bool heath_open_water(int min, int &l, int &r, int &y);

// ---- for met.h

// A storm, heavy rain and the forecast can be too much for the panel's memory as 16 colours. Then the rain thins
// out and the bay calms down, step by step, with palette() made again each time. prev as palette()'s.
void heath_fit(Scene &s, const uint32_t *prev);

}  // namespace flexit::panel::met
