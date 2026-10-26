// The heath, Ground G_HEATH: the cabin on the heath under the mountain from the SVG layers in scripts/met_art
// (met_art.h), with the bay by the Beaufort scale and Oceanforecast's waves, the people, the dog, the birch through
// the year, a reindeer and a walrus. met_scene.cpp and met_raster.cpp hand it their parts of the scene where it's
// the heath. Plain C++ without ESPHome, see met_scene.h.
#pragma once
#include "met_scene.h"

namespace flexit::panel::met {

// met_scene.cpp's, for the heath's things
Sprite *sprite(Scene &s, int x, int y, int w, int h, int layer, uint32_t c);
void sprite_rows(Sprite *p, const uint32_t *rows);
Sprite *blob(Scene &s, int cx, int cy, int rx, int ry, int layer, uint32_t c);
Sprite *box(Scene &s, int x, int y, int w, int h, int layer, uint32_t c);
void flag_norway(Scene &s, int x, int y, int dir, uint32_t red, uint32_t white, uint32_t blue);
void ov_text(Scene &s, int x, int y, const char *t, int k);

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
// The heath's row y over the sky, the clouds and the sprites of layer 0: the wind's streaks, the landscape with
// the bay and the fog, the smoke, the flagpole and the people
void heath_row(const Scene &s, int y, uint8_t *out);
// The heath's colour slots row() may draw
void heath_drawn(const Scene &s, Colours &u);

// ---- for met.h

// A storm, heavy rain and the forecast can be too much for the panel's memory as 16 colours. Then the rain thins
// out and the bay calms down, step by step, with palette() made again each time. prev as palette()'s.
void heath_fit(Scene &s, const uint32_t *prev);

}  // namespace flexit::panel::met
