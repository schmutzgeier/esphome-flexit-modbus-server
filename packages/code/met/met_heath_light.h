// Light on the heath at night, worked out on the panel as a field over the picture, added to it: the climber's
// headlamp, the glow of the lit window, or the moon's path on the bay, whichever matters most, one at a time. They
// share one slot, one place and a few bytes, see heath_light_room(). Called from "Life on the heath".
#pragma once
#include "met_art.h"
#include "met_heath_live.h"
#include "met_live.h"

namespace flexit::panel::met::live {

enum HeathLight : uint8_t { HL_NONE, HL_LAMP, HL_WINDOW, HL_MOON };

// It's dark enough for light to show
inline bool heath_night(const Scene &s) { return s.at.dark || s.at.light < 0.4f; }

// The figure on the mountain, the headlamp it has as drawn, if it's out in the dark with it on
inline const art::Figure *heath_lamp_figure(const Scene &s) {
  if (!s.heath.lamp || !heath_night(s))
    return nullptr;
  HeathKind k = heath_mountain(s);
  const art::Figure *f = k == HK_CLIMB ? &art::CLIMBER
                         : k == HK_RUN ? &art::RUNNER
                         : k == HK_SKI ? &art::SKIER_CLIMBER
                                       : nullptr;
  return f && f->lamp_x >= 0 ? f : nullptr;
}

// The moon's path on the bay: the columns of open water, from where the bay starts to its last row, under the moon
inline bool heath_moon_path(const Scene &s, int &x0, int &x1, int &y1) {
  constexpr int TOP = 159, HALF = 9;
  if (!s.moon || !s.at.dark || s.shine_x < 0 || s.at.cover > 85 || s.at.fog > 60)
    return false;
  x0 = std::max(0, (int) s.shine_x - HALF);
  x1 = std::min(W, (int) s.shine_x + HALF);
  y1 = H - 20;
  for (int x = x0; x < x1; x++)
    for (int k = 0; k < 3; k++)  // where the land starts under the bay's far edge, the ridge above it is behind
      if (s.land->hill[k][x] >= TOP)
        y1 = std::min<int>(y1, s.land->hill[k][x]);
  return y1 - TOP >= 12;
}

// The light to show: the headlamp, else the window, else the moon's path
inline HeathLight heath_light_of(const Scene &s) {
  int a, b, c;
  if (heath_lamp_figure(s))
    return HL_LAMP;
  if (s.window_lit && heath_night(s))
    return HL_WINDOW;
  return heath_moon_path(s, a, b, c) ? HL_MOON : HL_NONE;
}

// What the light takes of the panel, only at night: a field's slot and place, a script and a ramp of colours
inline Room heath_light_room(const Scene &s) {
  return s.ground == G_HEATH && heath_light_of(s) != HL_NONE ? Room{1, 1, 200} : Room{};
}
// What it's made of, for "Life on the heath" to start again when it changes
inline uint32_t heath_light_cast(const Scene &s) {
  HeathLight l = heath_light_of(s);
  return (uint32_t) l | (l == HL_MOON ? (uint32_t) (s.shine_x / 12) << 2 : 0);
}

inline void heath_lights_start(const Scene &s) {
  switch (heath_light_of(s)) {
    case HL_LAMP: {  // on the climber, as a cone up the slope, round the head when it runs or skis both ways
      const art::Figure &f = *heath_lamp_figure(s);
      HeathKind k = heath_mountain(s);
      HeathSprite sp = heath_sprite(s, k);
      // the copy's x and y are its top left, as are x(w_hmount) and y(w_hmount): the cone's point on the lamp
      bool cone = k == HK_CLIMB;
      int w = cone ? 30 : 22, h = cone ? 18 : 16;
      int lx = f.lamp_x - sp.x - (cone ? 0 : w / 2), ly = f.lamp_y - sp.y - h / 2;
      if (cone)
        add_field("hlight", w, h, OVERLAY_FIELD_CONE, 12, 0,
                  light(0xFFF0C0, [](float v) { return 0.4f * (2 * v - v * v) * (2 * v - v * v); }));
      else
        add_field("hlight", w, h, OVERLAY_FIELD_RADIAL, 0, 0, light(0xFFF0C0, [](float v) { return 0.4f * v * v; }));
      place("hlight", 0, sp.x + lx, sp.y + ly,
            "blend add\nsoft 0, 0, 3, 3\nx := x(w_hmount) + " + std::to_string(lx) + "\ny := y(w_hmount) + " +
                std::to_string(ly) + "\nlevel := 85 + noise(ms / 600, 8, 15)");
      break;
    }
    case HL_WINDOW: {  // a warm glow round the window, flickering as the fire does
      add_field("hlight", 44, 34, OVERLAY_FIELD_RADIAL, 0, 0, light(0xFFB45A, [](float v) { return 0.5f * v * v; }));
      place("hlight", 0, window_x(s) + 7 - 22, window_y(s) + 6 - 17,
            "blend add\nlevel := 70 + noise(ms / 130, 6, 30)\nsize_x := 96 + noise(ms / 400 + 9, 11, 8)");
      break;
    }
    case HL_MOON: {  // a column of light down the bay, from the horizon, brightest there, and glittering
      int x0 = 0, x1 = 0, y1 = 0;
      heath_moon_path(s, x0, x1, y1);
      int h = y1 - 159;
      add_field("hlight", x1 - x0, h, OVERLAY_FIELD_LINEAR, OVERLAY_LINEAR_DOWN, 0,
                light(0xCFDFFF, [](float v) { return 0.45f * (1 - v) * (1 - v); }));
      place("hlight", 0, x0, 159,
            "blend add\nsoft 4, 4, 0, 6\nlevel := 75 + noise(ms / 350, 9, 40)\nsize_x := 75 + noise(ms / 700 + 5, 15, "
            "40)");
      break;
    }
    default:
      break;
  }
}

}  // namespace flexit::panel::met::live
