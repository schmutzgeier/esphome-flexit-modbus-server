// The wind and the waves on the heath, moving on the panel instead of drawn in the picture (L_HEATH_SEA): the wind's
// streaks in the air above the bay, and the crests rolling over the bay. One field of noise is both, two copies of
// it each with its own script, so they take one slot and two places. The picture is complete without them, see
// heath_drawn() and sea_px().
//   the streaks: a copy stretched four times across, so the noise's peaks are dashes along the wind. Its level, how
//     much of the field shows, swings between the wind and its gusts, and it scrolls the way the wind blows, the
//     faster the harder it blows.
//   the crests: a copy over the bay, stretched across by the swell, taller by the height of the waves. Its rows
//     brighten and fade one after another, rolling toward the shore, and it creeps along the wind. More shows the
//     rougher the sea, and the whole of it heaves a pixel.
#pragma once
#include "met_heath.h"
#include "met_live.h"

namespace flexit::panel::met::live {

// What it takes of the panel: the field's slot and its two copies, a script each (about 24 bytes and a register
// each and its text), and the field's 4 bytes
inline constexpr Room HEATH_SEA = {1, 2, 300};
inline constexpr int SEA_TOP = HEATH_SEA_TOP, STREAK_TOP = 98, STREAK_W = 96, STREAK_H = 55;

inline bool heath_sea_fits(const Scene &s) {
  return s.ground == G_HEATH && (s.heath.nstreaks > 0 || s.heath.look.crest > 0);
}

// The tallest of the bay's waves, px
inline int heath_wave_px(const Scene &s) {
  int a = 0;
  for (int i = 0; i < s.heath.nsea; i++)
    a = std::max<int>(a, s.heath.sea_a[i]);
  return a;
}

// The script of the streaks' copy
inline std::string heath_streaks(const Scene &s) {
  const Heath &h = s.heath;
  float wind = s.at.wind, gust = std::max(s.at.gust, wind * 1.3f);
  int sx = 400, sy = 45, lo = 76 + 2 * h.nstreaks - 6 * (h.bft >= 7),
      hi = lo + 8 + (int) std::min(12.0f, (gust - wind) * 2);
  float v = h.wind_dir * (4 + 1.6f * wind) / (sx / 100.0f);
  return "glide w " + num(STREAK_W / (sx / 100)) + "; glide h " + num(STREAK_H * 100 / sy) + "; glide size_x " +
         num(sx) + "; glide size_y " + num(sy) + "\npivot 0, 0\nblend screen\nscroll " + overlay::decimal(v) +
         ", 0\nlevel := " + num(lo) + " + noise(ms, 5000, " + num(hi - lo) +
         ")\ny += sin(ms / 40, 2)\nsoft 3, 3, 4, 4\n";
}

// The crests' shape, so they keep to the water: runs of box columns, each cut to the bay's rows in them, joined while
// there are more than RUNS, showing only what both would. bw gets the box's width. sx and sy are the box's size in
// percent, whose rows start at SEA_TOP.
inline std::string heath_sea_shape(int sx, int sy, int &bw) {
  constexpr size_t RUNS = 8;
  uint8_t top[W], bottom[W];
  heath_water(top, bottom);
  int right = 0;
  for (int x = 0; x < W; x++)
    if (bottom[x])
      right = x + 1;
  bw = (right * 100 + sx - 1) / sx;
  struct Run {
    int w, top, bottom;
  };
  std::vector<Run> r;
  for (int c = 0; c < bw; c++) {
    int a = H, b = 0;
    for (int x = c * sx / 100; x < std::min(W, (c + 1) * sx / 100); x++)
      if (bottom[x]) {
        a = std::min<int>(a, top[x]);
        b = std::max<int>(b, bottom[x]);
      }
    a = b ? std::max(0, (a - SEA_TOP) * 100 / sy) : 0;
    b = b ? (b - SEA_TOP) * 100 / sy + 1 : 0;
    if (!r.empty() && r.back().top == a && r.back().bottom == b)
      r.back().w++;
    else
      r.push_back({1, a, b});
  }
  auto apart = [&](size_t i) { return abs(r[i].top - r[i + 1].top) + abs(r[i].bottom - r[i + 1].bottom); };
  while (r.size() > RUNS) {
    size_t k = 0;
    for (size_t i = 1; i + 1 < r.size(); i++)
      if (apart(i) < apart(k))
        k = i;
    int a = std::max(r[k].top, r[k + 1].top);
    r[k] = {r[k].w + r[k + 1].w, a, std::max(a, std::min(r[k].bottom, r[k + 1].bottom))};
    r.erase(r.begin() + (long) k + 1);
  }
  std::string text = "shape ";
  for (size_t i = 0; i < r.size(); i++)
    text += (i ? ", " : "") + num(r[i].w) + ", " + num(r[i].top) + ", " + num(r[i].bottom);
  return text;
}

// The script of the crests' copy
inline std::string heath_crests(const Scene &s) {
  const Heath &h = s.heath;
  int a = heath_wave_px(s), sx = 300 + (int) (100 * h.swell), sy = std::clamp(35 + 10 * a, 35, 100), bw;
  std::string shape = heath_sea_shape(sx, sy, bw);
  float vx = h.wind_dir * (0.3f + 0.25f * h.bft), vy = 0.15f + 0.04f * h.bft;
  int base = 40 + std::min(30, a * 6) + 3 * std::max(0, h.bft - 5), level = 85 + h.look.crest / 10 + h.look.cap / 10;
  int roll = std::clamp(9 - h.bft, 3, 8);  // ms per degree of the rows' brightening, the harder the quicker
  int heave = 1500 + 300 * (10 - std::min(h.bft, 9));
  return "glide w " + num(bw) + "; glide h " + num((H - SEA_TOP) * 100 / sy + 1) + "; glide size_x " + num(sx) +
         "; glide size_y " + num(sy) + "; glide level " + num(level) + "\npivot 0, 0\nblend screen\nscroll " +
         overlay::decimal(vx) + ", " + overlay::decimal(vy) + "\nrow.opacity := " + num(base) +
         " + sin(row * 24 - ms / " + num(roll) + ", 30)\nwave y -1, 1, " + num(heave) + ", " + num(heave) +
         " jitter 20 slow both\n" + shape + "\n";
}

// The field's colours: clear but for a band of light, the crest's line round the noise's peaks, and a spot of foam on
// the highest.
inline void heath_motion_start(const Scene &s) {
  const Heath &h = s.heath;
  uint32_t col = h.nstreaks > 0 && h.bft < 6 ? s.rgb[C_STREAK] : s.rgb[C_FOAM];  // spray is white
  // foam on the highest peaks, from lower down and whiter the harder it blows
  float foam = std::clamp(0.95f - 0.04f * (h.bft - 5), 0.72f, 0.95f), line = h.bft >= 6 ? 0.75f : 0.6f;
  float top = h.bft >= 7 ? 0.9f : line;
  auto colours = light(col, [=](float v) {
    return v > foam ? top : v > 0.55f && v < 0.8f ? line - 4 * fabsf(v - 0.667f) : 0.0f;
  });
  add_field("hsea", std::max(STREAK_W / 4, W / 3), (H - SEA_TOP) * 3, OVERLAY_FIELD_NOISE, 1, 5, colours);
  if (h.look.crest > 0)
    place("hsea", 0, 0, SEA_TOP, heath_crests(s), {}, {.fade = 1500, .depth = (uint8_t) (D_SEA + 1)});
  if (h.nstreaks > 0)
    place("hsea", 1, 0, STREAK_TOP, heath_streaks(s), {}, {.fade = 1500, .depth = D_SKY});
}

// The scene changed under it, as the wind turns or the hour has new waves: its scripts again
inline void heath_motion_redo(const Scene &s) {
  const Heath &h = s.heath;
  if (h.look.crest > 0)
    script("hsea", 0, heath_crests(s));
  if (h.nstreaks > 0)
    script("hsea", 1, heath_streaks(s));
}

// What it is made of, so it starts again when that changes: which copies it has
inline uint32_t heath_motion_made_of(const Scene &s) {
  return (uint32_t) (s.heath.look.crest > 0) | (uint32_t) (s.heath.nstreaks > 0) << 1;
}

}  // namespace flexit::panel::met::live
