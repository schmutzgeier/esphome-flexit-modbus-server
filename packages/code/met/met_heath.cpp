// The heath, see met_heath.h
#include "met_heath.h"
#include <cstdio>
#include <cstring>
#include "met_art.h"
#include "met_heath_live.h"
#include "met_raster.h"

namespace flexit::panel::met {

static_assert(art::SLOTS == ART_SLOTS, "a colour slot for each of met_art.h's");

static constexpr int HZ = 157;                   // the bay's far edge on the artboard
static constexpr int POLE_X = 276;               // the flagpole on flag days, by the cabin
static constexpr int MOON_HALO_R = 19;           // the artboard moon's halo and glow
static constexpr int MOON_GLOW_HR = 13;          // (MOON_R is the same as the other landscapes')
static constexpr int HOME_X = art::DOOR_X - 40;  // the hill in front of the cabin
// What the panel holds for a picture, a little under PIC_POOL, packed as picture.h packs it
static constexpr size_t PACK_MAX = 13824;

// ---------------------------------------------------------------- the sea

// The Beaufort scale as in MET Norway's wind and wave table: each force's top wind speed in m/s, and the height of a
// fully grown sea at it in m. A bay's waves are lower, but the picture goes by the table when Oceanforecast has none.
static constexpr float BFT_WIND[12] = {0.2f, 1.5f, 3.3f, 5.4f, 7.9f, 10.7f, 13.8f, 17.1f, 20.7f, 24.4f, 28.4f, 32.6f};
static constexpr float BFT_WAVE[13] = {0, 0.1f, 0.2f, 0.6f, 1, 2, 3, 4, 5.5f, 7, 9, 11.5f, 14};
static int beaufort(float wind) {
  float v = roundf(wind * 10) / 10;  // the table goes in tenths
  int b = 0;
  while (b < 12 && v > BFT_WIND[b])
    b++;
  return b;
}

// How the sea looks at each force: the chance an arch of the waves has a crest, that it breaks white and from how
// far down its top (out of 16), the chance of foam down a breaking wave's face and of a streak of blown foam, and
// how many rows of spray come off the crests. Chances are out of 256.
static const SeaLook SEA_LOOK[13] = {{0, 0, 16, 0, 0, 0},          // 0 calm: a mirror
                                     {40, 0, 16, 0, 0, 0},         // 1 ripples
                                     {80, 0, 16, 0, 0, 0},         // 2 small wavelets
                                     {110, 30, 14, 0, 0, 0},       // 3 large wavelets, a white horse here and there
                                     {140, 77, 12, 0, 0, 0},       // 4 small waves, fairly frequent white horses
                                     {170, 128, 11, 0, 0, 0},      // 5 moderate waves, many white horses
                                     {190, 180, 9, 50, 0, 0},      // 6 large waves, white foam crests everywhere
                                     {200, 205, 7, 80, 30, 0},     // 7 the sea heaps up, the foam blows in streaks
                                     {210, 230, 6, 110, 50, 1},    // 8 spindrift off the crests
                                     {220, 255, 5, 140, 80, 2},    // 9 dense streaks of foam, spray
                                     {230, 255, 3, 180, 120, 3},   // 10 the crests tumble, the sea looks white
                                     {240, 255, 2, 210, 160, 4},   // 11 exceptionally high waves
                                     {250, 255, 0, 240, 210, 5}};  // 12 the air is full of foam and spray

// 0..255, the same each time for the same a, b and c
static uint32_t hash8(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u ^ (c + 0x165667B1u) * 0xC2B2AE3Du;
  h ^= h >> 15;
  h *= 0x2C1B3C6Du;
  h ^= h >> 12;
  return h >> 24;
}

static int luma(uint32_t c) { return (int) ((c >> 16) * 3 + (c >> 8 & 255) * 6 + (c & 255)) / 10; }

// ---------------------------------------------------------------- the moment

// What the heath goes by besides Now
struct Day {
  bool awake;    // from six to midnight, the cabin's light can be on
  bool indoors;  // rain over 4 mm in the hour, or the night: everyone's in
  bool pride;    // International Pride Day: the rainbow flag
  bool pride_up;
  int bft;     // the wind's Beaufort force
  float wave;  // wave height in m: Oceanforecast's, or the table's for bft
};

static Day day_of(const Now &n) {
  Day d;
  int minutes = (int) (n.local % 86400 / 60);
  d.awake = minutes >= 6 * 60;                                                            // asleep from midnight to six
  d.indoors = (n.rain && n.w.ok && n.w.rain > 4) || !d.awake || minutes >= 23 * 60 + 30;  // home from half past 11
  d.bft = n.w.ok ? beaufort(n.w.wind) : 1;  // without the weather, a little light on the water
  d.wave = n.w.wave >= 0 ? n.w.wave : BFT_WAVE[d.bft];
  d.pride = n.month == 6 && n.day == 28;
  // the flag's hours, as Now has them for flag days
  bool set = n.sun.el < -0.5f && n.sun.az > 180;
  d.pride_up = d.pride && n.hour >= 8 && n.hour < 21 && !set;
  return d;
}

// ---------------------------------------------------------------- the figures

enum Fig : uint8_t { F_CLIMBER, F_RUNNER, F_SKIER_CLIMBER, F_KAYAKER, F_RUNNER_KAYAKER, F_SKIER_KAYAKER, F_DOG };
static const art::Figure *const FIGURES[] = {&art::CLIMBER, &art::RUNNER,         &art::SKIER_CLIMBER,
                                             &art::KAYAKER, &art::RUNNER_KAYAKER, &art::SKIER_KAYAKER,
                                             &art::DOG};

// The three of them behind the big window, 7 by 13 from its top left, leftmost pixel in bit 6: someone standing,
// someone sitting, and the dog's head between them. They keep off its edges so it still reads as a window, and its
// frames stay in front of them.
static const uint8_t INDOORS[13] = {0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x36, 0x36, 0x00, 0x2A, 0x3A, 0x12, 0x00};
static constexpr int INDOORS_X = art::WINDOW_X - 4, INDOORS_Y = art::WINDOW_Y - 6;

// The artboard moon's maria, relative to its middle: x, y, half width, half height
static const int8_t MARIA[6][4] = {{-3, -3, 2, 2}, {1, -4, 2, 1}, {3, -1, 2, 2},
                                   {-5, 1, 2, 3},  {-1, 3, 2, 1}, {4, 2, 1, 1}};

// a reindeer as in the rock carvings, facing left, 11 wide
static const uint32_t DEER[11] = {0x540, 0x380, 0x100, 0x380, 0x680, 0x0FE, 0x0FF, 0x07E, 0x052, 0x092, 0x091};
// a walrus swimming, facing left: its head and back at the water, 11 wide, and its tusks, 5 wide
static const uint32_t WALRUS[3] = {0x3C0, 0x7E2, 0x7FF};
static const uint32_t TUSKS[2] = {0x009, 0x009};

// ---------------------------------------------------------------- the landscape

// The landscape in daylight for each season_of(), for the slots that change with the seasons. At night they're
// the artboard's colours (art::NIGHT), or SNOW_NIGHT under snow. The mountain is bare granite all year, its ridge
// band dark; in snow only its lower bands go white.
enum Seasonal : uint8_t {
  L_MTN_A,
  L_MTN_B,
  L_MTN_C,
  L_HEATH_FAR,
  L_HEATH_MID,
  L_HEATH_NEAR,
  L_HEATH_FORE,
  L_GRANITE_FAR,
  L_GRANITE,
  L_PATH,
  L_ROCK_LIGHT,
  L_N
};
static const uint8_t SEASONAL_SLOT[L_N] = {art::S_MTN_A,     art::S_MTN_B,      art::S_MTN_C,      art::S_HEATH_FAR,
                                           art::S_HEATH_MID, art::S_HEATH_NEAR, art::S_HEATH_FORE, art::S_GRANITE_FAR,
                                           art::S_GRANITE,   art::S_PATH,       art::S_ROCK_LIGHT};
// clang-format off: laid out by hand. mountain bands, heath far to near, granite far and near, path, light rock
static const uint32_t SEASON_DAY[5][L_N] = {
    {0x4E4C4A, 0xA8B0B8, 0xC8D0D8, 0xE8EEF4, 0xD6E0EA, 0xC6D2DE, 0xB4C2D0, 0xD0DAE2, 0xC4CED8, 0xBCC8D4, 0xE4EBF0},
    {0x4A4642, 0x5E5A54, 0x74706A, 0x76745E, 0x5E5C4C, 0x4E4C40, 0x3E3C34, 0x7C8086, 0x868C92, 0x8A8270, 0xA4AEB6},
    {0x504C46, 0x66625A, 0x7C7870, 0x8CA868, 0x6E9050, 0x5A7C46, 0x466438, 0x8A949A, 0x949CA2, 0x9E9276, 0xACB6BE},
    {0x4E4A44, 0x64605A, 0x7A766E, 0x7A9660, 0x5E7C4C, 0x4A6A40, 0x385434, 0x88929A, 0x929AA2, 0x9C8E72, 0xACB6BE},
    {0x524A44, 0x686058, 0x7E766E, 0x8E8E56, 0x727244, 0x5C5E3A, 0x484C30, 0x8A9096, 0x949AA0, 0x9A8A6C, 0xA8B0B8}};
static const uint32_t SNOW_NIGHT[L_N] = {
     0x22262E, 0x3E4858, 0x4E5868, 0x46586E, 0x3C4E64, 0x34465C, 0x2C3C52, 0x3C4C60, 0x38485C, 0x34465C, 0x4A5C70};
// The mountain on a night without the aurora: the artboard's blue night, half way to rock. With the aurora it's
// the artboard as drawn.
static const uint32_t ROCK_NIGHT[3] = {0x22242A, 0x30333A, 0x40434A};
// clang-format on

// The birch's crown by day and at night: bare twigs, its first leaves, summer, autumn, and its last leaves. Bare
// twigs in snow are frosted.
static const uint32_t BIRCH_DAY[5] = {0x6E5A60, 0x9CC45C, 0x5A8A36, 0xD8A42C, 0xC8862A};
static const uint32_t BIRCH_NIGHT[5] = {0x262024, 0x22382A, 0x1E3426, 0x3A3418, 0x36301A};
static constexpr uint32_t BIRCH_FROST_DAY = 0xA8A2AA, BIRCH_FROST_NIGHT = 0x40404C;

// The other slots in daylight, all year. The window, the light it spills, heather and cotton grass are worked
// out in colours().
static uint32_t day_colour(int slot) {
  switch (slot) {
    case art::S_SEA_FAR:
      return 0x3A6EA2;
    case art::S_SEA_NEAR:
      return 0x285A8C;
    case art::S_ROCK_DARK:
      return 0x4A5662;
    case art::S_SHEEP:
      return 0xE6E8E2;
    case art::S_SHEEP_DARK:
      return 0x2C2E30;
    case art::S_CABIN_WALL:
      return 0x2A323C;
    case art::S_CABIN_TRIM:
      return 0x93A8BA;
    case art::S_TREE:
      return 0x2A4430;
    case art::S_JUNIPER:
      return 0x2E4A36;
    case art::S_BIRCH_BARK:
      return 0xD8DAD2;
    default:
      return 0x1A1E26;  // the dog, the cabin's roof and frames: dark all day
  }
}

static uint32_t &slot(Scene &s, int k) { return s.rgb[C_ART + k]; }

// The land for the animations' walks and drawn(): the mountain's top, the land's in front of it, and the near
// ground's, where the near heath, the granite and the path start, the nearest column's where there's none
static void make_land(Land &land) {
  using namespace art;
  memcpy(land.hill[0], RIDGE, W);
  memcpy(land.hill[1], LAND, W);
  uint8_t *near = land.hill[2];
  memset(near, NONE, W);
  auto ground = [](int k) {
    return k == S_HEATH_NEAR || k == S_HEATH_FORE || k == S_GRANITE || k == S_PATH || k == S_JUNIPER ||
           k == S_HEATHER || k == S_COTTON || k == S_SPILL;
  };
  for (int y = GROUND_Y0; y < H; y++)
    for (int i = GROUND_ROW[y - GROUND_Y0], x = 0; i < GROUND_ROW[y - GROUND_Y0 + 1]; i += 2)
      for (int end = x + GROUND[i + 1]; x < end; x++)
        if (near[x] == NONE && ground(GROUND[i]))
          near[x] = (uint8_t) y;
  for (int x = 0; x < W; x++)
    if (near[x] == NONE) {
      int best = -1;
      for (int d = 1; d < W && best < 0; d++)
        for (int e : {x - d, x + d})
          if (best < 0 && e >= 0 && e < W && near[e] != NONE)
            best = near[e];
      near[x] = (uint8_t) (best < 0 ? H - 1 : best);
    }
}

// The landscape's colour for each slot: night to day with the light, by season, then the sky's haze on the
// mountain, the low sun's glow, how overcast and wet it is, and the fog
static void colours(Scene &s, const Now &n, const Day &d) {
  using namespace art;
  Heath &h = s.heath;
  int seasonal[SLOTS];
  for (int k = 0; k < SLOTS; k++)
    seasonal[k] = -1;
  for (int l = 0; l < L_N; l++)
    seasonal[SEASONAL_SLOT[l]] = l;
  uint32_t sky4 = s.rgb[C_SKY + 4], fog = s.rgb[C_FOG];
  bool wet = n.rain && !n.white;
  // The birch leafs out in mid May, is summer green from mid June, yellow from mid September and loses its last
  // leaves by late October. Up north it's later in spring and earlier in autumn, two days a degree.
  int doy = day_of_year(n.year, n.month, n.day), late = (int) lroundf((float) (n.lat - 60) * 2);
  int birch = doy < 130 + late || doy >= 296 - late ? 0
              : doy < 165 + late                    ? 1
              : doy < 255 - late                    ? 2
              : doy < 282 - late                    ? 3
                                                    : 4;
  h.birch_keep = (uint8_t) (birch == 0 ? 90 : birch == 4 ? 140 : 255);  // bare it's mostly what's behind
  for (int k = 1; k < SLOTS; k++) {
    int l = seasonal[k];
    // at night the mountain's misty foot is as dark as the band above it, so it doesn't run into the bay
    int nk = k == S_MTN_C && !n.white ? S_MTN_B : k, nl = l == L_MTN_C && !n.white ? L_MTN_B : l;
    uint32_t night = l >= 0 && n.white ? SNOW_NIGHT[l] : NIGHT[nk];
    if (l >= 0 && l <= L_MTN_C && !n.white && !s.aurora)
      night = mix(night, ROCK_NIGHT[nl], 0.5f);
    uint32_t day = l >= 0 ? SEASON_DAY[n.season][l] : day_colour(k);
    if (k == S_BIRCH) {
      bool frost = n.white && birch == 0;
      night = frost ? BIRCH_FROST_NIGHT : BIRCH_NIGHT[birch];
      day = frost ? BIRCH_FROST_DAY : BIRCH_DAY[birch];
    }
    uint32_t c = mix(night, day, n.light);
    int depth = DEPTH[k];
    if (depth == 0)  // a little of the sky's colour by day, but it stands out
      c = mix(c, sky4, 0.08f * n.light);
    else if (k == S_HEATH_FAR || k == S_GRANITE_FAR || k == S_SEA_FAR)
      c = mix(c, sky4, 0.1f * n.light);
    if (s.glow && depth <= 1 && k != S_SEA_FAR && k != S_SEA_NEAR)  // alpenglow
      c = mix(c, 0xFF9070, 0.2f * n.glow);
    if (n.grey > 0.2f) {  // flat light under cloud
      int y = luma(c);
      c = mix(c, rgb4(y, y, y + 6), 0.3f * n.grey);
    }
    if (wet && depth >= 1 && k != S_SEA_FAR && k != S_SEA_NEAR && k != S_WINDOW)
      c = mix(c, 0x000000, 0.12f);
    if (n.fog > 15)
      c = mix(c, fog, n.fog / 100 * (0.5f - depth * 0.15f));
    slot(s, k) = c;
  }
  slot(s, S_MTN_RIM) = slot(s, S_MTN_A);  // no aurora glow on the ridge: it stands dark against the sky
  slot(s, S_HEATHER) = (n.month == 8 || n.month == 9) && !n.white  // in bloom
                           ? mix(mix(NIGHT[S_HEATHER], 0x8E5E90, n.light), slot(s, S_HEATH_NEAR), n.fog / 200)
                           : slot(s, S_HEATH_NEAR);
  slot(s, S_COTTON) =
      (n.month == 6 || n.month == 7) && !n.white ? mix(NIGHT[S_COTTON], 0xECEEE8, n.light) : slot(s, S_HEATH_NEAR);
  if (n.white)  // the sheep are in for the winter
    slot(s, S_SHEEP) = slot(s, S_SHEEP_DARK) = slot(s, S_HEATH_MID);
  slot(s, S_BIRCH_TWIG) = h.birch_keep < 255 ? slot(s, S_BLACK) : slot(s, S_BIRCH);  // its branches show when bare
  slot(s, S_SPILL) = s.window_lit ? mix(slot(s, S_HEATH_NEAR), 0xFFB45A, 0.3f) : slot(s, S_HEATH_NEAR);
  slot(s, S_WINDOW) =
      s.window_lit ? s.rgb[C_WINDOW] : mix(slot(s, S_BLACK), mix(s.rgb[C_SKY + 2], 0x2A3440, 0.55f), n.light);
  // By day small things share a colour with something like them, so the 16 go to the sky and the land's shapes.
  // At night they keep the artboard's own.
  uint32_t white = s.rgb[C_WHITE];
  if (n.light > 0) {
    if (!n.white) {
      slot(s, S_SHEEP) = white;
      slot(s, S_SHEEP_DARK) = slot(s, S_BLACK);
      slot(s, S_JUNIPER) = slot(s, S_HEATH_FORE);
    }
    if (slot(s, S_COTTON) != slot(s, S_HEATH_NEAR))
      slot(s, S_COTTON) = white;
    slot(s, S_GRANITE_FAR) = slot(s, S_PATH) = slot(s, S_GRANITE);
    slot(s, S_BIRCH_BARK) = white;
    slot(s, S_CABIN_TRIM) = slot(s, S_ROCK_LIGHT);
    if (!s.window_lit)
      slot(s, S_WINDOW) = slot(s, S_SEA_FAR);
  }
  // By day the mountain's ridge stands out, the bay keeps its two for the waves, and the cabin its wall, so its
  // door and frames show. Unless it's in the fog.
  bool keep = n.light > 0 && n.fog < 60;
  for (int k : {S_MTN_A, S_SEA_FAR, S_SEA_NEAR, S_CABIN_WALL})
    s.keep.set(C_ART + k, keep);
  s.keep.set(C_ART + (int) S_WINDOW, s.window_lit);
  s.keep.set(C_SUN, s.sun);
  s.keep.set(C_MOON, s.moon);
  s.keep.set(C_AUR_EDGE, s.aurora);
}

// Fog over the mountain, and over the far land and the bay when it's thick
static void fog_edges(Scene &s, const Now &n, uint32_t seed) {
  Rng h{seed};
  float ph = h.f() * 6.283f;
  for (int f = 0; f < 2; f++)
    for (int x = 0; x < W; x++) {
      bool on = f == 0 ? n.fog > 15 : n.fog > 45;
      int edge = f == 0 ? HZ - 14 - (int) (n.fog * 0.1f) : 178 - (int) (n.fog * 0.12f);
      s.fog_top[f][x] = (uint8_t) (on ? edge + (int) (3 * sinf(x / 17.0f + ph + f)) : NONE);
    }
}

// The bay by the Beaufort scale, new each hour. Rows of waves from the far edge, their crests as high as the waves
// are, smaller and closer together further off; how much of them breaks white goes by the force. In the dark only
// the light on the water shows small waves. And the wind's streaks in the air above it.
static void bay(Scene &s, const Now &n, const Day &d) {
  using namespace art;
  Heath &h = s.heath;
  const Weather &w = n.w;
  if (d.bft == 0 && w.ok)  // calm, the bay's a mirror of the sky
    slot(s, S_SEA_FAR) = mix(slot(s, S_SEA_FAR), s.rgb[C_SKY + 4], 0.15f * n.light);
  // the sun's or moon's light on the bay
  if (s.moon && n.dark)
    s.shine_x = s.moon_x;
  else if (s.sun && n.sun.el < 20 && n.grey < 0.5f)
    s.shine_x = s.sun_x;
  s.rgb[C_SHINE] = mix(slot(s, S_SEA_FAR), 0xFFFFFF, 0.3f + 0.25f * n.light);
  h.bft = d.bft;
  h.look = SEA_LOOK[w.ok ? d.bft : 0];
  h.look.crest = (uint8_t) std::max<int>(h.look.crest, std::min(200, (int) (d.wave * 80)));  // swell without wind
  if (n.light <= 0 && d.bft < 4 && d.wave < 1)
    h.look.crest = 0;
  h.wind_dir = n.wdir;
  h.sea_seed = (uint32_t) (n.t / 3600) * 7919u + 11;
  Rng sr{h.sea_seed};
  h.swell = clamp01((d.wave - BFT_WAVE[d.bft]) / 2);
  float amp = std::max(0.0f, 2.7f * sqrtf(d.wave) - 0.6f),  // at the bottom, px
      len = (10 + 7 * powf(d.wave, 0.6f)) * (1 + 0.8f * h.swell);
  for (int y = HZ + 2; y < H && h.nsea < SEA_ROWS;) {
    float p = (y - HZ) / (float) (H - HZ), g = 0.25f + 0.75f * p;
    int a = (int) lroundf(amp * g);
    h.sea_y[h.nsea] = (int16_t) y;
    h.sea_a[h.nsea] = (uint8_t) a;
    h.sea_len[h.nsea] = (uint8_t) std::max(4, (int) lroundf(len * (0.45f + 0.55f * p)));
    h.sea_ph[h.nsea++] = (uint8_t) sr.in(0, 255);
    y += std::max(2, a + 2 + (int) lroundf(2.5f * g));
  }
  h.bumps = d.wave >= 7 ? 2 : d.wave >= 3.5f ? 1 : 0;
  h.surf = d.wave >= 1 ? std::min(5.0f, d.wave * 0.5f) + (d.bft >= 6 ? 1 : 0) : d.bft >= 5 ? 1 : 0;
  h.surf_rock = d.wave >= 3 ? std::min(4.0f, (d.wave - 2) * 0.6f) : 0;
  h.hz_x0 = W;
  h.hz_x1 = -1;
  for (int i = GROUND_ROW[HZ + 1 - GROUND_Y0], x = 0; i < GROUND_ROW[HZ + 2 - GROUND_Y0]; i += 2) {
    if (GROUND[i] == S_SEA_FAR || GROUND[i] == S_SEA_NEAR) {
      h.hz_x0 = (int16_t) std::min<int>(h.hz_x0, x);
      h.hz_x1 = (int16_t) (x + GROUND[i + 1] - 1);
    }
    x += GROUND[i + 1];
  }
  // the sky in the waves' backs: the band a little lighter than the water
  uint32_t &crest = s.rgb[C_CREST] = s.rgb[C_SKY + 4];
  for (int i = 4; i >= 0; i--)
    if (luma(s.rgb[C_SKY + i]) >= luma(slot(s, S_SEA_FAR)) + 25 && luma(s.rgb[C_SKY + i]) < luma(crest))
      crest = s.rgb[C_SKY + i];
  bool white = n.light > 0.5f && s.rgb[C_WHITE] != s.rgb[C_SUN];  // not the sun's yellow
  s.rgb[C_FOAM] = white ? s.rgb[C_WHITE] : s.rgb[C_SHINE];
  // Streaks in the air above it, from 3 m/s by day and 10 in the dark: dashed where the wind comes from, curling
  // where it goes
  if (w.wind >= 3 && (n.light > 0 || w.wind >= 10)) {
    Rng st{(uint32_t) (n.t / 3600) * 104729u + 5};
    h.nstreaks = std::min(5, (int) (w.wind / 3));
    for (int i = 0; i < h.nstreaks; i++) {
      int len = std::min(44, 10 + (int) (w.wind * 2));
      h.streak_len[i] = (uint8_t) len;
      h.streak_x[i] = (int16_t) st.in(0, std::max(0, 96 - len));
      h.streak_y[i] = (int16_t) (98 + i * 11 + st.in(-3, 3));
      h.streak_ph[i] = (uint8_t) st.in(0, 255);
    }
    s.rgb[C_STREAK] = n.light > 0.5f ? s.rgb[C_WHITE] : mix(s.rgb[C_SKY + 3], 0xFFFFFF, 0.3f + 0.3f * n.light);
  }
}

// The two of them and the dog: climbing the mountain, paddling the bay, and on the hill by the cabin. When it rains
// the climber runs the ridge instead, with waves over 2 m or wind over 12 m/s the kayaker comes ashore and runs,
// with snow both are out on skis, and in rain over 4 mm all three are in the cabin, behind its big window. From
// half past eleven to six they're all home, behind the window until its light goes out at midnight.
static void people(Scene &s, const Now &n, const Day &d) {
  Heath &h = s.heath;
  s.rgb[C_FIG] = mix(0x070B11, 0x1A1E26, n.light);
  if (d.indoors) {
    h.indoors = s.window_lit;  // seen only with the light on
    return;
  }
  bool skis = n.white || (n.snow && !n.rain), run = !skis && n.rain, ashore = !skis && (n.w.wind > 12 || d.wave > 2);
  h.fig[h.nfig++] = skis ? F_SKIER_CLIMBER : run ? F_RUNNER : F_CLIMBER;
  h.fig[h.nfig++] = skis ? F_SKIER_KAYAKER : ashore ? F_RUNNER_KAYAKER : F_KAYAKER;
  h.fig[h.nfig++] = F_DOG;
  h.lamp = n.sun.el < -3;  // the headlamp's on when it's getting dark
  s.rgb[C_LAMP] = rgb4(247, 249, 250);
  s.rgb[C_LAMP_GLOW] = mix(s.rgb[C_SKY + 3], 0xDBE6EC, 0.35f);
  s.keep.set(C_LAMP, h.lamp);
}

void heath_ground(Scene &s, const Now &n, uint32_t seed) {
  using namespace art;
  Land &land = *s.land;
  if (!land.made || land.seed != seed || land.ground != G_HEATH) {
    make_land(land);
    land.seed = seed;
    land.ground = G_HEATH;
    land.made = true;
  }
  Day d = day_of(n);
  // where the cabin's things are, for the animations: the roof's top 15 rows under a gull's feet, the window's
  // lower half, the chimney's top, the pole only on flag days
  s.cabin = true;
  s.cabin_x = (int16_t) art::DOOR_X;
  s.cabin_y = (int16_t) art::DOOR_Y;
  s.win = {(int16_t) (WINDOW_X - 3), (int16_t) WINDOW_Y};
  s.home_x = (int16_t) HOME_X;
  s.chimney = {(int16_t) CHIMNEY_X, (int16_t) (CHIMNEY_Y - 1)};
  s.pole_x = -1;
  // lit in the dark and with everyone in, until they go to bed at midnight; at Christmas and New Year all night
  s.window_lit = (n.sun.el < -3 || d.indoors) && (d.awake || n.xmas || n.newyear);
  s.rgb[C_WINDOW] = n.halloween ? rgb4(255, 140, 40) : 0xFFCC77;
  if (s.moon) {  // the artboard's moon has maria, and a halo round its glow
    s.rgb[C_CRATER] = mix(s.rgb[C_MOON], 0x93A1AD, 0.5f);
    s.rgb[C_HALO] = mix(s.rgb[C_SKY + std::min(band_of(s.moon_y), 2)], s.rgb[C_MOON], 0.07f);
  }
  if (s.bolt) {  // out of the cloud down to the ridge
    int step = std::max(6, (RIDGE[std::clamp<int>(s.bolt_x[0], 0, W - 1)] + 4 - s.bolt_y[0]) / 6);
    for (int i = 1; i < 7; i++)
      s.bolt_y[i] = (int16_t) (s.bolt_y[0] + i * step);
  }
  colours(s, n, d);
  fog_edges(s, n, seed);
  bay(s, n, d);
  people(s, n, d);
  s.rgb[C_TREE] = s.rgb[C_FIG];  // the birds'
  s.rgb[C_DARK] = slot(s, S_BLACK);
}

// w bits the other way round, so a sprite faces the other way
static uint32_t mirror(uint32_t bits, int w) {
  uint32_t m = 0;
  for (int i = 0; i < w; i++)
    m |= (bits >> i & 1) << (w - 1 - i);
  return m;
}

// The rainbow flag for Pride, 22 by 18 in six stripes, on layer 2
static void flag_pride(Scene &s, int x, int y) {
  static const uint32_t STRIPES[6] = {0xE40303, 0xFF8C00, 0xFFED00, 0x008026, 0x24408E, 0x732982};
  for (int k = 0; k < 6; k++)
    box(s, x, y + 3 * k, 22, 3, 2, rgb4(STRIPES[k] >> 16, STRIPES[k] >> 8 & 255, STRIPES[k] & 255));
}

// Now and then, while it's light: a reindeer from the rock carvings in the ochre they're painted with, peeking out of
// the trees behind the cabin or walking out of them over the hour, and a walrus swimming in the bay. Each hour
// rolls its own, apart from the rest so nothing else moves. The show on the heath brings its own, see
// met_live_heath.h.
static void animals(Scene &s, const Now &n, const Day &d) {
  if (n.light <= 0)
    return;
  Rng a{(uint32_t) (n.t / 3600) * 2654435761u + 77};
  int minute = (int) (n.local % 3600 / 60), deer = a.in(0, 99);
  uint32_t ochre = mix(0x4A2618, 0x9E4A2E, n.light);  // the walrus is that reddish brown too, one colour for both
  s.tag = L_HEATH;
  if (deer >= 50) {  // half the hours not there, then peeking, then walking out a pixel every five minutes
    int x = deer < 80 ? art::TREES_X + a.in(0, 2) : art::TREES_X - minute / 5;
    sprite_rows(sprite(s, x, art::DOOR_Y + 1 - 10, 11, 11, 1, ochre), DEER);
  }
  int walrus = a.in(0, 99), x = a.in(4, 40), y = a.in(0, 16), dir = a.in(0, 1) ? 1 : -1;
  if (y < 6) {  // out at the back of the bay, behind the kayak, where the bay is narrower
    y += 160;
    x = 14 + x * 24 / 40;
  } else {  // or between the kayak and the wave height
    y += 190;
  }
  if (walrus < 35 && n.fog <= 45 && d.bft <= 6) {  // a third of the hours, swimming a pixel every ten minutes
    x += dir * minute / 10;
    uint32_t body[3], tusks[2];
    for (int r = 0; r < 3; r++)
      body[r] = dir < 0 ? WALRUS[r] : mirror(WALRUS[r], 11);
    for (int r = 0; r < 2; r++)
      tusks[r] = dir < 0 ? TUSKS[r] : mirror(TUSKS[r], 5);
    sprite_rows(sprite(s, x, y, 11, 3, 1, ochre), body);
    sprite_rows(sprite(s, dir < 0 ? x : x + 6, y + 3, 5, 2, 1, s.rgb[C_WHITE]), tusks);
  }
  s.tag = 0;
}

void heath_life(Scene &s, const Now &n, Rng &rng) {
  using namespace art;
  Heath &h = s.heath;
  Day d = day_of(n);
  // Chimney smoke while the heater's on, more in fireplace mode: a wisp curling up from the chimney as on the
  // artboard, taller the more there is. The animations' puffs take its place, or hearts on Valentine's day.
  int puffs = n.o.fireplace ? 5 : n.o.heater_pct > 0 ? 2 + std::min(3, n.o.heater_pct / 30) : 0;
  if (n.midsummer)
    puffs = 0;
  s.puffs = (uint8_t) (n.valentine && !puffs ? 3 : puffs);
  h.nsmoke = puffs ? std::min(48, 6 + puffs * 8) : 0;
  h.smoke_w = (uint8_t) (n.o.fireplace ? 2 : 1);
  s.rgb[C_SMOKE] = n.light > 0.5f ? s.rgb[C_WHITE] : mix(s.rgb[C_SKY + 2], 0xD8E2EA, 0.4f + 0.25f * n.light);
  float ph = rng.f() * 6.283f, lean = n.wdir * (1 + n.wind / 4) / 60.0f;
  for (int k = 0; k < h.nsmoke; k++)
    h.smoke_x[k] = (int16_t) (CHIMNEY_X + lean * k * k + 1.5f * sinf(k / 4.0f + ph));
  // The flag on flag days, or the rainbow flag on Pride day, on a pole by the cabin, flying above its roof and the
  // way the wind blows. The Norwegian blue is the top of the sky and the white white's, colours we have anyway.
  if (n.flag_up || d.pride_up) {
    s.pole_x = POLE_X;
    s.pole_base = (int16_t) (LAND[POLE_X] + 1);
    s.pole_top = (int16_t) std::min(EAVE_L_Y - 21, s.pole_base - 34);
    int fx = n.wdir < 0 ? POLE_X - 23 : POLE_X + 2;
    if (n.flag_up) {
      s.tag = L_FLAG;
      flag_norway(s, fx, s.pole_top, n.wdir, mix(0x601418, 0xEF2B2D, n.light), s.rgb[C_WHITE], s.rgb[C_SKY]);
      s.tag = 0;
    } else {
      flag_pride(s, fx, s.pole_top);
    }
  }
  animals(s, n, d);
}

void heath_holidays(Scene &s, const Now &n, Rng &rng) {
  using namespace art;
  uint32_t window = s.rgb[C_WINDOW], dark = slot(s, S_BLACK);
  if (n.xmas && n.dark) {  // lights along the eaves, a star on the chimney
    uint32_t cols[3] = {rgb4(255, 60, 60), window, rgb4(60, 220, 100)};
    s.tag = L_XMAS;
    for (int k = 0; k <= 12; k++) {
      int x = EAVE_L_X + (EAVE_R_X - EAVE_L_X) * k / 12, y = EAVE_L_Y + (EAVE_R_Y - EAVE_L_Y) * k / 12 + 1;
      s.light[k] = {(int16_t) x, (int16_t) y, (uint8_t) (k % 3)};
      box(s, x, y, 2, 1, 2, cols[k % 3]);
    }
    s.nlights = 13;
    s.star = {(int16_t) (CHIMNEY_X - 3), (int16_t) (CHIMNEY_Y - 9)};
    sprite_rows(sprite(s, s.star.x, s.star.y, 7, 7, 2, window), STAR7.row);
    s.tag = 0;
  }
  if (n.easter_days && !n.white) {  // striped eggs in the heather in front of the cabin
    uint32_t cols[3] = {window, rgb4(230, 100, 170), rgb4(100, 190, 230)};
    Rng eg{(uint32_t) n.year * 7u};
    for (int i = 0; i < (int) std::size(s.egg); i++) {
      int x = HOME_X - 30 + i * 13 + eg.in(-3, 3), y = LAND[x] + 8 + eg.in(0, 5);
      s.egg[s.neggs++] = {(int16_t) x, (int16_t) y};
      blob(s, x, y, 4, 5, 1, cols[i % 3]);
      Sprite *stripes = sprite(s, x - 3, y - 2, 7, 4, 1, s.rgb[C_WHITE]);  // both in one, sprites are scarce
      if (stripes)
        stripes->rows[0] = stripes->rows[3] = 0x7F;
    }
  }
  if (n.midsummer) {  // the bonfire, on the hill away from the cabin
    int bx = HOME_X - 30, by = LAND[bx] + 8;
    s.fire = {(int16_t) bx, (int16_t) by};
    box(s, bx - 9, by - 2, 19, 3, 2, dark);  // the logs
    s.tag = L_BONFIRE;
    sprite_rows(sprite(s, bx - 7, by - 21, 15, 20, 2, rgb4(255, 130, 30)), FLAME.row);
    sprite_rows(sprite(s, bx - 7, by - 21, 15, 20, 2, rgb4(255, 220, 90)), FLAME_IN.row);
    for (int i = 0; i < 10; i++)
      box(s, bx + rng.in(-12, 12), by - 24 - rng.in(0, 26), 1, 1, 2, rgb4(255, 220, 90));
    blob(s, bx + n.wdir * 10, by - 36, 6, 4, 1, slot(s, S_MTN_C));  // smoke in the mountain's blue-grey
    blob(s, bx + n.wdir * 18, by - 46, 8, 5, 1, slot(s, S_MTN_C));
    s.tag = 0;
  }
  if (n.halloween) {  // a pumpkin by the door
    int px = DOOR_X - 14, py = DOOR_Y - 5;
    s.pumpkin = {(int16_t) px, (int16_t) py};
    blob(s, px, py, 7, 6, 1, rgb4(255, 130, 40));
    box(s, px - 1, py - 8, 3, 3, 1, dark);  // its stalk
    if (n.dark)
      sprite_rows(sprite(s, px - 5, py - 3, 11, 7, 2, rgb4(17, 17, 34)), FACE.row);
  }
}

void heath_overlay(Scene &s, const Now &n) {
  using namespace art;
  // outline in the darkest colour already in the picture, if it's dark enough
  uint32_t cand[] = {s.rgb[C_SKY], slot(s, S_MTN_A), slot(s, S_HEATH_FORE), slot(s, S_BLACK), s.rgb[C_SHADE + 2]};
  uint32_t &out = s.rgb[C_OUTLINE] = cand[0];
  for (uint32_t c : cand)
    if (luma(c) < luma(out))
      out = c;
  if (luma(out) > 70)
    out = rgb4(17, 17, 34);
  if (n.w.wave >= 0) {  // and Oceanforecast's wave height, in the bay
    char b[8];
    snprintf(b, sizeof b, n.w.wave < 9.95f ? "%.1fm" : "%.0fm", n.w.wave);
    s.heath.sea_text = true;
    ov_text(s, SEA_OV_X + 12, SEA_OV_Y + 4, b, 2);
  }
}

// ---------------------------------------------------------------- drawing a row

static void span(uint8_t *row, int a, int b, uint8_t c) {
  a = std::max(a, 0);
  b = std::min(b, W - 1);
  if (a <= b)
    memset(row + a, c, (size_t) (b - a + 1));
}

static void disc(uint8_t *row, int y, int cx, int cy, int r, uint8_t c) {
  int h = half(y - cy, r, r);
  if (h >= 0)
    for (int k = -1; k <= 1; k++)  // north is at both edges, so draw it there too
      span(row, cx - h + k * W, cx + h + k * W, c);
}

void heath_moon_row(const Scene &s, int y, uint8_t *out) {
  if (s.moon_glow) {
    disc(out, y, s.moon_x, s.moon_y, MOON_HALO_R, s.ix[C_HALO]);
    disc(out, y, s.moon_x, s.moon_y, MOON_GLOW_HR, s.ix[C_MOON_GLOW]);
  }
  int dy = y - s.moon_y, h = half(dy, MOON_R, MOON_R);
  if (h < 0)
    return;
  // lit from the right while waxing, from the left after full
  float lim = cosf(s.moon_phase * 2 * kPiF) * MOON_R * sqrtf(1 - dy * dy / (float) (MOON_R * MOON_R));
  for (int dx = -h; dx <= h; dx++) {
    bool lit = s.moon_phase < 0.5f ? dx > lim : dx < -lim;
    int x = s.moon_x + dx;
    if (!lit || x < 0 || x >= W)
      continue;
    bool mare = false;
    for (const int8_t *m : MARIA)
      mare = mare || (half(dy - m[1], m[2], m[3]) >= abs(dx - m[0]));
    out[x] = s.ix[mare ? C_CRATER : C_MOON];
  }
}

// The bay at x, y, base its colour without the waves. near is the nearer water, darker, where the waves' faces show
// lighter than it; further off they show as a crest over a dark trough. Nearer waves are drawn over further ones.
static uint8_t sea_px(const Scene &s, int x, int y, uint8_t base, bool near) {
  const Heath &hs = s.heath;
  const SeaLook &l = hs.look;
  uint8_t c = base, foam = s.ix[C_FOAM];
  if (l.streak && y % 3 == 0) {  // foam blown into long streaks along the wind
    int u = x + (int) hash8((uint32_t) y, 7, hs.sea_seed) % 16;
    uint32_t h = hash8((uint32_t) (u / 16), (uint32_t) y, hs.sea_seed);
    if (h < l.streak && u % 16 < 6 + (int) (h % 8))
      c = foam;
  }
  if (!l.crest)
    return c;
  float peak = 0.5f + 0.15f * hs.wind_dir;  // the side the wind blows to is steeper
  for (int j = 0; j < hs.nsea; j++) {
    int b0 = hs.sea_y[j], a = hs.sea_a[j];
    if (y > b0 + 2 || y < b0 - a - l.spray - 1)
      continue;
    int len = hs.sea_len[j], pos = x + hs.sea_ph[j], m = pos / len;
    uint32_t h = hash8((uint32_t) j, (uint32_t) m, hs.sea_seed);
    if (h >= l.crest)
      continue;               // no crest on this arch
    b0 += (int) (h % 3) - 1;  // not quite in line with the next one
    float u = (pos % len + 0.5f) / len, b = u < peak ? u / peak : (1 - u) / (1 - peak);
    b = b * (0.4f + 0.6f * b);  // pointed at the top
    if (b < (0.12f + (h >> 2) % 4 * 0.08f) * (1 - 0.7f * hs.swell))
      continue;  // and only so long, swell longer
    bool cap = hash8((uint32_t) j, (uint32_t) m, hs.sea_seed ^ 0xA5A5u) < l.cap && b * 16 >= l.cap_from;
    int top = b0 - (int) lroundf(a * b);
    if (y == top) {
      c = cap ? foam : s.ix[C_CREST];
    } else if (y > top && y <= b0) {  // its face, white down from the top the harder it blows
      int down = y - top;
      bool white = cap && a >= 2 && down <= 1 + l.foam / 64 && b * 16 >= l.cap_from + 2 * down;
      c = white ? foam : near ? s.ix[C_ART + (int) art::S_SEA_FAR] : base;
    } else if (y == b0 + 1) {
      if (!near && a > 0)
        c = s.ix[C_ART + (int) art::S_SEA_NEAR];
    } else if (y < top && cap && hash8((uint32_t) x, (uint32_t) y, hs.sea_seed + 1) < 90u - (top - y) * 15u) {
      c = foam;  // spray off the crest
    }
  }
  return c;
}

void heath_row(const Scene &s, int y, uint8_t *out) {
  using namespace art;
  const Heath &h = s.heath;
  const uint8_t *ix = s.ix;
  for (int i = 0; i < h.nstreaks; i++) {  // the wind's streaks, a gentle wave along each
    int len = h.streak_len[i];
    for (int k = 0; k < len; k++) {
      int along = h.wind_dir > 0 ? k : len - 1 - k;  // 0 where the wind comes from
      if (along < len * 3 / 10 && (along / 2) % 2)
        continue;                                           // dashed at the start
      int curl = along >= len - 3 ? along - (len - 4) : 0;  // and a little hook at the end
      int x = h.streak_x[i] + k,
          ly = h.streak_y[i] + (int) lroundf(1.5f * sinf(k / 5.0f + h.streak_ph[i] / 40.0f)) - curl;
      if (ly == y && x >= 0 && x < W)
        out[x] = ix[C_STREAK];
    }
  }
  if (h.bft >= 11 && y < HZ && y >= HZ - 10 * (h.bft - 10))  // the air over the bay full of spray
    for (int x = 0; x <= h.hz_x1 + 6 && x < W; x++)
      if (hash8((uint32_t) x, (uint32_t) y, h.sea_seed + 4) < 50u - (HZ - y) * 2u)
        out[x] = ix[C_FOAM];
  if (y >= GROUND_Y0) {  // the landscape, the bay with its waves and the light on it, and the fog
    int dy = y - HZ, spread = 1 + dy / 9;
    uint32_t glints = 256;  // calm it's a column of light, rougher it breaks into glints
    bool shine = s.shine_x >= 0;
    if (h.bft == 0) {
      spread = dy / 12;
    } else if (h.bft <= 5) {
      shine = shine && (y & 1);
      if (h.bft >= 3) {
        spread = 1 + dy / 6;
        glints = 170;
      }
    } else {
      spread = 2 + dy / 4;
      glints = 70;
    }
    float g = 0.4f + 0.6f * dy / (H - HZ);  // nearer, more surf
    int surf = (int) lroundf(h.surf * g), surf_rock = (int) lroundf(h.surf_rock * g);
    auto sea = [](int k) { return k == S_SEA_FAR || k == S_SEA_NEAR; };
    auto rock = [](int k) { return k == S_ROCK_LIGHT || k == S_ROCK_DARK; };
    int r0 = GROUND_ROW[y - GROUND_Y0], r1 = GROUND_ROW[y - GROUND_Y0 + 1];
    for (int i = r0, x = 0; i < r1; i += 2) {
      int k = GROUND[i], x0 = x, end = x + GROUND[i + 1], d = DEPTH[k];
      int prev = i > r0 ? GROUND[i - 2] : -1, next = i + 2 < r1 ? GROUND[i + 2] : -1;
      for (; x < end; x++) {
        uint8_t c = k ? ix[C_ART + k] : out[x];
        if (k == S_BIRCH && h.birch_keep < 255 && hash8((uint32_t) x, (uint32_t) y, 5) >= h.birch_keep) {
          // through the bare twigs, what's behind
          int u = BIRCH_UNDER[(y - BIRCH_Y) * BIRCH_W + x - BIRCH_X];
          c = u ? ix[C_ART + u] : out[x];
        }
        if (sea(k)) {
          c = sea_px(s, x, y, c, k == S_SEA_NEAR);
          if (shine && c != ix[C_FOAM] && abs(x - s.shine_x) <= spread &&
              (glints > 255 || hash8((uint32_t) x, (uint32_t) y, h.sea_seed + 5) < glints))
            c = ix[C_SHINE];
          int shore = std::min(rock(next) ? end - 1 - x : 99, rock(prev) ? x - x0 : 99);
          if (shore < surf && hash8((uint32_t) shore, (uint32_t) y, h.sea_seed + 2) < 190)
            c = ix[C_FOAM];  // surf on the rocks
        } else if (rock(k) && surf_rock) {
          int shore = std::min(sea(prev) ? x - x0 : 99, sea(next) ? end - 1 - x : 99);
          if (shore < surf_rock && (int) hash8((uint32_t) x, (uint32_t) y, h.sea_seed + 3) < 110 - shore * 25)
            c = ix[C_FOAM];  // and spray up them
        } else if (k == S_WINDOW && h.indoors && x >= INDOORS_X && x < INDOORS_X + 7 && y >= INDOORS_Y &&
                   y < INDOORS_Y + 13 && (INDOORS[y - INDOORS_Y] >> (6 - (x - INDOORS_X)) & 1)) {
          c = ix[C_ART + (int) S_BLACK];
        } else if (!k && y > HZ - h.bumps && x >= h.hz_x0 && x <= h.hz_x1 &&
                   HZ - y < (int) (hash8((uint32_t) x / 3, 9, h.sea_seed) % (uint32_t) (h.bumps + 1))) {
          c = ix[C_ART + (int) S_SEA_FAR];  // the biggest waves break the horizon
        }
        if ((d <= 0 && y >= s.fog_top[0][x]) || (d <= 1 && y >= s.fog_top[1][x]))
          c = ix[C_FOG];
        out[x] = c;
      }
    }
  }
  int d = CHIMNEY_Y - 1 - y;  // the smoke, thinning out at the top, unless the animations' puffs are there
  if (!(s.live & L_SMOKE) && d >= 0 && d < h.nsmoke && (d < h.nsmoke * 2 / 3 || (d & 1)))
    span(out, h.smoke_x[d], h.smoke_x[d] + h.smoke_w - 1, ix[C_SMOKE]);
  if (s.pole_x >= 0 && y >= s.pole_top && y <= s.pole_base)  // the flagpole
    out[s.pole_x] = ix[C_ART + (int) S_BLACK];
  // the people, with the headlamp's glow, in the fog where it's thick enough. The show on the heath has them.
  for (int i = 0; i < h.nfig && !(s.live & L_HEATH); i++) {
    const Figure &f = *FIGURES[h.fig[i]];
    auto fogged = [&](int x) {
      return (f.depth <= 0 && y >= s.fog_top[0][x]) || (f.depth <= 1 && y >= s.fog_top[1][x]);
    };
    if (h.lamp && f.lamp_x >= 0) {
      int r = half(y - f.lamp_y, 3, 3);
      for (int x = std::max(0, f.lamp_x - r); r >= 0 && x <= std::min(W - 1, f.lamp_x + r); x++)
        if (!fogged(x))
          out[x] = ix[C_LAMP_GLOW];
    }
    if (y < f.y || y >= f.y + f.h)
      continue;
    const uint8_t *px = f.px + (y - f.y) * ((f.w + 3) / 4);
    for (int c = 0; c < f.w; c++) {
      int v = px[c / 4] >> (c % 4 * 2) & 3, x = f.x + c;
      if (v && x >= 0 && x < W && !fogged(x))  // the dog in the artboard's black, as it was drawn
        out[x] = v == 2 && h.lamp ? ix[C_LAMP] : h.fig[i] == F_DOG ? ix[C_ART + (int) S_BLACK] : ix[C_FIG];
    }
  }
}

void heath_drawn(const Scene &s, Colours &u) {
  const Heath &h = s.heath;
  for (int k = 1; k < art::SLOTS; k++)
    u.set(C_ART + k);
  u.set(C_CRATER, s.moon);
  u.set(C_HALO, s.moon && s.moon_glow);
  u.set(C_CREST, h.look.crest > 0);
  u.set(C_FOAM, h.look.cap || h.look.streak || h.surf > 0 || h.surf_rock > 0 || h.bft >= 11);
  u.set(C_SHINE, s.shine_x >= 0);
  u.set(C_STREAK, h.nstreaks > 0);
  bool out = h.nfig > 0 && !(s.live & L_HEATH);
  u.set(C_FIG, out);
  bool lamp = false;
  for (int i = 0; i < h.nfig; i++)
    lamp = lamp || FIGURES[h.fig[i]]->lamp_x >= 0;
  u.set(C_LAMP, out && h.lamp && lamp);
  u.set(C_LAMP_GLOW, out && h.lamp && lamp);
  u.set(C_SMOKE, h.nsmoke > 0 && !(s.live & L_SMOKE));
}

// ---------------------------------------------------------------- the characters for the show

namespace live {

// A pose, w x h: 0 nothing, 1 dark, 2 light (headlamp, rim, tusks), 3 the headlamp's glow
struct Pose {
  int w, h;
  std::vector<uint8_t> px;
  Pose(int w_, int h_) : w(w_), h(h_), px((size_t) (w_ * h_)) {}
  uint8_t at(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h ? px[(size_t) (y * w + x)] : 0; }
  void set(int x, int y, uint8_t v) {
    if (x >= 0 && y >= 0 && x < w && y < h)
      px[(size_t) (y * w + x)] = v;
  }
  // rows y0 to y1 - 1 one way, then put back dy rows down. Their pixels only, what's around stays.
  void move(int x0, int y0, int x1, int y1, int dx, int dy) {
    Pose was = *this;
    for (int y = y0; y < y1; y++)
      for (int x = x0; x < x1; x++)
        if (was.at(x, y))
          set(x, y, 0);
    for (int y = y0; y < y1; y++)
      for (int x = x0; x < x1; x++)
        if (uint8_t v = was.at(x, y))
          set(x + dx, y + dy, v);
  }
  // rows y0 to y1 - 1 mirrored about the middle of what's in them, a step of the legs
  void mirror_rows(int y0, int y1) {
    int l = w, r = -1;
    for (int y = y0; y < y1; y++)
      for (int x = 0; x < w; x++)
        if (at(x, y) == 1 || at(x, y) == 2)
          l = std::min(l, x), r = std::max(r, x);
    Pose was = *this;
    for (int y = y0; y < y1; y++)
      for (int x = l; x <= r; x++)
        set(x, y, was.at(l + r - x, y));
  }
  // everything up by n rows
  Pose up(int n) const {
    Pose p(w, h);
    for (int y = n; y < h; y++)
      for (int x = 0; x < w; x++)
        p.set(x, y - n, at(x, y));
    return p;
  }
};

// The poses as frames in colours pal[1..3]
static Frames frames_of(std::vector<Pose> poses, std::vector<uint16_t> delays, std::array<uint32_t, 4> pal) {
  Frames f;
  f.w = poses[0].w;
  f.h = poses[0].h;
  f.delays = std::move(delays);
  f.draw = [poses = std::move(poses), pal](int k, const Canvas &c) {
    const Pose &p = poses[(size_t) k % poses.size()];
    for (int y = 0; y < p.h; y++)
      for (int x = 0; x < p.w; x++)
        if (uint8_t v = p.at(x, y))
          c.put(x, y, pal[v]);
  };
  return f;
}

// The figure in a pose pad px in from the left and right, top px down
static Pose pose_of(const art::Figure &f, int pad, int top) {
  Pose p(f.w + 2 * pad, f.h + top + pad);
  for (int y = 0; y < f.h; y++) {
    const uint8_t *row = f.px + y * ((f.w + 3) / 4);
    for (int x = 0; x < f.w; x++)
      p.set(x + pad, y + top, (uint8_t) (row[x / 4] >> (x % 4 * 2) & 3));
  }
  return p;
}

// A bitmap of w-bit rows, leftmost pixel highest, as value v at x0, y0
static void put_bits(Pose &p, const uint32_t *rows, int w, int h, int x0, int y0, uint8_t v) {
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      if (rows[y] >> (w - 1 - x) & 1)
        p.set(x0 + x, y0 + y, v);
}

// The headlamp's glow under the figure, a disc of radius 3 round cx, cy
static void glow(Pose &p, int cx, int cy) {
  for (int dy = -3; dy <= 3; dy++) {
    int r = half(dy, 3, 3);
    for (int dx = -r; dx <= r; dx++)
      if (!p.at(cx + dx, cy + dy))
        p.set(cx + dx, cy + dy, 3);
  }
}

// A person in 4 frames: as drawn, a bob, the bottom legs rows a step on (above skis rows of skis, which stay), and
// a bob. The headlamp's glow in them while it's on.
static HeathSprite figure(const Scene &s, const art::Figure &f, int legs, int skis, int ms) {
  bool lamp = s.heath.lamp && f.lamp_x >= 0;
  int pad = lamp ? 3 : 0, top = pad + 1;  // a row for the bob
  Pose a = pose_of(f, pad, top), b = a;
  b.mirror_rows(top + f.h - skis - legs, top + f.h - skis);
  std::vector<Pose> poses = {a, a.up(1), b, b.up(1)};
  for (int k = 0; lamp && k < 4; k++)
    glow(poses[(size_t) k], f.lamp_x - f.x + pad, f.lamp_y - f.y + top - k % 2);
  uint32_t fig = s.rgb[C_FIG];
  return {frames_of(poses, {(uint16_t) ms, (uint16_t) ms, (uint16_t) ms, (uint16_t) ms},
                    {0, fig, lamp ? s.rgb[C_LAMP] : fig, s.rgb[C_LAMP_GLOW]}),
          f.x - pad, f.y - top, top + f.h};
}

// The kayak with its paddle at four angles through the paddler's hands, right blade in, flat, left, flat
static HeathSprite kayak(const Scene &s) {
  const art::Figure &f = art::KAYAKER;
  Pose body = pose_of(f, 0, 0);
  for (int y = 0; y < f.h; y++)  // the art's paddle and splash go, the hull, the paddler and the stern stay
    for (int x = 0; x < f.w; x++)
      if (y == 15 || (y <= 12 && !(x >= 11 && x <= 17 && y >= 5) && !(y == 12 && x >= 22)))
        body.set(x, y, 0);
  static const float DIR[4][2] = {{0.6f, 0.8f}, {0.95f, 0.3f}, {-0.6f, 0.8f}, {-0.95f, 0.3f}};
  std::vector<Pose> poses;
  for (const float *d : DIR) {
    Pose p = body;
    for (float t = -7.5f; t <= 7.5f; t += 0.25f) {
      int x = (int) lroundf(16.5f + t * d[0]), y = (int) lroundf(6 + t * d[1]);
      if (y <= 12)  // the blade under water from there
        p.set(x, y, 1);
    }
    int bx = (int) lroundf(16.5f + 7.5f * d[0]);  // a splash where the blade's in
    if (d[1] > 0.5f)
      p.set(bx, 15, 1);
    poses.push_back(p);
  }
  return {frames_of(poses, {320, 220, 320, 220}, {0, s.rgb[C_FIG], s.rgb[C_FIG], 0}), f.x, f.y, f.h - 1};
}

// The dog, facing left as drawn, in a gait. Only its legs, head and tail move, its feet stay on the ground. Two
// rows above it as room for the head.
static HeathSprite dog(const Scene &s, HeathKind k) {
  const art::Figure &f = art::DOG;
  Pose a = pose_of(f, 0, 2);
  std::vector<Pose> poses;
  std::vector<uint16_t> ms;
  if (k == HK_DOG_SIT) {  // its tail wags
    Pose b = a, c = a;
    b.set(7, 3, 0), b.set(7, 2, 1);
    c.set(7, 3, 0), c.set(8, 3, 1);
    poses = {a, b, a, c};
    ms = {160, 160, 160, 160};
  } else if (k == HK_DOG_TROT) {  // legs apart and together
    Pose b = a;
    b.move(3, 10, 5, 12, -1, 0);
    b.move(7, 10, 8, 12, 1, 0);
    poses = {a, b};
    ms = {150, 150};
  } else if (k == HK_DOG_GALLOP) {  // legs stretched out, then gathered under it
    Pose b = a, c = a;
    b.move(3, 9, 5, 12, -2, 0);
    b.move(7, 9, 8, 12, 1, 0);
    c.move(3, 10, 5, 12, 1, 0);
    c.move(7, 10, 8, 12, -1, 0);
    poses = {b, c};
    ms = {110, 110};
  } else {  // its nose down in the heather, sniffing
    Pose b = a, c = a;
    b.move(0, 2, 3, 5, 0, 2);
    c.move(0, 2, 3, 5, 0, 3);
    poses = {b, c, b, b};
    ms = {300, 200, 300, 500};
  }
  uint32_t black = s.rgb[C_ART + (int) art::S_BLACK];
  return {frames_of(poses, ms, {0, black, black, 0}), f.x, f.y - 2, f.h + 2};
}

// a reindeer's legs a step on, rows 8 to 10 of DEER
static const uint32_t DEER_STEP[3] = {0x04A, 0x08A, 0x10A};

static uint32_t ochre(const Scene &s) { return mix(0x4A2618, 0x9E4A2E, s.at.light); }

static HeathSprite deer(const Scene &s, HeathKind k) {
  Pose a(11, 11);
  put_bits(a, DEER, 11, 11, 0, 0, 1);
  std::vector<Pose> poses;
  std::vector<uint16_t> ms;
  if (k == HK_DEER_WALK) {
    Pose b(11, 11);
    put_bits(b, DEER, 11, 8, 0, 0, 1);
    put_bits(b, DEER_STEP, 11, 3, 0, 8, 1);
    poses = {a, b};
    ms = {260, 260};
  } else {  // its head down to the lichen and up a little, chewing
    Pose b = a, c = a;
    b.move(0, 0, 6, 5, 0, 3);
    c.move(0, 0, 6, 5, 0, 2);
    poses = {b, c, b, b};
    ms = {700, 400, 600, 900};
  }
  return {frames_of(poses, ms, {0, ochre(s), 0, 0}), art::TREES_X, art::DOOR_Y + 1 - 10, 11};
}

// The walrus facing left, coming up out of the water, head and tusks, and going down again
static HeathSprite walrus(const Scene &s) {
  std::vector<Pose> poses;
  for (int rows : {0, 1, 2, 3, 2, 1}) {
    Pose p(11, 5);
    put_bits(p, WALRUS + 0, 11, rows, 0, 3 - rows, 1);  // the top rows at the water line, the rest under it
    if (rows == 3)
      put_bits(p, TUSKS, 5, 2, 0, 3, 2);
    poses.push_back(p);
  }
  return {frames_of(poses, {1500, 500, 500, 2600, 500, 500}, {0, ochre(s), s.rgb[C_WHITE], 0}), 20, 160, 3};
}

HeathKind heath_mountain(const Scene &s) {
  if (s.heath.nfig < 1)
    return HK_NONE;
  return s.heath.fig[0] == F_RUNNER ? HK_RUN : s.heath.fig[0] == F_SKIER_CLIMBER ? HK_SKI : HK_CLIMB;
}

HeathKind heath_bay(const Scene &s) {
  if (s.heath.nfig < 2)
    return HK_NONE;
  return s.heath.fig[1] == F_RUNNER_KAYAKER  ? HK_RUN_SHORE
         : s.heath.fig[1] == F_SKIER_KAYAKER ? HK_SKI_SHORE
                                             : HK_KAYAK;
}

HeathSprite heath_sprite(const Scene &s, HeathKind k) {
  switch (k) {
    case HK_CLIMB:
      return figure(s, art::CLIMBER, 3, 0, 420);
    case HK_RUN:
      return figure(s, art::RUNNER, 3, 0, 140);
    case HK_SKI:
      return figure(s, art::SKIER_CLIMBER, 2, 1, 260);
    case HK_KAYAK:
      return kayak(s);
    case HK_RUN_SHORE:
      return figure(s, art::RUNNER_KAYAKER, 2, 0, 140);
    case HK_SKI_SHORE:
      return figure(s, art::SKIER_KAYAKER, 2, 1, 260);
    case HK_DEER_WALK:
    case HK_DEER_GRAZE:
      return deer(s, k);
    case HK_WALRUS:
      return walrus(s);
    default:
      return dog(s, k);
  }
}

}  // namespace live

// ---------------------------------------------------------------- fitting the panel

// How many bytes the picture packs into for the panel, as picture.h packs it: rows of 4-bit pixels, a run of three
// or more equal bytes as two bytes and the rest as they are
static size_t packed_size(const Scene &s) {
  size_t total = 0;
  uint8_t idx[W], b[W / 2];
  for (int y = 0; y < H; y++) {
    row(s, y, idx);
    for (int x = 0; x < W / 2; x++)
      b[x] = (uint8_t) (idx[2 * x] << 4 | idx[2 * x + 1]);
    for (int i = 0; i < W / 2;) {
      int run = 1;
      while (i + run < W / 2 && run < 129 && b[i + run] == b[i])
        run++;
      if (run >= 3) {
        total += 2;
        i += run;
        continue;
      }
      int len = 0;
      while (i < W / 2 && len < 128 && !(i + 2 < W / 2 && b[i] == b[i + 1] && b[i] == b[i + 2])) {
        i++;
        len++;
      }
      total += 1 + len;
    }
  }
  return total;
}

void heath_fit(Scene &s, const uint32_t *prev) {
  // first the foam streaks and spray go, then half the waves, then all of them and the surf, and the rain thins out
  Heath &h = s.heath;
  int nprec = s.nprec;
  for (int step = 0; step < 4 && packed_size(s) > PACK_MAX; step++) {
    s.nprec = nprec * (3 - std::min(step, 2)) / 4;
    if (step >= 1)
      h.look.streak = h.look.spray = 0;
    if (step == 2)
      h.look.crest /= 2;
    if (step == 3) {
      h.look.crest = 0;
      h.surf = h.surf_rock = 0;
      h.bumps = 0;
    }
    palette(s, prev);
  }
}

}  // namespace flexit::panel::met
