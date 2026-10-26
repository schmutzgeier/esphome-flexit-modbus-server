// The weather as a 320x240 picture in about 16 flat colours, drawn a row at a time: the cabin on the heath under
// the mountain, from the SVG layers in scripts/met_art (met_art.h). The sky, the light, the weather and the days
// work as they always did. Plain C++ without ESPHome, so sim/met_render.cpp in flexit-ci600-firmware can render it
// on a PC.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include "met_art.h"

namespace flexit::panel::met {

static constexpr int W = 320, H = 240;
static constexpr int HORIZON = 157;                        // the bay's far edge, just above the clock band
static constexpr int OV_H = 80;                            // rows of the forecast overlay, at the top
static constexpr int16_t BAND[5] = {0, 52, 92, 118, 138};  // first row of each sky band
static constexpr int RB_RX = 100, RB_RY = 96;              // the rainbow's outer radii
static constexpr int POLE_X = 276;                         // the flagpole on flag days, by the cabin
static constexpr int MOON_R = 9, SUN_R = 10;               // as drawn on the artboard
static constexpr int SEA_OV_X = 0, SEA_OV_Y = 218;         // where the wave height is written, low in the bay
static constexpr int SEA_OV_W = 64, SEA_OV_H = 22;
static constexpr int SEA_ROWS = 32;  // rows of waves on the bay at most

// ---------------------------------------------------------------- the forecast

enum Icon : uint8_t {
  I_NONE,
  I_SUN,
  I_MOON,
  I_PARTLY_SUN,
  I_PARTLY_MOON,
  I_CLOUD,
  I_RAIN,
  I_SNOW,
  I_SLEET,
  I_THUNDER,
  I_FOG
};

struct Hour {     // one hour of the forecast, for the overlay
  uint32_t time;  // UTC
  float temp;
  Icon icon;
  float wind;         // m/s
  int16_t wind_from;  // degrees
};

static constexpr int HOURS = 18;

struct Weather {
  bool ok = false;
  uint32_t time = 0;  // forecast hour, UTC seconds since 1970
  float temp = 10, cloud = 0, cloud_low = -1, cloud_mid = -1, cloud_high = -1, fog = 0;  // %, -1 = not given
  float rain = 0;                                                                        // mm in the coming hour
  float wind = 0, wind_from = 0, thunder = 0;                                            // m/s, degrees, %
  float wave = -1;        // significant wave height in m, from Oceanforecast (see Sea). -1 = none
  char symbol[40] = "";   // MET symbol for the coming hour, like "lightrainshowers_day"
  Hour hour[HOURS] = {};  // this hour onwards, as far as the response went
  int hours = 0;
};

// What the picture shows besides the weather. There's one landscape now, the artboard's; Ground is kept so
// met.yaml's Panel Weather Scene and older callers still build, and it changes nothing.
enum Ground : uint8_t { G_HILLS, G_COAST, G_MOUNTAINS, G_CITY };
enum Aurora : uint8_t { A_AUTO, A_OFF, A_ALWAYS };
struct Options {
  Ground ground = G_HILLS;  // not used
  Aurora aurora = A_AUTO;   // auto = some clear nights north of 55 degrees
  int heater_pct = 0;       // the unit's heater %, for the chimney smoke
  bool fireplace = false;   // more smoke
};

// MET symbol code to our Icon, see https://api.met.no/weatherapi/weathericon/2.0/documentation
inline Icon icon_of(const char *s) {
  bool night = strstr(s, "_night") != nullptr || strstr(s, "_polartwilight") != nullptr;
  if (strstr(s, "thunder"))
    return I_THUNDER;
  if (strstr(s, "sleet"))
    return I_SLEET;
  if (strstr(s, "snow"))
    return I_SNOW;
  if (strstr(s, "rain") || strstr(s, "drizzle"))
    return I_RAIN;
  if (strstr(s, "fog"))
    return I_FOG;
  if (strstr(s, "partlycloudy") || strstr(s, "fair"))
    return night ? I_PARTLY_MOON : I_PARTLY_SUN;
  if (strstr(s, "clearsky"))
    return night ? I_MOON : I_SUN;
  if (strstr(s, "cloudy"))
    return I_CLOUD;
  return I_NONE;
}

inline uint32_t days_from_civil(int y, int m, int d) {  // days since 1970-01-01
  y -= m <= 2;
  int era = (y >= 0 ? y : y - 399) / 400;
  int yoe = y - era * 400, doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  return (uint32_t) (era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468);
}

inline void civil_from_days(int32_t z, int *y, int *m, int *d) {  // days since 1970-01-01 to a date
  z += 719468;
  int era = (z >= 0 ? z : z - 146096) / 146097;
  int doe = z - era * 146097;
  int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int doy = doe - (365 * yoe + yoe / 4 - yoe / 100), mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = yoe + era * 400 + (*m <= 2);
}

inline int easter_day_of_year(int y) {  // Gregorian Easter Sunday as day of the year, 1 = Jan 1
  int a = y % 19, b = y / 100, c = y % 100, d = b / 4, e = b % 4, f = (b + 8) / 25, g = (b - f + 1) / 3;
  int h = (19 * a + b - d - g + 15) % 30, i = c / 4, k = c % 4, l = (32 + 2 * e + 2 * i - h - k) % 7;
  int m = (a + 11 * h + 22 * l) / 451, month = (h + l - 7 * m + 114) / 31, day = (h + l - 7 * m + 114) % 31 + 1;
  return (int) (days_from_civil(y, month, day) - days_from_civil(y, 1, 1)) + 1;
}

// Reads a MET Norway forecast in their JSON as it comes in, a step of its timeseries at a time, so it needs only a
// small buffer. D::step() gets each one, from its {"time" to the next, and says whether it wants more.
template<typename D, size_t N> struct Series {
  static constexpr size_t SIZE = N;
  char *buf;  // SIZE + 1 bytes
  size_t n = 0;
  bool series = false;  // past "timeseries":[

  explicit Series(char *b) : buf(b) { buf[0] = 0; }
  // Where the next bytes go, and how many fit
  char *space(size_t &room) {
    room = SIZE - n;
    return buf + n;
  }
  // n bytes came in at space(). False once it has all it wants, or what came can't be read.
  bool got(size_t k) {
    n += k;
    buf[n] = 0;
    if (!series) {
      const char *p = strstr(buf, "\"timeseries\":[");
      if (!p) {
        drop(n - std::min<size_t>(n, 15));  // the key may be cut in two
        return true;
      }
      series = true;
      drop((size_t) (p - buf));
    }
    for (;;) {  // each step once the next one starts, so the last one, maybe cut off, is never read
      const char *e = strstr(buf, "{\"time\"");
      const char *f = e ? strstr(e + 1, "{\"time\"") : nullptr;
      if (!f)
        return n < SIZE;
      if (!static_cast<D *>(this)->step(std::string_view(e, (size_t) (f - e))))
        return false;
      drop((size_t) (f - buf));
    }
  }

 private:
  void drop(size_t k) {
    memmove(buf, buf + k, n - k + 1);
    n -= k;
  }
};

// Reads a Locationforecast 2.0 answer. The first hour is the weather now, it and the ones after it are for the
// overlay. "complete" has the cloud layers and fog, "compact" doesn't.
struct Reader : Series<Reader, 4096> {  // an hour is about 1.2 KB
  Weather w;                            // w.ok once it has the first hour

  explicit Reader(char *b) : Series(b) {}
  // One hour, from its {"time" to the next one. False when no more are needed.
  bool step(std::string_view e) {
    auto at = [&](const char *key, size_t from = 0) -> size_t {
      char k[40];
      snprintf(k, sizeof k, "\"%s\":", key);
      size_t p = e.find(k, from);
      return p == std::string_view::npos ? p : p + strlen(k);
    };
    auto num = [&](const char *key, float &out, size_t from = 0) {
      size_t p = at(key, from);
      if (p != std::string_view::npos)
        out = strtof(e.data() + p, nullptr);
      return p != std::string_view::npos;
    };
    auto symbol = [&](size_t from, char *out, size_t size) {  // out stays "" without one
      size_t s = at("symbol_code", from);
      if (s == std::string_view::npos || e[s] != '"')
        return;
      size_t q = e.find('"', s + 1);
      if (q != std::string_view::npos && q - s - 1 < size)
        memcpy(out, e.data() + s + 1, q - s - 1);
    };
    int y, mo, d, h;
    float temp, wind = 0, from = 0;
    if (sscanf(e.data(), "{\"time\":\"%d-%d-%dT%d", &y, &mo, &d, &h) != 4 || !num("air_temperature", temp))
      return w.ok;  // a later hour is skipped, without the first there's no forecast
    uint32_t time = days_from_civil(y, mo, d) * 86400u + (uint32_t) h * 3600u;
    size_t n1 = at("next_1_hours");
    if (!w.ok) {
      if (!num("cloud_area_fraction", w.cloud))
        return false;
      w.time = time;
      w.temp = temp;
      num("cloud_area_fraction_low", w.cloud_low);
      num("cloud_area_fraction_medium", w.cloud_mid);
      num("cloud_area_fraction_high", w.cloud_high);
      num("fog_area_fraction", w.fog);
      num("wind_speed", w.wind);
      num("wind_from_direction", w.wind_from);
      size_t next = n1 != std::string_view::npos ? n1 : at("next_6_hours");
      if (next != std::string_view::npos) {
        num("precipitation_amount", w.rain, next);
        num("probability_of_thunder", w.thunder, next);
        symbol(next, w.symbol, sizeof w.symbol);
      }
      w.ok = true;
    }
    if (n1 != std::string_view::npos) {
      char sym[40] = "";
      symbol(n1, sym, sizeof sym);
      num("wind_speed", wind);
      num("wind_from_direction", from);
      w.hour[w.hours++] = {time, temp, icon_of(sym), wind, (int16_t) from};
    }
    return w.hours < HOURS;
  }
};

// The waves from Oceanforecast 2.0: the significant wave height, hourly from about now. MET Norway has them for
// points at sea, and points near the coast go to the nearest one; inland there are none.
static constexpr int SEA_HOURS = 24;
struct Sea {
  bool ok = false;
  int hours = 0;
  uint32_t time[SEA_HOURS] = {};  // UTC
  float height[SEA_HOURS] = {};   // m, the mean of the highest third of the waves
};

struct SeaReader : Series<SeaReader, 1024> {  // an hour is about 250 bytes
  Sea sea;

  explicit SeaReader(char *b) : Series(b) {}
  bool step(std::string_view e) {
    static const char KEY[] = "\"sea_surface_wave_height\":";
    int y, mo, d, h;
    size_t p = e.find(KEY);
    if (sscanf(e.data(), "{\"time\":\"%d-%d-%dT%d", &y, &mo, &d, &h) != 4 || p == std::string_view::npos)
      return true;  // an hour without them is skipped
    sea.time[sea.hours] = days_from_civil(y, mo, d) * 86400u + (uint32_t) h * 3600u;
    sea.height[sea.hours++] = std::max(0.0f, strtof(e.data() + p + sizeof KEY - 1, nullptr));
    sea.ok = true;
    return sea.hours < SEA_HOURS;
  }
};

// The wave height at t, UTC: the hour nearest it, if that's within three hours. -1 without one.
inline float wave_at(const Sea &sea, uint32_t t) {
  float best = -1;
  uint32_t gap = 3 * 3600 + 1800 + 1;
  for (int i = 0; i < sea.hours; i++) {
    uint32_t mid = sea.time[i] + 1800, d = t > mid ? t - mid : mid - t;
    if (d < gap) {
      gap = d;
      best = sea.height[i];
    }
  }
  return best;
}

// ---------------------------------------------------------------- positions of the sun, moon and stars

static constexpr double kPi = 3.14159265358979323846, kRad = kPi / 180;  // not PI, that's a macro in Arduino.h

struct Pos {
  float el, az;  // degrees above the horizon, degrees clockwise from north
};

inline double wrap360(double a) { return a - 360 * floor(a / 360); }

// Right ascension and declination in radians to elevation and azimuth, d days after J2000
inline Pos horizontal(double ra, double dec, double lat, double lon, double d) {
  double ha = wrap360(280.46061837 + 360.98564736629 * d + lon) * kRad - ra, la = lat * kRad;
  double el = asin(sin(la) * sin(dec) + cos(la) * cos(dec) * cos(ha));
  double az = atan2(sin(ha), cos(ha) * sin(la) - tan(dec) * cos(la)) / kRad + 180;  // from south + 180
  return {(float) (el / kRad), (float) wrap360(az)};
}

inline double j2000(uint32_t t) { return t / 86400.0 - 10957.5; }

inline double sun_longitude(double d) {  // ecliptic longitude in radians, good to 0.01 degrees
  double g = (357.529 + 0.98560028 * d) * kRad;
  return (280.459 + 0.98564736 * d + 1.915 * sin(g) + 0.020 * sin(2 * g)) * kRad;
}

inline Pos sun_at(double lat, double lon, uint32_t t) {
  double d = j2000(t), l = sun_longitude(d), e = (23.439 - 0.00000036 * d) * kRad;
  return horizontal(atan2(cos(e) * sin(l), cos(l)), asin(sin(e) * sin(l)), lat, lon, d);
}

// Moon position from Schlyter's elements and the largest perturbations, good to a degree or two.
// phase 0 new, 0.25 first quarter, 0.5 full, 0.75 last quarter
inline Pos moon_at(double lat, double lon, uint32_t t, float *phase) {
  double d = j2000(t) + 1.5;  // Schlyter's day 0 is 1999-12-31 0h
  double n = (125.1228 - 0.0529538083 * d) * kRad, i = 5.1454 * kRad, w = (318.0634 + 0.1643573223 * d) * kRad;
  double e = 0.0549, m = wrap360(115.3654 + 13.0649929509 * d) * kRad;
  double ea = m + e * sin(m) * (1 + e * cos(m));
  double x = cos(ea) - e, y = sqrt(1 - e * e) * sin(ea), v = atan2(y, x);
  double lon_e =
      atan2(sin(n) * cos(v + w) + cos(n) * sin(v + w) * cos(i), cos(n) * cos(v + w) - sin(n) * sin(v + w) * cos(i));
  double lat_e = asin(sin(v + w) * sin(i));
  double ms = (356.0470 + 0.9856002585 * d) * kRad, ls = ms + (282.9404 + 4.70935e-5 * d) * kRad;
  double lm = m + w + n, dd = lm - ls, f = lm - n;
  lon_e += (-1.274 * sin(m - 2 * dd) + 0.658 * sin(2 * dd) - 0.186 * sin(ms) - 0.059 * sin(2 * m - 2 * dd) -
            0.057 * sin(m - 2 * dd + ms) + 0.053 * sin(m + 2 * dd)) *
           kRad;
  lat_e += (-0.173 * sin(f - 2 * dd) - 0.055 * sin(m - f - 2 * dd) - 0.046 * sin(m + f - 2 * dd)) * kRad;
  if (phase)
    *phase = (float) (wrap360((lon_e - sun_longitude(j2000(t))) / kRad) / 360);
  double ob = (23.4393 - 3.563e-7 * d) * kRad;
  double xe = cos(lon_e) * cos(lat_e), ye = sin(lon_e) * cos(lat_e), ze = sin(lat_e);
  double yq = ye * cos(ob) - ze * sin(ob), zq = ye * sin(ob) + ze * cos(ob);
  return horizontal(atan2(yq, xe), atan2(zq, sqrt(xe * xe + yq * yq)), lat, lon, j2000(t));
}

// Bright stars. ra in hours, dec in degrees, bright 1 is drawn bigger
struct Star {
  float ra, dec;
  uint8_t bright;
};
// clang-format off: laid out by hand
static const Star STARS[] = {
    // 0 Polaris
    {2.530f, 89.26f, 0},
    // 1-7 the Big Dipper
    {11.062f, 61.75f, 1}, {11.031f, 56.38f, 0}, {11.897f, 53.69f, 0}, {12.257f, 57.03f, 0},
    {12.900f, 55.96f, 1}, {13.399f, 54.93f, 0}, {13.792f, 49.31f, 1},
    // 8-12 Cassiopeia
    {0.153f, 59.15f, 0}, {0.675f, 56.54f, 1}, {0.945f, 60.72f, 0}, {1.430f, 60.24f, 0}, {1.907f, 63.67f, 0},
    // 13 Vega, 14 Deneb, 15 Altair
    {18.616f, 38.78f, 1}, {20.690f, 45.28f, 1}, {19.846f, 8.87f, 1},
    // 16-19 Cygnus
    {20.370f, 40.26f, 0}, {19.512f, 27.96f, 0}, {20.770f, 33.97f, 0}, {19.750f, 45.13f, 0},
    // 20 Capella, 21 Aldebaran, 22 Pleiades
    {5.278f, 45.998f, 1}, {4.599f, 16.51f, 1}, {3.790f, 24.10f, 0},
    // 23-26 Orion: Betelgeuse, Rigel, Bellatrix, Saiph
    {5.919f, 7.41f, 1}, {5.242f, -8.20f, 1}, {5.419f, 6.35f, 0}, {5.796f, -9.67f, 0},
    // 27-29 Orion's belt
    {5.679f, -1.94f, 0}, {5.604f, -1.20f, 0}, {5.533f, -0.30f, 0},
    // 30 Sirius, 31 Procyon, 32 Castor, 33 Pollux
    {6.752f, -16.72f, 1}, {7.655f, 5.22f, 1}, {7.577f, 31.89f, 0}, {7.755f, 28.03f, 1},
    // 34 Regulus, 35 Denebola, 36 Arcturus, 37 Spica
    {10.139f, 11.97f, 1}, {11.818f, 14.57f, 0}, {14.261f, 19.18f, 1}, {13.420f, -11.16f, 1},
    // 38 Antares, 39 Mirfak, 40 Algol
    {16.490f, -26.43f, 1}, {3.405f, 49.86f, 0}, {3.136f, 40.96f, 0},
    // 41-43 Andromeda
    {0.140f, 29.09f, 0}, {1.162f, 35.62f, 0}, {2.065f, 42.33f, 0},
    // 44-46 Pegasus, with Alpheratz
    {23.063f, 28.08f, 0}, {23.079f, 15.21f, 0}, {0.220f, 15.18f, 0},
    // 47 Hamal, 48 Elnath, 49 Menkalinan, 50 Kochab
    {2.120f, 23.46f, 0}, {5.438f, 28.61f, 0}, {5.992f, 44.95f, 0}, {14.845f, 74.16f, 0},
};
// clang-format on
// ---------------------------------------------------------------- colours
// All colours are 4 bits per channel, so the panel's 16-colour palette holds them exactly.

// RGB rounded to 4 bits per channel, multiples of 17
inline uint32_t q(int r, int g, int b) {
  auto c = [](int v) { return (uint32_t) ((v < 0 ? 0 : v > 255 ? 255 : v) + 8) / 17 * 17; };
  return c(r) << 16 | c(g) << 8 | c(b);
}
inline uint32_t mix(uint32_t a, uint32_t b, float t) {
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  auto ch = [&](int s) { return (int) ((a >> s & 255) + ((int) (b >> s & 255) - (int) (a >> s & 255)) * t + 0.5f); };
  return q(ch(16), ch(8), ch(0));
}
inline float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }

// ---------------------------------------------------------------- the sea

// The Beaufort scale as in MET Norway's wind and wave table: each force's top wind speed in m/s, and the height of a
// fully grown sea at it in m. A bay's waves are lower, but the picture goes by the table when Oceanforecast has none.
static constexpr float BFT_WIND[12] = {0.2f, 1.5f, 3.3f, 5.4f, 7.9f, 10.7f, 13.8f, 17.1f, 20.7f, 24.4f, 28.4f, 32.6f};
static constexpr float BFT_WAVE[13] = {0, 0.1f, 0.2f, 0.6f, 1, 2, 3, 4, 5.5f, 7, 9, 11.5f, 14};
inline int beaufort(float wind) {
  float v = roundf(wind * 10) / 10;  // the table goes in tenths
  int b = 0;
  while (b < 12 && v > BFT_WIND[b])
    b++;
  return b;
}

// How the sea looks at each force: the chance an arch of the waves has a crest, that it breaks white and from how
// far down its top (out of 16), the chance of foam down a breaking wave's face and of a streak of blown foam, and
// how many rows of spray come off the crests. Chances are out of 256.
struct SeaLook {
  uint8_t crest, cap, cap_from, foam, streak, spray;
};
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
inline uint32_t hash8(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u ^ (c + 0x165667B1u) * 0xC2B2AE3Du;
  h ^= h >> 15;
  h *= 0x2C1B3C6Du;
  h ^= h >> 12;
  return h >> 24;
}

// ---------------------------------------------------------------- the scene

struct Ellipse {
  int16_t x, y, rx, ry;
};

struct Cloud {
  int16_t x, top, base, shade;  // x is the left edge in the wrapping field, rows below shade are darker
  Ellipse e[5];                 // x relative to the cloud's x
  uint8_t n;
};

// A one-colour bitmap. Layer 0 is drawn in the sky behind the ground, 1 in front of it, 2 on top of everything.
struct Sprite {
  int16_t x, y;
  uint8_t w, h, layer;  // at most 32 by 24
  uint32_t c;
  uint32_t rows[24];  // leftmost pixel in the highest of w bits
};
static constexpr int SPRITES = 56;

// Everything build() works out, so row() can draw any row on its own
struct Scene {
  uint32_t sky[5];  // sky bands from BAND, top to the horizon
  bool glow;        // sunrise or sunset light around the sun, low in the sky
  int16_t glow_x, glow_rx, glow_ry;
  uint32_t glow_c, glow_outer_c;
  bool sun, sun_rings, moon, moon_glow;
  int16_t sun_x, sun_y, moon_x, moon_y;
  float moon_phase;
  uint32_t sun_c, ring1_c, ring2_c, moon_c, crater_c, moon_glow_c, moon_halo_c;
  int nstars;  // faint background stars at made-up positions
  int16_t star_x[90], star_y[90];
  uint32_t star_c, dim_c;  // catalogue stars, background stars
  int nbright;             // catalogue stars, at their real positions
  int16_t bright_x[64], bright_y[64];
  uint8_t bright_big[64];
  bool aurora;            // the artboard's curtain, in art::AURORA's three bands
  uint32_t aur_c[3];      // its faint top, its body, its bright lower edge
  int nclouds, drift[3];  // drift is each layer's offset in the wrapping field
  Cloud cloud[22];
  uint8_t cloud_layer[22];
  uint32_t cloud_lit[3], cloud_shade[3];  // high, middle, low
  bool rainbow;
  int16_t rb_x, rb_y;
  uint32_t rb_c[3];
  int nprec;
  int16_t prec_x[220], prec_y[220];
  uint8_t prec_kind[220];  // 0 rain streak, 1 snowflake, 2 big snowflake
  int16_t prec_top[W];     // where it starts falling in each column
  int8_t slant;            // rain's x per 8 rows
  uint32_t rain_c, snow_c;
  bool bolt;
  int16_t bolt_x[7], bolt_y[7];
  uint32_t bolt_c;
  int16_t fog_top[2][W];  // over the mountain, and over the far land and the bay when it's thick. H = none
  uint32_t fog_c;
  uint32_t land[art::SLOTS];  // the landscape's colour for each of met_art.h's slots, now
  bool keep_ridge;            // the mountain's dark ridge band keeps its own colour
  uint8_t birch_keep;         // the chance out of 256 a pixel of the birch's crown has leaves or twigs, 255 all
  bool keep_sea;              // and the bay its two, for the waves, and the cabin its wall
  int16_t shine_x;            // the sun's or moon's reflection on the bay, -1 = none
  uint32_t shine_c;
  int nfig;  // the people: on the mountain and in the kayak, running, or skiing, and the dog
  const art::Figure *fig[3];
  bool indoors;  // the three of them behind the big window instead
  bool lamp;     // the headlamp's on
  uint32_t fig_c, lamp_c, lamp_glow_c;
  bool window_lit;
  uint32_t window_c;
  int nsmoke;  // the chimney's smoke, a wisp nsmoke rows tall
  int16_t smoke_x[48];
  uint8_t smoke_w;
  uint32_t smoke_c;
  uint32_t white_c;             // for small things, borrowed from a lit cloud or the sun
  int16_t pole_top, pole_base;  // the flagpole by the cabin, else pole_top = H
  int nstreaks;                 // wind streaks in the air above the bay, more and longer the harder it blows
  int16_t streak_x[5], streak_y[5];
  uint8_t streak_len[5], streak_ph[5];
  int8_t wind_dir;  // 1 it blows to the right, -1 to the left
  uint32_t streak_c;
  // The bay by the Beaufort scale: rows of waves far to near with crests as high as the waves are, and white caps,
  // foam streaks, spray and surf the harder it blows
  int bft, nsea;
  int16_t sea_y[SEA_ROWS];                                       // each row's baseline
  uint8_t sea_a[SEA_ROWS], sea_len[SEA_ROWS], sea_ph[SEA_ROWS];  // its crests' height and spacing in px, offset
  SeaLook look;                                                  // SEA_LOOK for bft
  float swell;               // 0 .. 1, how much higher the waves are than the wind makes them: long crests
  int bumps;                 // rows the biggest waves break the horizon by
  float surf, surf_rock;     // how far surf reaches out from the rocks, and spray up them, in px at the bottom
  int16_t hz_x0, hz_x1;      // the bay's far edge
  uint32_t sea_seed;         // new each hour
  uint32_t crest_c, foam_c;  // the waves' backs, and white water
  uint8_t sea_ov[SEA_OV_H][SEA_OV_W / 4];  // the wave height written in the bay, as ov
  // The two biggest parts are the caller's, kept apart so the scene needs no big block
  int nsprites;
  Sprite *sprite;        // SPRITES of them
  bool overlay;          // the forecast drawn over the picture
  uint8_t (*ov)[W / 4];  // OV_H rows, 2 bits per pixel. 0 shows the picture, 1 outline, 2 text
  uint32_t text_c, outline_c;
  uint32_t scratch[W];  // a row, for counting the colours
};

static constexpr int FIELD = W + 200;  // clouds drift through a field wider than the screen

struct Rng {  // xorshift32
  uint32_t s;
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  int in(int a, int b) { return a + (int) (next() % (uint32_t) (b - a + 1)); }
  float f() { return (next() & 0xFFFFFF) / 16777216.0f; }
};

inline bool has(const Weather &w, const char *s) { return strstr(w.symbol, s) != nullptr; }

// ---------------------------------------------------------------- the forecast over the picture

// 5x7 font, leftmost pixel in the highest of w bits. '^' is the degree sign.
struct Glyph {
  char c;
  uint8_t w, row[7];
};
// clang-format off: laid out by hand
static const Glyph FONT[] = {
    {'0', 5, {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', 5, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', 5, {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', 5, {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', 5, {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', 5, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', 5, {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', 5, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', 5, {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', 5, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {'-', 4, {0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}}, {'.', 2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03}},
    {'^', 4, {0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00}}, {'m', 5, {0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15}},
    {'/', 5, {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10}}, {'s', 5, {0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E}},
    {' ', 2, {0}}};
// clang-format on

// 9x9 icons, leftmost pixel in bit 8
static const uint16_t ICON_SUN[9] = {0x010, 0x082, 0x038, 0x07C, 0x17D, 0x07C, 0x038, 0x082, 0x010};
static const uint16_t ICON_MOON[9] = {0x03C, 0x078, 0x0F0, 0x0E0, 0x0E0, 0x0E0, 0x0F0, 0x078, 0x03C};
static const uint16_t ICON_CLOUD[9] = {0x000, 0x000, 0x038, 0x07C, 0x0FE, 0x1FF, 0x1FF, 0x0FE, 0x000};
static const uint16_t ICON_RAIN[9] = {0x038, 0x07C, 0x0FE, 0x1FF, 0x0FE, 0x000, 0x092, 0x124, 0x000};
static const uint16_t ICON_SNOW[9] = {0x038, 0x07C, 0x0FE, 0x1FF, 0x0FE, 0x000, 0x092, 0x000, 0x124};
static const uint16_t ICON_SLEET[9] = {0x038, 0x07C, 0x0FE, 0x1FF, 0x0FE, 0x000, 0x092, 0x124, 0x049};
static const uint16_t ICON_THUNDER[9] = {0x038, 0x07C, 0x0FE, 0x1FF, 0x0FE, 0x030, 0x060, 0x030, 0x060};
static const uint16_t ICON_FOG[9] = {0x000, 0x1FE, 0x000, 0x0FF, 0x000, 0x1FE, 0x000, 0x0FF, 0x000};

// The overlay's byte for x, y: the forecast at the top, or the wave height in the bay. Null elsewhere.
inline const uint8_t *ov_at(const Scene &s, int x, int y) {
  if (x >= 0 && x < W && y >= 0 && y < OV_H)
    return &s.ov[y][x / 4];
  if (x >= SEA_OV_X && x < SEA_OV_X + SEA_OV_W && y >= SEA_OV_Y && y < SEA_OV_Y + SEA_OV_H)
    return &s.sea_ov[y - SEA_OV_Y][(x - SEA_OV_X) / 4];
  return nullptr;
}
inline int ov_get(const Scene &s, int x, int y) {
  const uint8_t *b = ov_at(s, x, y);
  return b ? *b >> (x % 4 * 2) & 3 : 0;
}
inline void ov_put(Scene &s, int x, int y, int v) {
  if (uint8_t *b = const_cast<uint8_t *>(ov_at(s, x, y)))
    *b = (uint8_t) ((*b & ~(3 << (x % 4 * 2))) | v << (x % 4 * 2));
}

// A bitmap of w-bit rows scaled k times. Pass 1 draws its outline, pass 2 the bitmap.
inline void ov_bitmap(Scene &s, int x0, int y0, const uint16_t *rows, int w, int h, int k, int pass) {
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++) {
      if (!(rows[r] >> (w - 1 - c) & 1))
        continue;
      for (int dy = 0; dy < k; dy++)
        for (int dx = 0; dx < k; dx++) {
          int x = x0 + c * k + dx, y = y0 + r * k + dy;
          if (pass == 2)
            ov_put(s, x, y, 2);
          else  // two pixels round, so it reads over a white cloud too
            for (int oy = -2; oy <= 2; oy++)
              for (int ox = -2; ox <= 2; ox++)
                if (abs(ox) + abs(oy) < 4)
                  ov_put(s, x + ox, y + oy, 1);
        }
    }
}

inline int ov_text_width(const char *t, int k) {
  int w = 0;
  for (; *t; t++)
    for (const Glyph &g : FONT)
      if (g.c == *t)
        w += (g.w + 1) * k;
  return w > 0 ? w - k : 0;
}

inline void ov_text(Scene &s, int x, int y, const char *t, int k) {
  for (int pass = 1; pass <= 2; pass++) {  // all outlines first, so they don't cut into the letters
    int cx = x;
    for (const char *p = t; *p; p++)
      for (const Glyph &g : FONT)
        if (g.c == *p) {
          uint16_t rows[7];
          for (int r = 0; r < 7; r++)
            rows[r] = g.row[r];
          ov_bitmap(s, cx, y, rows, g.w, 7, k, pass);
          cx += (g.w + 1) * k;
        }
  }
}

inline void ov_icon(Scene &s, int x, int y, Icon i, int k) {
  auto one = [&](const uint16_t *b, int dx, int dy) {
    for (int pass = 1; pass <= 2; pass++)
      ov_bitmap(s, x + dx * k, y + dy * k, b, 9, 9, k, pass);
  };
  switch (i) {
    case I_SUN:
      one(ICON_SUN, 0, 0);
      break;
    case I_MOON:
      one(ICON_MOON, 0, 0);
      break;
    case I_PARTLY_SUN:
      one(ICON_SUN, -1, -2);
      one(ICON_CLOUD, 2, 1);
      break;  // a cloud in front of it
    case I_PARTLY_MOON:
      one(ICON_MOON, -1, -2);
      one(ICON_CLOUD, 2, 1);
      break;
    case I_CLOUD:
      one(ICON_CLOUD, 0, 0);
      break;
    case I_RAIN:
      one(ICON_RAIN, 0, 0);
      break;
    case I_SNOW:
      one(ICON_SNOW, 0, 0);
      break;
    case I_SLEET:
      one(ICON_SLEET, 0, 0);
      break;
    case I_THUNDER:
      one(ICON_THUNDER, 0, 0);
      break;
    case I_FOG:
      one(ICON_FOG, 0, 0);
      break;
    default:
      break;
  }
}

inline int round_temp(float v) { return (int) (v < 0 ? v - 0.5f : v + 0.5f); }

// Which way the wind blows, an arrow 9 by 9 drawn k times as big: a shaft and a solid head. from is where it comes
// from, clockwise from north.
inline void ov_arrow(Scene &s, int x, int y, float from, int k) {
  uint16_t rows[9] = {};
  float a = (from + 180) * (float) kRad, ux = sinf(a), uy = -cosf(a);  // the way it points, up is north
  for (int r = 0; r < 9; r++)
    for (int c = 0; c < 9; c++) {
      float px = c - 4.0f, py = r - 4.0f, along = px * ux + py * uy, across = fabsf(px * uy - py * ux);
      bool shaft = along >= -4.2f && along <= 1 && across <= 0.6f;
      bool head = along >= 0 && along <= 4.2f && across <= (4.2f - along) * 0.8f;
      if (shaft || head)
        rows[r] = (uint16_t) (rows[r] | 1 << (8 - c));
    }
  for (int pass = 1; pass <= 2; pass++)
    ov_bitmap(s, x, y, rows, 9, 9, k, pass);
}

// Now at top left with the rain and the wind, the next four 3-hour marks in local time at top right, each with
// its wind. utc_offset is local time minus UTC in seconds.
inline void forecast(Scene &s, const Weather &w, uint32_t t, int utc_offset) {
  char b[16];
  Icon now = w.hours ? w.hour[0].icon : icon_of(w.symbol);
  if (now == I_NONE)
    now = icon_of(w.symbol);
  ov_icon(s, 6, 7, now, 3);
  snprintf(b, sizeof b, "%d^", round_temp(w.temp));
  ov_text(s, 46, 6, b, 4);
  int wy = 42;  // the rain, then the wind: an arrow the way it blows and its speed
  if (w.rain >= 0.1f) {
    snprintf(b, sizeof b, "%.1f mm", w.rain);
    ov_text(s, 8, 42, b, 2);
    wy = 60;
  }
  ov_arrow(s, 8, wy - 2, w.wind_from, 2);
  snprintf(b, sizeof b, "%d m/s", (int) lroundf(w.wind));
  ov_text(s, 30, wy, b, 2);
  int col = 0;
  for (int i = 0; i < w.hours && col < 4; i++) {
    const Hour &h = w.hour[i];
    int local = (int) ((h.time + (uint32_t) (utc_offset + 86400)) % 86400 / 3600);
    if (h.time <= t || local % 3)
      continue;
    int cx = W - 6 - (4 - col) * 40 + 20;  // middle of the column
    snprintf(b, sizeof b, "%02d", local);
    ov_text(s, cx - ov_text_width(b, 2) / 2, 6, b, 2);
    ov_icon(s, cx - 9, 24, h.icon, 2);
    snprintf(b, sizeof b, "%d^", round_temp(h.temp));
    ov_text(s, cx - ov_text_width(b, 2) / 2 + 2, 46, b, 2);
    ov_arrow(s, cx - 17, 66, h.wind_from, 1);  // and the wind then
    snprintf(b, sizeof b, "%d", (int) lroundf(h.wind));
    ov_text(s, cx - 5, 64, b, 2);
    col++;
  }
}

// ---------------------------------------------------------------- sprites

inline Sprite *sprite(Scene &s, int x, int y, int w, int h, int layer, uint32_t c) {
  if (s.nsprites >= SPRITES)
    return nullptr;
  Sprite &p = s.sprite[s.nsprites++];
  p = Sprite{};
  p.x = (int16_t) x;
  p.y = (int16_t) y;
  p.w = (uint8_t) (w > 32 ? 32 : w);
  p.h = (uint8_t) (h > 24 ? 24 : h);
  p.layer = (uint8_t) layer;
  p.c = c;
  return &p;
}

inline void sprite_rows(Sprite *p, const uint32_t *rows) {
  if (p)
    for (int r = 0; r < p->h; r++)
      p->rows[r] = rows[r];
}

inline void sprite_set(Sprite *p, int x, int y) {
  if (p && x >= 0 && x < p->w && y >= 0 && y < p->h)
    p->rows[y] |= 1u << (p->w - 1 - x);
}

// a filled disc as a sprite
inline Sprite *blob(Scene &s, int cx, int cy, int rx, int ry, int layer, uint32_t c) {
  if (rx > 15 || ry > 11)
    return nullptr;
  Sprite *p = sprite(s, cx - rx, cy - ry, 2 * rx + 1, 2 * ry + 1, layer, c);
  if (!p)
    return nullptr;
  for (int dy = -ry; dy <= ry; dy++) {
    float v = 1 - (float) (dy * dy) / (float) (ry * ry + 0.01f);
    int h = (int) (rx * sqrtf(v < 0 ? 0 : v) + 0.5f);
    for (int dx = -h; dx <= h; dx++)
      sprite_set(p, dx + rx, dy + ry);
  }
  return p;
}

inline Sprite *box(Scene &s, int x, int y, int w, int h, int layer, uint32_t c) {
  Sprite *p = sprite(s, x, y, w, h, layer, c);
  if (p)
    for (int r = 0; r < p->h; r++)
      p->rows[r] = p->w == 32 ? 0xFFFFFFFFu : (1u << p->w) - 1;
  return p;
}

// a firework of rays around a bright middle
inline void burst(Scene &s, int cx, int cy, int r, uint32_t c, uint32_t heart) {
  Sprite *p = sprite(s, cx - r, cy - r, 2 * r + 1, 2 * r + 1, 0, c);
  for (int k = 0; k < 12; k++) {
    float a = k * 6.2832f / 12;
    for (int d = 3; d <= r; d++)
      if ((d + k) % 3 != 2)  // gaps so the rays look like they're burning out
        sprite_set(p, r + (int) lroundf(d * cosf(a)), r + (int) lroundf(d * sinf(a)));
  }
  Sprite *in = sprite(s, cx - 2, cy - 2, 5, 5, 0, heart);
  for (int d = -2; d <= 2; d++) {
    sprite_set(in, 2 + d, 2);
    sprite_set(in, 2, 2 + d);
  }
}

// The Norwegian flag, 22 by 16 in the official 6-1-2-1-12 by 6-1-2-1-6 proportions, on layer 2
inline void flag_norway(Scene &s, int x, int y, uint32_t red, uint32_t white, uint32_t blue) {
  Sprite *r = sprite(s, x, y, 22, 16, 2, red), *w = sprite(s, x, y, 22, 16, 2, white),
         *b = sprite(s, x, y, 22, 16, 2, blue);
  for (int row = 0; row < 16; row++)
    for (int col = 0; col < 22; col++) {
      bool vb = col == 7 || col == 8, vw = col == 6 || col == 9, hb = row == 7 || row == 8, hw = row == 6 || row == 9;
      sprite_set(vb || hb ? b : vw || hw ? w : r, col, row);
    }
}

// The rainbow flag for Pride, 22 by 18 in six stripes, on layer 2
inline void flag_pride(Scene &s, int x, int y) {
  static const uint32_t STRIPES[6] = {0xE40303, 0xFF8C00, 0xFFED00, 0x008026, 0x24408E, 0x732982};
  for (int k = 0; k < 6; k++)
    box(s, x, y + 3 * k, 22, 3, 2, q(STRIPES[k] >> 16, STRIPES[k] >> 8 & 255, STRIPES[k] & 255));
}

// The vimpel, a red pennant 9 rows tall at the pole and len long, with a white-blue-white stripe. Points left
// when dir < 0.
inline void pennant(Scene &s, int x, int y, int len, int dir, uint32_t red, uint32_t white, uint32_t blue) {
  Sprite *r = sprite(s, x, y, len, 9, 2, red), *w = sprite(s, x, y, len, 9, 2, white),
         *b = sprite(s, x, y, len, 9, 2, blue);
  for (int col = 0; col < len; col++) {
    int hi = (int) (4.0f * (1 - (float) col / len) + 0.5f);  // half height at this column
    for (int row = 4 - hi; row <= 4 + hi; row++) {
      int d = abs(row - 4);  // the stripe while there's room for it, the tip all red
      sprite_set(d == 0 && hi >= 1 ? b : d == 1 && hi >= 2 ? w : r, dir < 0 ? len - 1 - col : col, row);
    }
  }
}

// Sprite bitmaps, one entry per row, leftmost pixel in the highest of w bits
// clang-format off: laid out by hand
static const uint32_t BIRD[3] = {0x41, 0x36, 0x8};                               // 7 wide
static const uint32_t BAT[4] = {0x183, 0x1EF, 0xFE, 0x28};                        // 9 wide
static const uint32_t FLAME[20] = {0x100, 0x100, 0x300, 0x380, 0x780, 0x7A0, 0xFA0, 0xFE0, 0x1FE0, 0x1FF0,   // 15 wide
                                   0x1FF0, 0x3FF8, 0x3FF8, 0x3FFC, 0x7FFC, 0x7FFE, 0x7FFF, 0x7FFF, 0x3FFE, 0x1FFC};
static const uint32_t FLAME_IN[20] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x100, 0x100, 0x300, 0x380,
                                      0x780, 0x780, 0x7C0, 0xFC0, 0xFC0, 0xFE0, 0x1FE0, 0x1FE0, 0xFE0, 0x7C0};
static const uint32_t STAR7[7] = {0x08, 0x1C, 0x7F, 0x3E, 0x3E, 0x63, 0x41};     // 7 wide
// the pumpkin's face, 11 wide
static const uint32_t FACE[7] = {0x104, 0x38E, 0x7FF, 0x0, 0x555, 0x6DB, 0x3FE};
// a reindeer as in the rock carvings, facing left, 11 wide
static const uint32_t DEER[11] = {0x540, 0x380, 0x100, 0x380, 0x680, 0x0FE, 0x0FF, 0x07E, 0x052, 0x092, 0x091};
// a walrus swimming, facing left: its head and back at the water, 11 wide, and its tusks, 5 wide
static const uint32_t WALRUS[3] = {0x3C0, 0x7E2, 0x7FF};
static const uint32_t TUSKS[2] = {0x009, 0x009};
// clang-format on

// ---------------------------------------------------------------- the sky

// Zenith, horizon and glow colours for a sun elevation. Night is the artboard's sky. The dark ones keep red and
// green close, or at 4 bits per channel they come out teal or purple.
inline void sky_colours(float el, uint32_t *top, uint32_t *bottom, uint32_t *glow) {
  // clang-format off: laid out by hand
  static const struct { float el; uint32_t top, bottom, glow; } K[] = {
      {-90, 0x000822, 0x1E5288, 0x1E5288}, {-18, 0x000822, 0x1E5288, 0x1E5288},
      {-12, 0x0C1636, 0x2A4870, 0x3C4670}, {-6, 0x16225A, 0x544680, 0xA85C7A},
      {-2, 0x22357A, 0x8E6488, 0xF0885A},  {2, 0x2A4994, 0xD88C6C, 0xFFAE60},
      {8, 0x3263B2, 0xEABC90, 0xFFD48E},   {16, 0x3277CC, 0xA6CEEC, 0xE6EEF0},
      {90, 0x2868C4, 0xB2D8F2, 0xFFFFFF}};
  // clang-format on
  int k = 0;
  while (k < 7 && el > K[k + 1].el)
    k++;
  float t = clamp01((el - K[k].el) / (K[k + 1].el - K[k].el));
  *top = mix(K[k].top, K[k + 1].top, t);
  *bottom = mix(K[k].bottom, K[k + 1].bottom, t);
  *glow = mix(K[k].glow, K[k + 1].glow, t);
}

inline int sky_x(float az) { return (int) (az / 360 * W) % W; }  // north at the edges, south in the middle
inline int sky_y(float el) { return HORIZON - (int) (el * (HORIZON - 26) / 55); }
inline int band_of(int y) { return y < BAND[1] ? 0 : y < BAND[2] ? 1 : y < BAND[3] ? 2 : y < BAND[4] ? 3 : 4; }

// 0 winter with snow on the ground, 1 winter without, 2 spring, 3 summer, 4 autumn
inline int season_of(int month, int day, bool white) {
  if (white)
    return 0;
  int k = month * 100 + day;
  if (k < 320 || k >= 1201)
    return 1;
  if (k < 601)
    return 2;
  if (k < 901)
    return 3;
  return 4;
}

// The light, weather and date the parts of the scene go by. utc_offset is local time minus UTC in seconds.
struct Now {
  const Weather &w;
  const Options &o;
  double lat, lon;
  uint32_t t, local;  // UTC, and local time with a day added so it's never negative
  Pos sun, moon;
  float moon_phase, light, fog, cover, grey, glow;  // light 0 night .. 1 day, grey how overcast, glow see sky()
  bool snow, rain, thunder, dark, white;            // white: snow on the ground
  bool awake;                                       // from six to midnight, the cabin's light can be on
  bool indoors;                                     // rain over 4 mm in the hour, or the night: everyone's in
  int year, month, day, hour, season, wdir;         // wdir: which way the wind blows things
  int bft;                                          // the wind's Beaufort force
  float wave;                                       // wave height in m: Oceanforecast's, or the table's for bft
  bool xmas, newyear, easter_days, midsummer, halloween, pride, flag_day, flag_up;

  Now(const Weather &weather, const Options &options, double la, double lo, uint32_t utc, int utc_offset)
      : w(weather), o(options), lat(la), lon(lo), t(utc), local(utc + (uint32_t) (utc_offset + 86400)) {
    sun = sun_at(lat, lon, t);
    moon = moon_at(lat, lon, t, &moon_phase);
    light = clamp01((sun.el + 6) / 18);
    snow = has(w, "snow") || has(w, "sleet");
    rain = has(w, "rain") || has(w, "sleet") || has(w, "drizzle");
    if (w.ok && w.rain > 0.05f && !snow && !rain)
      (w.temp > 1 ? rain : snow) = true;
    thunder = has(w, "thunder") || w.thunder >= 30;
    fog = std::max(w.fog, has(w, "fog") ? 85.0f : 0.0f);
    cover = w.ok ? w.cloud : 0;
    grey = clamp01((cover - 45) / 50) * 0.9f + (rain || snow ? 0.1f : 0) + (thunder ? 0.15f : 0);
    glow = (1 - fabsf(sun.el - 1) / 11) * (1 - grey);
    dark = sun.el < -6;
    wdir = w.wind_from > 0 && w.wind_from < 180 ? -1 : 1;
    bft = w.ok ? beaufort(w.wind) : 1;  // without the weather, a little light on the water as it always was
    wave = w.wave >= 0 ? w.wave : BFT_WAVE[bft];
    hour = (int) (local % 86400 / 3600);
    int minutes = (int) (local % 86400 / 60);
    awake = minutes >= 6 * 60;                                                    // asleep from midnight to six
    indoors = (rain && w.ok && w.rain > 4) || !awake || minutes >= 23 * 60 + 30;  // home from half past eleven
    civil_from_days((int32_t) (local / 86400) - 1, &year, &month, &day);
    white =
        (w.ok && w.temp <= 0 && snow) || (w.ok && w.temp <= -3) || (w.ok && w.temp <= 1 && (month == 12 || month <= 2));
    season = season_of(month, day, white);
    int doy = (int) (days_from_civil(year, month, day) - days_from_civil(year, 1, 1)) + 1,
        easter = easter_day_of_year(year);
    xmas = month == 12 && day >= 24 && day <= 26;
    newyear = (month == 12 && day == 31 && hour >= 18) || (month == 1 && day == 1 && hour < 3);
    easter_days = doy >= easter - 3 && doy <= easter + 1;
    midsummer = month == 6 && day == 23 && hour >= 19;
    halloween = month == 10 && day == 31;
    pride = month == 6 && day == 28;  // International Pride Day: the rainbow flag
    // Official flag days from forskrift om bruk av statsflagget § 4. New Year's Day, Princess Ingrid Alexandra,
    // the Sami people's day, the King, 1 May, Liberation Day, Constitution Day, the union's dissolution, the
    // Queen, the Crown Prince, Olsok, the Crown Princess, Christmas Day, Easter Sunday, Pentecost, and the
    // Storting election on the second Monday of September every fourth year
    int weekday = (int) ((local / 86400 - 1 + 4) % 7);  // 0 Sunday. local has a day added
    flag_day = (month == 5 && day == 17) || (month == 1 && day == 1) || (month == 1 && day == 21) ||
               (month == 2 && day == 6) || (month == 2 && day == 21) || (month == 5 && day == 1) ||
               (month == 5 && day == 8) || (month == 6 && day == 7) || (month == 7 && day == 4) ||
               (month == 7 && day == 20) || (month == 7 && day == 29) || (month == 8 && day == 19) ||
               (month == 12 && day == 25) || doy == easter || doy == easter + 49 ||
               (year % 4 == 1 && month == 9 && weekday == 1 && day >= 8 && day <= 14);
    // Up at 8 from March to October and at 9 in winter, down at sunset or by 21. From Nordland and north it's
    // 10 to 15 in winter, § 6.
    bool winter_months = month <= 2 || month >= 11;
    flag_up = (flag_day || pride) &&
              (lat >= 65 && winter_months ? hour >= 10 && hour < 15
                                          : hour >= (winter_months ? 9 : 8) && hour < 21 && sun.el > -0.5f);
  }
};

// The sky's bands, the fog's colour and the glow around a low sun
inline void sky(Scene &s, const Now &n) {
  uint32_t top, bottom, glow;
  sky_colours(n.sun.el, &top, &bottom, &glow);
  uint32_t grey_top = mix(0x14171F, n.thunder ? 0x5C626C : 0x8A94A2, n.light),
           grey_bottom = mix(0x20242C, n.thunder ? 0x7A808A : 0xBEC4CC, n.light);
  top = mix(top, grey_top, n.grey);
  bottom = mix(bottom, grey_bottom, n.grey);
  float darken = std::min(0.4f, (n.w.ok ? n.w.rain : 0) / 12);
  top = mix(top, 0x2A2F38, darken);
  bottom = mix(bottom, 0x4A505A, darken);
  s.fog_c = mix(mix(0x262C38, 0xC6CCD4, n.light), bottom, 0.3f);
  if (n.fog >= 60) {
    top = mix(top, s.fog_c, (n.fog - 60) / 80);
    bottom = mix(bottom, s.fog_c, (n.fog - 60) / 60);
  }
  static const float AT[5] = {0, 0.3f, 0.55f, 0.78f, 1};
  for (int i = 0; i < 5; i++)
    s.sky[i] = mix(top, bottom, AT[i]);
  float g = n.glow;
  if (g > 0.15f) {
    s.glow = true;
    s.glow_x = (int16_t) sky_x(n.sun.az);
    s.glow_rx = (int16_t) (90 + 60 * g);
    s.glow_ry = (int16_t) (16 + 22 * g);
    s.glow_c = mix(s.sky[4], glow, 0.3f + 0.45f * g);
    s.glow_outer_c = mix(s.sky[3], glow, 0.15f + 0.25f * g);
  }
}

// The sun with two rings of light, and the artboard's moon: a pale disc with its maria and a soft glow
inline void sun_moon(Scene &s, const Now &n) {
  if (n.sun.el > -1) {
    s.sun = true;
    s.sun_x = (int16_t) sky_x(n.sun.az);
    s.sun_y = (int16_t) sky_y(n.sun.el);
    uint32_t behind = s.sky[band_of(std::max(0, (int) s.sun_y))];
    if (n.grey > 0.75f) {
      s.sun_c = mix(behind, 0xFFFFFF, 0.3f);  // a pale disc through the clouds
    } else {
      s.sun_c = n.sun.el < 6 ? mix(0xFF9A50, 0xFFE0A0, n.sun.el / 6) : 0xFFF6D2;
      s.sun_rings = true;
      s.ring1_c = mix(behind, s.sun_c, 0.42f);
      s.ring2_c = mix(behind, s.sun_c, 0.2f);
    }
  }
  if (n.moon.el > 1 && n.sun.el < 10) {
    s.moon = true;
    s.moon_x = (int16_t) sky_x(n.moon.az);
    s.moon_y = (int16_t) sky_y(n.moon.el);
    s.moon_phase = n.moon_phase;
    s.moon_c = mix(0xDCE6F2, 0xF4F6FA, 1 - n.light);
    s.crater_c = mix(s.moon_c, 0x93A1AD, 0.5f);
    if (n.sun.el < -6 && n.grey < 0.6f) {
      s.moon_glow = true;
      uint32_t behind = s.sky[band_of(std::max(0, (int) s.moon_y))];
      s.moon_glow_c = mix(behind, s.moon_c, 0.16f);
      s.moon_halo_c = mix(behind, s.moon_c, 0.07f);
    }
  }
}

// Northern lights, some clear nights up north: the artboard's curtain, in one of three colourings
inline void aurora(Scene &s, const Now &n) {
  if (n.o.aurora == A_OFF || n.sun.el >= -9 || n.cover >= 40 || n.fog >= 30)
    return;
  Rng a{(uint32_t) ((n.local - 43200) / 86400) * 2246822519u + 3};  // from noon to noon, the same all night
  if (n.o.aurora != A_ALWAYS && (n.lat < 55 || a.f() >= (n.lat >= 65 ? 0.6f : 0.35f)))
    return;
  s.aurora = true;
  for (int k = 0; k < 3; k++)  // the draws the old ribbons' shapes took, so a night keeps its colouring
    a.f();
  a.in(40, 120);
  a.in(W - 60, W + 80);
  // top, body, lower edge: mostly green under violet, some nights teal under pink, now and then green under red
  // The artboard's is teal (0x2C7B7C) with a mint edge (0x7DCEB7) under a dim blue haze: the first keeps it,
  // a touch greener, with the haze turned violet
  static const uint32_t HUES[3][3] = {
      {0x241E4C, 0x2C8274, 0x84D8B6}, {0x3E2046, 0x2C7A86, 0x8AD4D4}, {0x401C26, 0x3A8452, 0xA0DCA4}};
  const uint32_t *hue = HUES[a.f() < 0.6f ? 0 : a.f() < 0.6f ? 1 : 2];
  for (int k = 0; k < 3; k++)
    s.aur_c[k] = mix(hue[k], hue[k], 0);  // rounded to 4 bits per channel
}

// Faint random stars, and the catalogue ones where they really are
inline void stars(Scene &s, const Now &n) {
  if (n.sun.el >= -7 || n.grey >= 0.85f)
    return;
  Rng r{0x5EED5u};
  s.nstars = (int) ((n.sun.el < -12 ? 90 : 40) * (1 - n.cover / 100));
  for (int i = 0; i < s.nstars; i++) {
    s.star_x[i] = (int16_t) r.in(0, W - 1);
    s.star_y[i] = (int16_t) r.in(2, 132);
  }
  s.star_c = 0xFFFFFF;  // same white as the forecast text
  s.dim_c = s.aurora ? s.star_c : mix(s.sky[1], 0xE8EEFA, 0.4f);
  double d = j2000(n.t);
  for (unsigned i = 0; i < sizeof STARS / sizeof *STARS && n.cover < 70 && s.nbright < 64; i++) {
    Pos p = horizontal(STARS[i].ra * 15 * kRad, STARS[i].dec * kRad, n.lat, n.lon, d);
    if (p.el < 1 || p.el > 58)
      continue;
    s.bright_x[s.nbright] = (int16_t) sky_x(p.az);
    s.bright_y[s.nbright] = (int16_t) std::max(3, sky_y(p.el));
    s.bright_big[s.nbright++] = STARS[i].bright;
  }
}

// Clouds, seeded by the forecast hour so they don't jump about between redraws
inline void clouds(Scene &s, const Now &n) {
  const Weather &w = n.w;
  float layer[3] = {w.cloud_high, w.cloud_mid, w.cloud_low};
  if (layer[0] < 0 || layer[1] < 0 || layer[2] < 0) {
    layer[0] = n.cover * 0.25f;
    layer[1] = n.cover * 0.45f;
    layer[2] = n.cover * 0.6f;
  }
  bool wet = n.rain || n.snow;
  uint32_t lit = mix(0x353C56, wet ? 0xC4CAD2 : 0xF6F8FB, n.light),
           shade = mix(0x232839, wet ? 0x8E96A2 : 0xC6CEDA, n.light);
  if (n.thunder) {
    lit = mix(lit, 0x5E646E, 0.6f);
    shade = mix(shade, 0x3A3F48, 0.6f);
  }
  if (s.glow) {  // lit from below at sunrise and sunset
    lit = mix(lit, 0xFFD0A4, 0.75f * n.glow);
    shade = mix(shade, 0x86668A, 0.6f * n.glow);
  }
  for (int l = 0; l < 3; l++) {
    s.cloud_lit[l] = l == 0 ? mix(lit, s.sky[0], 0.25f) : lit;
    s.cloud_shade[l] = l == 2 && n.grey > 0.3f ? mix(shade, 0x404650, 0.25f * n.grey) : shade;
  }
  Rng r{w.time * 2654435761u + 7};
  uint32_t into_hour = w.ok && n.t > w.time ? n.t - w.time : n.t % 3600;
  for (int l = 0; l < 3; l++) {
    s.drift[l] = (int) (into_hour / 60.0f * (0.3f + w.wind * 0.12f) * (l + 1)) % FIELD;
    int count = (int) (layer[l] / 100 * (l == 0 ? 5 : 7) + 0.5f);
    for (int k = 0; k < count && s.nclouds < 22; k++) {
      Cloud &c = s.cloud[s.nclouds];
      s.cloud_layer[s.nclouds++] = (uint8_t) l;
      int width = l == 0 ? r.in(60, 120) : l == 1 ? r.in(48, 86) : r.in(70, 128);
      c.x = (int16_t) ((k * FIELD) / count + r.in(-20, 20) + FIELD) % FIELD;
      c.base = (int16_t) (l == 0 ? r.in(20, 64) : l == 1 ? r.in(62, 102) : r.in(100, 146));
      if (l == 0) {  // high ones are thin streaks
        c.n = (uint8_t) r.in(2, 3);
        for (int j = 0; j < c.n; j++)
          c.e[j] = {(int16_t) (width * (j + 1) / (c.n + 1) + r.in(-8, 8)), (int16_t) (c.base - r.in(0, 6)),
                    (int16_t) (width / (c.n + 1) + r.in(4, 14)), (int16_t) r.in(1, 3)};
        c.top = (int16_t) (c.base - 12);
        c.shade = 999;
        c.base += 4;
        continue;
      }
      c.n = (uint8_t) r.in(3, 5);
      int rmax = width / c.n + 4;
      c.top = c.base;
      for (int j = 0; j < c.n; j++) {
        int middle = 2 * j - (c.n - 1);  // bigger towards the middle
        int rad = rmax - abs(middle) * 3 + r.in(-3, 3);
        c.e[j] = {(int16_t) (width * (2 * j + 1) / (2 * c.n)), (int16_t) (c.base - rad * 6 / 10), (int16_t) rad,
                  (int16_t) rad};
        c.top = (int16_t) std::min<int>(c.top, c.e[j].y - rad);
      }
      c.shade = (int16_t) (c.base - (c.base - c.top) * 26 / 100);
    }
  }
}

// The rainbow, rain and snow from under the clouds, not through them, and the bolt from there to the ridge
inline void precipitation(Scene &s, const Now &n) {
  const Weather &w = n.w;
  if (n.rain && !n.snow && n.sun.el > 1 && n.sun.el < 42 && n.grey < 0.8f) {  // with the sun out and low enough
    s.rainbow = true;
    int x = sky_x(n.sun.az + 180);  // opposite the sun, kept whole on the screen
    s.rb_x = (int16_t) (x < RB_RX ? RB_RX : x > W - 1 - RB_RX ? W - 1 - RB_RX : x);
    s.rb_y = (int16_t) (HORIZON + n.sun.el * 1.6f);
    uint32_t behind = s.sky[2];
    s.rb_c[0] = mix(behind, 0xE05050, 0.55f);
    s.rb_c[1] = mix(behind, 0xE8D850, 0.55f);
    s.rb_c[2] = mix(behind, 0x6090E0, 0.55f);
    s.sun_rings = false;
  }
  if (n.rain || n.snow || n.thunder) {
    int deck = 0;  // lowest base of the middle and low clouds, for the columns between them
    for (int x = 0; x < W; x++)
      s.prec_top[x] = -1;
    for (int i = 0; i < s.nclouds; i++) {  // under each cloud, from its own base
      const Cloud &c = s.cloud[i];
      if (s.cloud_layer[i] == 0)
        continue;  // not from the high thin ones
      deck = std::max<int>(deck, c.base);
      int x0 = (c.x + s.drift[s.cloud_layer[i]]) % FIELD - 100;
      for (int j = 0; j < c.n; j++)
        for (int k = -1; k <= 0; k++) {
          int cx = x0 + c.e[j].x + k * FIELD;
          for (int x = std::max(0, cx - c.e[j].rx); x <= std::min(W - 1, cx + c.e[j].rx); x++)
            s.prec_top[x] = (int16_t) std::max<int>(s.prec_top[x], c.base + 2);
        }
    }
    for (int x = 0; x < W; x++)
      if (s.prec_top[x] < 0)
        s.prec_top[x] = (int16_t) (deck ? deck : 110);
  }
  if (n.rain || n.snow) {
    Rng p{w.time + 99};
    float amount = std::max(w.rain, has(w, "heavy") ? 4.0f : has(w, "light") ? 0.3f : 1.0f);
    s.nprec = (int) std::min(220.0f, 30 + amount * 45);
    float across = -n.wdir * std::min(1.0f, w.wind / 9);
    s.slant = (int8_t) (across * 4);
    bool both = has(w, "sleet");
    for (int i = 0; i < s.nprec; i++) {
      int x = p.in(0, W - 1);
      s.prec_x[i] = (int16_t) x;
      s.prec_y[i] = (int16_t) p.in(s.prec_top[x], H - 1);
      s.prec_kind[i] = (uint8_t) (n.snow && (!both || i % 2) ? (p.in(0, 3) ? 1 : 2) : 0);
    }
    s.rain_c = mix(0x4A5C78, 0x9EBAD6, n.light);
    s.snow_c = mix(0xAAB4C6, 0xF6FAFF, n.light);
  }
  if (n.thunder) {
    Rng b{w.time + 5};
    s.bolt = true;
    int x = b.in(70, 250), top = s.prec_top[x] - 3,
        step = std::max(6, (art::RIDGE[x] + 4 - top) / 6);  // out of the cloud, to the ridge
    for (int i = 0; i < 7; i++) {
      s.bolt_x[i] = (int16_t) x;
      s.bolt_y[i] = (int16_t) (top + i * step);
      x += (i % 2 ? 1 : -1) * b.in(5, 13);
    }
    s.bolt_c = q(255, 248, 196);
  }
}

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
// out in land().
inline uint32_t day_colour(int slot) {
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

// The landscape's colour for each slot: night to day with the light, by season, then the sky's haze on the
// mountain, the low sun's glow, how overcast and wet it is, and the fog
inline void land(Scene &s, const Now &n) {
  using namespace art;
  int seasonal[SLOTS];
  for (int k = 0; k < SLOTS; k++)
    seasonal[k] = -1;
  for (int l = 0; l < L_N; l++)
    seasonal[SEASONAL_SLOT[l]] = l;
  // lit in the dark and with everyone in, until they go to bed at midnight; at Christmas and New Year all night
  s.window_lit = (n.sun.el < -3 || n.indoors) && (n.awake || n.xmas || n.newyear);
  s.keep_ridge = n.light > 0 && n.fog < 60;  // by day the mountain stands out, unless it's in the fog
  s.keep_sea = n.light > 0 && n.fog < 60;
  s.window_c = n.halloween ? q(255, 140, 40) : q(255, 200, 120);
  bool wet = n.rain && !n.white;
  // The birch leafs out in mid May, is summer green from mid June, yellow from mid September and loses its last
  // leaves by late October. Up north it's later in spring and earlier in autumn, two days a degree.
  int doy = (int) (days_from_civil(n.year, n.month, n.day) - days_from_civil(n.year, 1, 1)) + 1,
      late = (int) lroundf((float) (n.lat - 60) * 2);
  int birch = doy < 130 + late || doy >= 296 - late ? 0
              : doy < 165 + late                    ? 1
              : doy < 255 - late                    ? 2
              : doy < 282 - late                    ? 3
                                                    : 4;
  s.birch_keep = birch == 0 ? 90 : birch == 4 ? 140 : 255;  // bare it's mostly what's behind
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
      c = mix(c, s.sky[4], 0.08f * n.light);
    else if (k == S_HEATH_FAR || k == S_GRANITE_FAR || k == S_SEA_FAR)
      c = mix(c, s.sky[4], 0.1f * n.light);
    if (s.glow && depth <= 1 && k != S_SEA_FAR && k != S_SEA_NEAR)  // alpenglow
      c = mix(c, 0xFF9070, 0.2f * n.glow);
    if (n.grey > 0.2f) {  // flat light under cloud
      int y = (int) ((c >> 16) * 3 + (c >> 8 & 255) * 6 + (c & 255)) / 10;
      c = mix(c, q(y, y, y + 6), 0.3f * n.grey);
    }
    if (wet && depth >= 1 && k != S_SEA_FAR && k != S_SEA_NEAR && k != S_WINDOW)
      c = mix(c, 0x000000, 0.12f);
    if (n.fog > 15)
      c = mix(c, s.fog_c, n.fog / 100 * (0.5f - depth * 0.15f));
    s.land[k] = c;
  }
  s.land[S_MTN_RIM] = s.land[S_MTN_A];  // no aurora glow on the ridge: it stands dark against the sky
  s.land[S_HEATHER] = (n.month == 8 || n.month == 9) && !n.white
                          ? mix(mix(NIGHT[S_HEATHER], 0x8E5E90, n.light), s.land[S_HEATH_NEAR], n.fog / 200)
                          : s.land[S_HEATH_NEAR];  // in bloom
  s.land[S_COTTON] =
      (n.month == 6 || n.month == 7) && !n.white ? mix(NIGHT[S_COTTON], 0xECEEE8, n.light) : s.land[S_HEATH_NEAR];
  if (n.white)  // the sheep are in for the winter
    s.land[S_SHEEP] = s.land[S_SHEEP_DARK] = s.land[S_HEATH_MID];
  s.land[S_BIRCH_TWIG] = s.birch_keep < 255 ? s.land[S_BLACK] : s.land[S_BIRCH];  // its branches show when bare
  s.land[S_SPILL] = s.window_lit ? mix(s.land[S_HEATH_NEAR], 0xFFB45A, 0.3f) : s.land[S_HEATH_NEAR];
  s.land[S_WINDOW] = s.window_lit ? s.window_c : mix(s.land[S_BLACK], mix(s.sky[2], 0x2A3440, 0.55f), n.light);
  // white for small things, borrowed so it doesn't cost one of the 16 colours
  bool mid_low = false, high = false;
  for (int i = 0; i < s.nclouds; i++)
    (s.cloud_layer[i] ? mid_low : high) = true;
  s.white_c = mid_low ? s.cloud_lit[1] : high ? s.cloud_lit[0] : s.sun ? s.sun_c : s.cloud_lit[1];
  // By day small things share a colour with something like them, so the 16 go to the sky and the land's shapes.
  // At night they keep the artboard's own.
  if (n.light > 0) {
    if (!n.white) {
      s.land[S_SHEEP] = s.white_c;
      s.land[S_SHEEP_DARK] = s.land[S_BLACK];
      s.land[S_JUNIPER] = s.land[S_HEATH_FORE];
    }
    if (s.land[S_COTTON] != s.land[S_HEATH_NEAR])
      s.land[S_COTTON] = s.white_c;
    s.land[S_GRANITE_FAR] = s.land[S_PATH] = s.land[S_GRANITE];
    s.land[S_BIRCH_BARK] = s.white_c;
    s.land[S_CABIN_TRIM] = s.land[S_ROCK_LIGHT];
    if (!s.window_lit)
      s.land[S_WINDOW] = s.land[S_SEA_FAR];
  }
  // fog over the mountain, and over the far land and the bay when it's thick
  Rng h{(uint32_t) (int32_t) (n.lat * 1000) * 31u + (uint32_t) (int32_t) (n.lon * 1000) + 1};
  float ph = h.f() * 6.283f;
  for (int f = 0; f < 2; f++)
    for (int x = 0; x < W; x++) {
      bool on = f == 0 ? n.fog > 15 : n.fog > 45;
      int edge = f == 0 ? HORIZON - 14 - (int) (n.fog * 0.1f) : 178 - (int) (n.fog * 0.12f);
      s.fog_top[f][x] = (int16_t) (on ? edge + (int) (3 * sinf(x / 17.0f + ph + f)) : H);
    }
  // calm, the bay's a mirror of the sky
  if (n.bft == 0 && n.w.ok)
    s.land[S_SEA_FAR] = mix(s.land[S_SEA_FAR], s.sky[4], 0.15f * n.light);
  // the sun's or moon's light on the bay
  s.shine_x = -1;
  if (s.moon && n.dark)
    s.shine_x = s.moon_x;
  else if (s.sun && n.sun.el < 20 && n.grey < 0.5f)
    s.shine_x = s.sun_x;
  s.shine_c = mix(s.land[S_SEA_FAR], 0xFFFFFF, 0.3f + 0.25f * n.light);
  // The bay by the Beaufort scale, new each hour. Rows of waves from the far edge, their crests as high as the waves
  // are, smaller and closer together further off; how much of them breaks white goes by the force. In the dark
  // only the light on the water shows small waves.
  const Weather &w = n.w;
  s.bft = n.bft;
  s.look = SEA_LOOK[w.ok ? n.bft : 0];
  s.look.crest = (uint8_t) std::max<int>(s.look.crest, std::min(200, (int) (n.wave * 80)));  // swell without wind
  if (n.light <= 0 && n.bft < 4 && n.wave < 1)
    s.look.crest = 0;
  s.wind_dir = (int8_t) n.wdir;
  s.sea_seed = (uint32_t) (n.t / 3600) * 7919u + 11;
  Rng sr{s.sea_seed};
  s.swell = clamp01((n.wave - BFT_WAVE[n.bft]) / 2);
  float amp = std::max(0.0f, 2.7f * sqrtf(n.wave) - 0.6f),  // at the bottom, px
      len = (10 + 7 * powf(n.wave, 0.6f)) * (1 + 0.8f * s.swell);
  for (int y = HORIZON + 2; y < H && s.nsea < SEA_ROWS;) {
    float p = (y - HORIZON) / (float) (H - HORIZON), g = 0.25f + 0.75f * p;
    int a = (int) lroundf(amp * g);
    s.sea_y[s.nsea] = (int16_t) y;
    s.sea_a[s.nsea] = (uint8_t) a;
    s.sea_len[s.nsea] = (uint8_t) std::max(4, (int) lroundf(len * (0.45f + 0.55f * p)));
    s.sea_ph[s.nsea++] = (uint8_t) sr.in(0, 255);
    y += std::max(2, a + 2 + (int) lroundf(2.5f * g));
  }
  s.bumps = n.wave >= 7 ? 2 : n.wave >= 3.5f ? 1 : 0;
  s.surf = n.wave >= 1 ? std::min(5.0f, n.wave * 0.5f) + (n.bft >= 6 ? 1 : 0) : n.bft >= 5 ? 1 : 0;
  s.surf_rock = n.wave >= 3 ? std::min(4.0f, (n.wave - 2) * 0.6f) : 0;
  s.hz_x0 = W;
  s.hz_x1 = -1;
  for (int i = GROUND_ROW[HORIZON + 1 - GROUND_Y0], x = 0; i < GROUND_ROW[HORIZON + 2 - GROUND_Y0]; i += 2) {
    if (GROUND[i] == S_SEA_FAR || GROUND[i] == S_SEA_NEAR) {
      s.hz_x0 = (int16_t) std::min<int>(s.hz_x0, x);
      s.hz_x1 = (int16_t) (x + GROUND[i + 1] - 1);
    }
    x += GROUND[i + 1];
  }
  // the sky in the waves' backs: the band a little lighter than the water
  auto luma = [](uint32_t c) { return (int) ((c >> 16) * 3 + (c >> 8 & 255) * 6 + (c & 255)) / 10; };
  s.crest_c = s.sky[4];
  for (int i = 4; i >= 0; i--)
    if (luma(s.sky[i]) >= luma(s.land[S_SEA_FAR]) + 25 && luma(s.sky[i]) < luma(s.crest_c))
      s.crest_c = s.sky[i];
  s.foam_c = n.light > 0.5f && s.white_c != s.sun_c ? s.white_c : s.shine_c;  // not the sun's yellow
  // and streaks in the air above it, from 3 m/s by day and 10 in the dark: dashed where the wind comes from,
  // curling where it goes
  if (w.wind >= 3 && (n.light > 0 || w.wind >= 10)) {
    Rng st{(uint32_t) (n.t / 3600) * 104729u + 5};
    s.nstreaks = std::min(5, (int) (w.wind / 3));
    for (int i = 0; i < s.nstreaks; i++) {
      int len = std::min(44, 10 + (int) (w.wind * 2));
      s.streak_len[i] = (uint8_t) len;
      s.streak_x[i] = (int16_t) st.in(0, std::max(0, 96 - len));
      s.streak_y[i] = (int16_t) (98 + i * 11 + st.in(-3, 3));
      s.streak_ph[i] = (uint8_t) st.in(0, 255);
    }
    s.streak_c = n.light > 0.5f ? s.white_c : mix(s.sky[3], 0xFFFFFF, 0.3f + 0.3f * n.light);
  }
}

// The two of them and the dog: climbing the mountain, paddling the bay, and on the hill by the cabin. When it rains
// the climber runs the ridge instead, with waves over 2 m or wind over 12 m/s the kayaker comes ashore and runs,
// with snow both are out on skis, and in rain over 4 mm all three are in the cabin, behind its big window. From
// half past eleven to six they're all home, behind the window until its light goes out at midnight.
inline void people(Scene &s, const Now &n) {
  if (n.indoors) {
    s.indoors = s.window_lit;  // seen only with the light on
    return;
  }
  bool skis = n.white || (n.snow && !n.rain), run = !skis && n.rain, ashore = !skis && (n.w.wind > 12 || n.wave > 2);
  s.fig[s.nfig++] = skis ? &art::SKIER_CLIMBER : run ? &art::RUNNER : &art::CLIMBER;
  s.fig[s.nfig++] = skis ? &art::SKIER_KAYAKER : ashore ? &art::RUNNER_KAYAKER : &art::KAYAKER;
  s.fig[s.nfig++] = &art::DOG;
  s.lamp = n.sun.el < -3;  // the headlamp's on when it's getting dark
  s.fig_c = mix(0x070B11, 0x1A1E26, n.light);
  s.lamp_c = q(247, 249, 250);
  s.lamp_glow_c = mix(s.sky[3], 0xDBE6EC, 0.35f);
}

// Birds, bats, the chimney's smoke and the flag
inline void life(Scene &s, const Now &n, Rng &rng) {
  const Weather &w = n.w;
  bool fair = n.sun.el > 3 && n.cover < 60 && !n.rain && !n.snow && !n.thunder && n.fog < 30 && w.wind < 12;
  if (fair && !n.halloween) {
    int count = rng.in(2, 5), bx = rng.in(20, W - 60), by = rng.in(40, 100);
    for (int i = 0; i < count; i++)
      sprite_rows(sprite(s, bx + i * 11 + rng.in(-2, 2), by + (i % 2 ? 4 : 0) + rng.in(-1, 1), 7, 3, 0, s.fig_c), BIRD);
  }
  if (n.halloween && n.dark) {  // bats, around the moon if it's up, else low where the sky is lighter
    int bx = s.moon ? s.moon_x - 40 : rng.in(40, W - 100), by = s.moon ? s.moon_y - 16 : rng.in(70, 110);
    for (int i = 0; i < 7; i++)
      sprite_rows(sprite(s, bx + rng.in(-10, 90), by + rng.in(-20, 36), 9, 4, 0, s.land[art::S_BLACK]), BAT);
  }
  // Chimney smoke while the heater's on, more in fireplace mode: a wisp curling up from the chimney as on the
  // artboard, taller the more there is
  int puffs = n.o.fireplace ? 5 : n.o.heater_pct > 0 ? 2 + std::min(3, n.o.heater_pct / 30) : 0;
  if (n.midsummer)
    puffs = 0;
  s.nsmoke = puffs ? std::min(48, 6 + puffs * 8) : 0;
  s.smoke_w = (uint8_t) (n.o.fireplace ? 2 : 1);
  s.smoke_c = n.light > 0.5f ? s.white_c : mix(s.sky[2], 0xD8E2EA, 0.4f + 0.25f * n.light);
  float ph = rng.f() * 6.283f, lean = n.wdir * (1 + w.wind / 4) / 60.0f;
  for (int d = 0; d < s.nsmoke; d++)
    s.smoke_x[d] = (int16_t) (art::CHIMNEY_X + lean * d * d + 1.5f * sinf(d / 4.0f + ph));
  // The flag on flag days, or the rainbow flag on Pride day, on a pole by the cabin, flying above its roof and the
  // way the wind blows. The Norwegian blue is the top of the sky and the white white_c, colours we have anyway.
  s.pole_top = H;
  if (n.flag_up) {
    s.pole_base = (int16_t) (art::LAND[POLE_X] + 1);
    s.pole_top = (int16_t) std::min(art::EAVE_L_Y - 21, s.pole_base - 34);
    int fx = n.wdir < 0 ? POLE_X - 23 : POLE_X + 2;
    if (n.pride)
      flag_pride(s, fx, s.pole_top);
    else
      flag_norway(s, fx, s.pole_top, mix(0x601418, 0xEF2B2D, n.light), s.white_c, s.sky[0]);
  }
}

// w bits the other way round, so a sprite faces the other way
inline uint32_t mirror(uint32_t bits, int w) {
  uint32_t m = 0;
  for (int i = 0; i < w; i++)
    m |= (bits >> i & 1) << (w - 1 - i);
  return m;
}

// Now and then, while it's light: a reindeer from the rock carvings in the ochre they're painted with, peeking out of
// the trees behind the cabin or walking out of them over the hour, and a walrus swimming in the bay. Each hour
// rolls its own, apart from the rest so nothing else moves.
inline void animals(Scene &s, const Now &n) {
  if (n.light <= 0)
    return;
  Rng a{(uint32_t) (n.t / 3600) * 2654435761u + 77};
  int minute = (int) (n.local % 3600 / 60), deer = a.in(0, 99);
  uint32_t ochre = mix(0x4A2618, 0x9E4A2E, n.light);  // the walrus is that reddish brown too, one colour for both
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
  if (walrus < 35 && n.fog <= 45 && n.bft <= 6) {  // a third of the hours, swimming a pixel every ten minutes
    x += dir * minute / 10;
    uint32_t body[3], tusks[2];
    for (int r = 0; r < 3; r++)
      body[r] = dir < 0 ? WALRUS[r] : mirror(WALRUS[r], 11);
    for (int r = 0; r < 2; r++)
      tusks[r] = dir < 0 ? TUSKS[r] : mirror(TUSKS[r], 5);
    sprite_rows(sprite(s, x, y, 11, 3, 1, ochre), body);
    sprite_rows(sprite(s, dir < 0 ? x : x + 6, y + 3, 5, 2, 1, s.white_c), tusks);
  }
}

inline void holidays(Scene &s, const Now &n, Rng &rng) {
  int home = art::DOOR_X - 40;  // the hill in front of the cabin
  if (n.xmas && n.dark) {       // lights along the eaves, a star on the chimney
    uint32_t cols[3] = {q(255, 60, 60), s.window_c, q(60, 220, 100)};
    for (int k = 0; k <= 12; k++) {
      int x = art::EAVE_L_X + (art::EAVE_R_X - art::EAVE_L_X) * k / 12,
          y = art::EAVE_L_Y + (art::EAVE_R_Y - art::EAVE_L_Y) * k / 12 + 1;
      box(s, x, y, 2, 1, 2, cols[k % 3]);
    }
    sprite_rows(sprite(s, art::CHIMNEY_X - 3, art::CHIMNEY_Y - 9, 7, 7, 2, s.window_c), STAR7);
  }
  if (n.newyear && n.dark) {  // fireworks, somewhere new every redraw
    Rng fw{(uint32_t) (n.t / 300) * 40503u + 1};
    uint32_t cols[3] = {q(255, 100, 170), q(100, 220, 255), q(255, 230, 120)};
    for (int i = 0; i < 4; i++)
      burst(s, fw.in(30, W - 30), fw.in(24, 100), fw.in(9, 12), cols[(i + fw.in(0, 1)) % 3],
            s.nstars ? s.star_c : 0xFFFFFF);
  }
  if (n.easter_days && !n.white) {  // striped eggs in the heather in front of the cabin
    uint32_t cols[3] = {s.window_c, q(230, 100, 170), q(100, 190, 230)};
    Rng eg{(uint32_t) n.year * 7u};
    for (int i = 0; i < 5; i++) {
      int x = home - 30 + i * 13 + eg.in(-3, 3), y = art::LAND[x] + 8 + eg.in(0, 5);
      blob(s, x, y, 4, 5, 1, cols[i % 3]);
      box(s, x - 3, y - 2, 7, 1, 1, s.white_c);
      box(s, x - 3, y + 1, 7, 1, 1, s.white_c);
    }
  }
  if (n.midsummer) {  // the bonfire, on the hill away from the cabin
    int bx = home - 30, by = art::LAND[bx] + 8;
    box(s, bx - 9, by - 2, 19, 3, 2, s.land[art::S_BLACK]);  // the logs
    sprite_rows(sprite(s, bx - 7, by - 21, 15, 20, 2, q(255, 130, 30)), FLAME);
    sprite_rows(sprite(s, bx - 7, by - 21, 15, 20, 2, q(255, 220, 90)), FLAME_IN);
    for (int i = 0; i < 10; i++)
      box(s, bx + rng.in(-12, 12), by - 24 - rng.in(0, 26), 1, 1, 2, q(255, 220, 90));
    blob(s, bx + n.wdir * 10, by - 36, 6, 4, 1, s.land[art::S_MTN_C]);  // smoke in the mountain's blue-grey
    blob(s, bx + n.wdir * 18, by - 46, 8, 5, 1, s.land[art::S_MTN_C]);
  }
  if (n.halloween) {  // a pumpkin by the door
    int px = art::DOOR_X - 14, py = art::DOOR_Y - 5;
    blob(s, px, py, 7, 6, 1, q(255, 130, 40));
    box(s, px - 1, py - 8, 3, 3, 1, s.land[art::S_BLACK]);  // its stalk
    if (n.dark)
      sprite_rows(sprite(s, px - 5, py - 3, 11, 7, 2, q(17, 17, 34)), FACE);
  }
}

// The forecast over the picture, in white with an outline
inline void overlay(Scene &s, const Now &n, int utc_offset) {
  s.overlay = true;
  memset(s.ov, 0, OV_H * sizeof *s.ov);
  s.text_c = q(255, 255, 255);
  // outline in the darkest colour already in the picture, if it's dark enough
  uint32_t cand[] = {s.sky[0], s.land[art::S_MTN_A], s.land[art::S_HEATH_FORE], s.land[art::S_BLACK], s.cloud_shade[2]};
  auto luma = [](uint32_t c) { return (int) ((c >> 16) * 3 + (c >> 8 & 255) * 6 + (c & 255)) / 10; };
  s.outline_c = cand[0];
  for (uint32_t c : cand)
    if (luma(c) < luma(s.outline_c))
      s.outline_c = c;
  if (luma(s.outline_c) > 70)
    s.outline_c = q(17, 17, 34);
  forecast(s, n.w, n.t, utc_offset);
  if (n.w.wave >= 0) {  // and Oceanforecast's wave height, in the bay
    char b[8];
    snprintf(b, sizeof b, n.w.wave < 9.95f ? "%.1fm" : "%.0fm", n.w.wave);
    ov_text(s, SEA_OV_X + 12, SEA_OV_Y + 4, b, 2);
  }
}

// ---------------------------------------------------------------- drawing a row

inline void span(uint32_t *row, int a, int b, uint32_t c) {
  if (a < 0)
    a = 0;
  if (b > W - 1)
    b = W - 1;
  for (int x = a; x <= b; x++)
    row[x] = c;
}

// Half width of an ellipse dy rows from its centre, -1 if outside
inline int half(int dy, int rx, int ry) {
  if (dy < -ry || dy > ry || ry <= 0)
    return -1;
  float v = 1 - (float) (dy * dy) / (float) (ry * ry);
  return (int) (rx * sqrtf(v) + 0.5f);
}

inline void disc(uint32_t *row, int y, int cx, int cy, int r, uint32_t c) {
  int h = half(y - cy, r, r);
  if (h >= 0)
    for (int k = -1; k <= 1; k++)  // north is at both edges, so draw it there too
      span(row, cx - h + k * W, cx + h + k * W, c);
}

inline void sprites(const Scene &s, int y, uint32_t *out, int layer) {
  for (int i = 0; i < s.nsprites; i++) {
    const Sprite &p = s.sprite[i];
    if (p.layer != layer || y < p.y || y >= p.y + p.h)
      continue;
    uint32_t bits = p.rows[y - p.y];
    for (int c = 0; c < p.w; c++)
      if (bits >> (p.w - 1 - c) & 1) {
        int x = p.x + c;
        if (x >= 0 && x < W)
          out[x] = p.c;
      }
  }
}

// The three of them behind the big window, 7 by 13 from its top left, leftmost pixel in bit 6: someone standing,
// someone sitting, and the dog's head between them. They keep off its edges so it still reads as a window, and its
// frames stay in front of them.
static const uint8_t INDOORS[13] = {0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x36, 0x36, 0x00, 0x2A, 0x3A, 0x12, 0x00};
static constexpr int INDOORS_X = art::WINDOW_X - 4, INDOORS_Y = art::WINDOW_Y - 6;

// The artboard moon's maria, relative to its middle: x, y, half width, half height
static const int8_t MARIA[6][4] = {{-3, -3, 2, 2}, {1, -4, 2, 1}, {3, -1, 2, 2},
                                   {-5, 1, 2, 3},  {-1, 3, 2, 1}, {4, 2, 1, 1}};

// The bay at x, y, base its colour without the waves. near is the nearer water, darker, where the waves' faces show
// lighter than it; further off they show as a crest over a dark trough. Nearer waves are drawn over further ones.
inline uint32_t sea_px(const Scene &s, int x, int y, uint32_t base, bool near) {
  const SeaLook &l = s.look;
  uint32_t c = base;
  if (l.streak && y % 3 == 0) {  // foam blown into long streaks along the wind
    int u = x + (int) hash8((uint32_t) y, 7, s.sea_seed) % 16;
    uint32_t h = hash8((uint32_t) (u / 16), (uint32_t) y, s.sea_seed);
    if (h < l.streak && u % 16 < 6 + (int) (h % 8))
      c = s.foam_c;
  }
  if (!l.crest)
    return c;
  float peak = 0.5f + 0.15f * s.wind_dir;  // the side the wind blows to is steeper
  for (int j = 0; j < s.nsea; j++) {
    int b0 = s.sea_y[j], a = s.sea_a[j];
    if (y > b0 + 2 || y < b0 - a - l.spray - 1)
      continue;
    int len = s.sea_len[j], pos = x + s.sea_ph[j], m = pos / len;
    uint32_t h = hash8((uint32_t) j, (uint32_t) m, s.sea_seed);
    if (h >= l.crest)
      continue;               // no crest on this arch
    b0 += (int) (h % 3) - 1;  // not quite in line with the next one
    float u = (pos % len + 0.5f) / len, b = u < peak ? u / peak : (1 - u) / (1 - peak);
    b = b * (0.4f + 0.6f * b);  // pointed at the top
    if (b < (0.12f + (h >> 2) % 4 * 0.08f) * (1 - 0.7f * s.swell))
      continue;  // and only so long, swell longer
    bool cap = hash8((uint32_t) j, (uint32_t) m, s.sea_seed ^ 0xA5A5u) < l.cap && b * 16 >= l.cap_from;
    int top = b0 - (int) lroundf(a * b);
    if (y == top) {
      c = cap ? s.foam_c : s.crest_c;
    } else if (y > top && y <= b0) {  // its face, white down from the top the harder it blows
      int down = y - top;
      bool foam = cap && a >= 2 && down <= 1 + l.foam / 64 && b * 16 >= l.cap_from + 2 * down;
      c = foam ? s.foam_c : near ? s.land[art::S_SEA_FAR] : base;
    } else if (y == b0 + 1) {
      if (!near && a > 0)
        c = s.land[art::S_SEA_NEAR];
    } else if (y < top && cap && hash8((uint32_t) x, (uint32_t) y, s.sea_seed + 1) < 90u - (top - y) * 15u) {
      c = s.foam_c;  // spray off the crest
    }
  }
  return c;
}

// Draws row y of the scene into out, W pixels
inline void row(const Scene &s, int y, uint32_t *out) {
  span(out, 0, W - 1, s.sky[band_of(y)]);
  if (y <= HORIZON) {
    if (s.glow)
      for (int step = 0; step < 2; step++) {  // a wide faint one, a smaller strong one
        int rx = step ? s.glow_rx * 3 / 5 : s.glow_rx * 3 / 2, ry = step ? s.glow_ry * 3 / 5 : s.glow_ry;
        int h = half(y - HORIZON, rx, ry);
        if (h >= 0)
          for (int k = -1; k <= 1; k++)
            span(out, s.glow_x - h + k * W, s.glow_x + h + k * W, step ? s.glow_c : s.glow_outer_c);
      }
    if (s.aurora && y >= art::AURORA_Y0 && y < art::AURORA_Y1)  // the curtain's faint top, body, bright edge
      for (int i = art::AURORA_ROW[y - art::AURORA_Y0], x = 0; i < art::AURORA_ROW[y - art::AURORA_Y0 + 1]; i += 2) {
        int v = art::AURORA[i], len = art::AURORA[i + 1];
        if (v)
          span(out, x, x + len - 1, s.aur_c[v - 1]);
        x += len;
      }
    for (int i = 0; i < s.nstars; i++)
      if (s.star_y[i] == y)
        out[s.star_x[i]] = s.dim_c;
    for (int i = 0; i < s.nbright; i++) {  // catalogue stars a dot, the brightest a small cross
      int dy = y - s.bright_y[i], x = s.bright_x[i];
      if (dy == 0)
        span(out, x - (s.bright_big[i] ? 1 : 0), x + (s.bright_big[i] ? 1 : 0), s.star_c);
      else if (s.bright_big[i] && (dy == -1 || dy == 1) && x >= 0 && x < W)
        out[x] = s.star_c;
    }
    if (s.moon) {
      if (s.moon_glow) {
        if (!s.aurora)  // its colour goes to the aurora then
          disc(out, y, s.moon_x, s.moon_y, MOON_R * 22 / 10, s.moon_halo_c);
        disc(out, y, s.moon_x, s.moon_y, MOON_R * 3 / 2, s.moon_glow_c);
      }
      int dy = y - s.moon_y, h = half(dy, MOON_R, MOON_R);
      if (h >= 0) {  // lit from the right while waxing, from the left after full
        float lim = cosf(s.moon_phase * 2 * (float) kPi) * MOON_R * sqrtf(1 - dy * dy / (float) (MOON_R * MOON_R));
        for (int dx = -h; dx <= h; dx++) {
          bool lit = s.moon_phase < 0.5f ? dx > lim : dx < -lim;
          int x = s.moon_x + dx;
          if (!lit || x < 0 || x >= W)
            continue;
          bool mare = false;
          for (const int8_t *m : MARIA)
            mare = mare || (half(dy - m[1], m[2], m[3]) >= abs(dx - m[0]));
          out[x] = mare ? s.crater_c : s.moon_c;
        }
      }
    }
    if (s.sun) {
      if (s.sun_rings) {
        disc(out, y, s.sun_x, s.sun_y, SUN_R * 23 / 10, s.ring2_c);
        disc(out, y, s.sun_x, s.sun_y, SUN_R * 16 / 10, s.ring1_c);
      }
      disc(out, y, s.sun_x, s.sun_y, SUN_R, s.sun_c);
    }
  }
  for (int i = 0; i < s.nclouds; i++) {
    const Cloud &c = s.cloud[i];
    if (y < c.top || y > c.base)
      continue;
    int l = s.cloud_layer[i];
    uint32_t col = y > c.shade ? s.cloud_shade[l] : s.cloud_lit[l];
    int x0 = (c.x + s.drift[l]) % FIELD - 100;
    for (int j = 0; j < c.n; j++) {
      int h = half(y - c.e[j].y, c.e[j].rx, c.e[j].ry);
      if (h < 0)
        continue;
      int cx = x0 + c.e[j].x;
      span(out, cx - h, cx + h, col);
      span(out, cx - h - FIELD, cx + h - FIELD, col);
    }
  }
  if (s.rainbow && y < HORIZON)
    for (int k = 0; k < 3; k++) {  // three bands, each 4 rows thick, red outermost
      int outer = half(y - s.rb_y, RB_RX - k * 4, RB_RY - k * 4),
          inner = half(y - s.rb_y, RB_RX - k * 4 - 4, RB_RY - k * 4 - 4);
      if (outer < 0)
        continue;
      if (inner < 0)
        inner = -1;
      span(out, s.rb_x - outer, s.rb_x - inner - 1, s.rb_c[k]);
      span(out, s.rb_x + inner + 1, s.rb_x + outer, s.rb_c[k]);
    }
  for (int i = 0; i < s.nstreaks; i++) {  // the wind's streaks, a gentle wave along each
    int len = s.streak_len[i];
    for (int k = 0; k < len; k++) {
      int along = s.wind_dir > 0 ? k : len - 1 - k;  // 0 where the wind comes from
      if (along < len * 3 / 10 && (along / 2) % 2)
        continue;                                           // dashed at the start
      int curl = along >= len - 3 ? along - (len - 4) : 0;  // and a little hook at the end
      int x = s.streak_x[i] + k,
          ly = s.streak_y[i] + (int) lroundf(1.5f * sinf(k / 5.0f + s.streak_ph[i] / 40.0f)) - curl;
      if (ly == y && x >= 0 && x < W)
        out[x] = s.streak_c;
    }
  }
  if (s.bft >= 11 && y < HORIZON && y >= HORIZON - 10 * (s.bft - 10))  // the air over the bay full of spray
    for (int x = 0; x <= s.hz_x1 + 6 && x < W; x++)
      if (hash8((uint32_t) x, (uint32_t) y, s.sea_seed + 4) < 50u - (HORIZON - y) * 2u)
        out[x] = s.foam_c;
  sprites(s, y, out, 0);
  if (y >= art::GROUND_Y0) {  // the landscape, the bay with its waves and the light on it, and the fog
    using namespace art;
    int dy = y - HORIZON, spread = 1 + dy / 9;
    uint32_t glints = 256;  // calm it's a column of light, rougher it breaks into glints
    bool shine = s.shine_x >= 0;
    if (s.bft == 0) {
      spread = dy / 12;
    } else if (s.bft <= 5) {
      shine = shine && (y & 1);
      if (s.bft >= 3) {
        spread = 1 + dy / 6;
        glints = 170;
      }
    } else {
      spread = 2 + dy / 4;
      glints = 70;
    }
    float g = 0.4f + 0.6f * dy / (H - HORIZON);  // nearer, more surf
    int surf = (int) lroundf(s.surf * g), surf_rock = (int) lroundf(s.surf_rock * g);
    auto sea = [](int k) { return k == S_SEA_FAR || k == S_SEA_NEAR; };
    auto rock = [](int k) { return k == S_ROCK_LIGHT || k == S_ROCK_DARK; };
    int r0 = GROUND_ROW[y - GROUND_Y0], r1 = GROUND_ROW[y - GROUND_Y0 + 1];
    for (int i = r0, x = 0; i < r1; i += 2) {
      int k = GROUND[i], x0 = x, end = x + GROUND[i + 1], d = DEPTH[k];
      int prev = i > r0 ? GROUND[i - 2] : -1, next = i + 2 < r1 ? GROUND[i + 2] : -1;
      for (; x < end; x++) {
        uint32_t c = k ? s.land[k] : out[x];
        if (k == S_BIRCH && s.birch_keep < 255 && hash8((uint32_t) x, (uint32_t) y, 5) >= s.birch_keep) {
          // through the bare twigs, what's behind
          int u = BIRCH_UNDER[(y - BIRCH_Y) * BIRCH_W + x - BIRCH_X];
          c = u ? s.land[u] : out[x];
        }
        if (sea(k)) {
          c = sea_px(s, x, y, c, k == S_SEA_NEAR);
          if (shine && c != s.foam_c && abs(x - s.shine_x) <= spread &&
              (glints > 255 || hash8((uint32_t) x, (uint32_t) y, s.sea_seed + 5) < glints))
            c = s.shine_c;
          int shore = std::min(rock(next) ? end - 1 - x : 99, rock(prev) ? x - x0 : 99);
          if (shore < surf && hash8((uint32_t) shore, (uint32_t) y, s.sea_seed + 2) < 190)
            c = s.foam_c;  // surf on the rocks
        } else if (rock(k) && surf_rock) {
          int shore = std::min(sea(prev) ? x - x0 : 99, sea(next) ? end - 1 - x : 99);
          if (shore < surf_rock && (int) hash8((uint32_t) x, (uint32_t) y, s.sea_seed + 3) < 110 - shore * 25)
            c = s.foam_c;  // and spray up them
        } else if (k == S_WINDOW && s.indoors && x >= INDOORS_X && x < INDOORS_X + 7 && y >= INDOORS_Y &&
                   y < INDOORS_Y + 13 && (INDOORS[y - INDOORS_Y] >> (6 - (x - INDOORS_X)) & 1)) {
          c = s.land[S_BLACK];
        } else if (!k && y > HORIZON - s.bumps && x >= s.hz_x0 && x <= s.hz_x1 &&
                   HORIZON - y < (int) (hash8((uint32_t) x / 3, 9, s.sea_seed) % (uint32_t) (s.bumps + 1))) {
          c = s.land[S_SEA_FAR];  // the biggest waves break the horizon
        }
        if ((d <= 0 && y >= s.fog_top[0][x]) || (d <= 1 && y >= s.fog_top[1][x]))
          c = s.fog_c;
        out[x] = c;
      }
    }
  }
  int d = art::CHIMNEY_Y - 1 - y;  // the smoke, thinning out at the top
  if (d >= 0 && d < s.nsmoke && (d < s.nsmoke * 2 / 3 || (d & 1)))
    span(out, s.smoke_x[d], s.smoke_x[d] + s.smoke_w - 1, s.smoke_c);
  if (y >= s.pole_top && y <= s.pole_base)  // the flagpole
    out[POLE_X] = s.land[art::S_BLACK];
  for (int i = 0; i < s.nfig; i++) {  // the people, with the headlamp's glow, in the fog where it's thick enough
    const art::Figure &f = *s.fig[i];
    auto fogged = [&](int x) {
      return (f.depth <= 0 && y >= s.fog_top[0][x]) || (f.depth <= 1 && y >= s.fog_top[1][x]);
    };
    if (s.lamp && f.lamp_x >= 0) {
      int h = half(y - f.lamp_y, 3, 3);
      for (int x = std::max(0, f.lamp_x - h); h >= 0 && x <= std::min(W - 1, f.lamp_x + h); x++)
        if (!fogged(x))
          out[x] = s.lamp_glow_c;
    }
    if (y < f.y || y >= f.y + f.h)
      continue;
    const uint8_t *px = f.px + (y - f.y) * ((f.w + 3) / 4);
    for (int c = 0; c < f.w; c++) {
      int v = px[c / 4] >> (c % 4 * 2) & 3, x = f.x + c;
      if (v && x >= 0 && x < W && !fogged(x))  // the dog in the artboard's black, as it was drawn
        out[x] = v == 2 && s.lamp ? s.lamp_c : &f == &art::DOG ? s.land[art::S_BLACK] : s.fig_c;
    }
  }
  sprites(s, y, out, 1);
  for (int i = 0; i < s.nprec; i++) {
    int dy = y - s.prec_y[i];
    if (dy < 0 || y < s.prec_top[s.prec_x[i]])
      continue;
    if (s.prec_kind[i] == 0) {  // a streak, 7 rows
      if (dy >= 0 && dy < 7) {
        int x = s.prec_x[i] + s.slant * dy / 8;
        if (x >= 0 && x < W)
          out[x] = s.rain_c;
      }
    } else if (dy >= 0 && dy < s.prec_kind[i]) {
      span(out, s.prec_x[i], s.prec_x[i] + s.prec_kind[i] - 1, s.snow_c);
    }
  }
  if (s.bolt)
    for (int i = 0; i < 6; i++) {
      int y0 = s.bolt_y[i], y1 = s.bolt_y[i + 1];
      if (y >= y0 && y < y1) {
        int x = s.bolt_x[i] + (s.bolt_x[i + 1] - s.bolt_x[i]) * (y - y0) / (y1 - y0);
        span(out, x, x + 1, s.bolt_c);
      }
    }
  sprites(s, y, out, 2);
  if (s.overlay && (y < OV_H || (y >= SEA_OV_Y && y < SEA_OV_Y + SEA_OV_H)))
    for (int x = 0; x < W; x++) {
      int v = ov_get(s, x, y);
      if (v)
        out[x] = v == 2 ? s.text_c : s.outline_c;
    }
}

// ---------------------------------------------------------------- 16 colours

// Every colour the scene draws with
template<typename F> void each_colour(Scene &s, F f) {
  for (uint32_t *c : {&s.glow_c,      &s.glow_outer_c, &s.sun_c,       &s.ring1_c, &s.ring2_c, &s.moon_c,
                      &s.crater_c,    &s.moon_glow_c,  &s.moon_halo_c, &s.star_c,  &s.dim_c,   &s.rain_c,
                      &s.snow_c,      &s.bolt_c,       &s.fog_c,       &s.shine_c, &s.fig_c,   &s.lamp_c,
                      &s.lamp_glow_c, &s.window_c,     &s.smoke_c,     &s.white_c, &s.text_c,  &s.outline_c})
    f(*c);
  f(s.crest_c);
  f(s.foam_c);
  f(s.streak_c);
  for (uint32_t &c : s.sky)
    f(c);
  for (uint32_t &c : s.aur_c)
    f(c);
  for (int l = 0; l < 3; l++) {
    f(s.cloud_lit[l]);
    f(s.cloud_shade[l]);
    f(s.rb_c[l]);
  }
  for (uint32_t &c : s.land)
    f(c);
  for (int i = 0; i < s.nsprites; i++)
    f(s.sprite[i].c);
}

// The things that must keep their own colour: flags and the holidays' things, the lit windows, the headlamp, the
// sun, the moon, the aurora's bright edge, the forecast's text, the bolt, the rainbow, and by day the mountain's
// ridge, the bay for its waves and the cabin's wall, so its door and frames show
inline bool accent(const Scene &s, uint32_t c) {
  for (int i = 0; i < s.nsprites; i++)
    if (s.sprite[i].c == c)
      return true;
  return (s.window_lit && c == s.window_c) || (s.lamp && s.nfig && c == s.lamp_c) || (s.sun && c == s.sun_c) ||
         (s.moon && c == s.moon_c) || (s.aurora && c == s.aur_c[2]) || (s.overlay && c == s.text_c) ||
         (s.bolt && c == s.bolt_c) || (s.rainbow && (c == s.rb_c[0] || c == s.rb_c[1] || c == s.rb_c[2])) ||
         (s.keep_ridge && c == s.land[art::S_MTN_A]) ||
         (s.keep_sea &&
          (c == s.land[art::S_SEA_FAR] || c == s.land[art::S_SEA_NEAR] || c == s.land[art::S_CABIN_WALL]));
}

// The panel takes 16 colours. The scene can have more, so while it does, the colour that costs least to lose
// becomes another: how far apart they look times the square root of how much there is of it, the landscape
// counting four times the sky (it's the picture; the sky is bands of a gradient). Accents are never lost, but
// others may become them. line is a row of scratch space.
inline void fit_16_colours(Scene &s, uint32_t *line) {
  static constexpr int MAX = 48;
  uint32_t col[MAX], cnt[MAX];
  int n = 0;
  for (int y = 0; y < H; y++) {
    row(s, y, line);
    int k = -1;
    for (int x = 0; x < W; x++) {
      if (k < 0 || col[k] != line[x]) {
        for (k = 0; k < n && col[k] != line[x]; k++) {
        }
        if (k == n) {
          if (n == MAX) {
            k = -1;
            continue;
          }
          col[n] = line[x];
          cnt[n++] = 0;
        }
      }
      cnt[k] += y >= art::RIDGE[x] ? 4 : 1;
    }
  }
  // How far apart two colours look: OKLab, with hue counting double so grey rock doesn't become green heath
  float lab[MAX][3];
  bool keep[MAX];
  for (int k = 0; k < n; k++) {
    float c[3];
    for (int i = 0; i < 3; i++) {
      float v = (col[k] >> (16 - 8 * i) & 255) / 255.0f;
      c[i] = v <= 0.04045f ? v / 12.92f : powf((v + 0.055f) / 1.055f, 2.4f);
    }
    float l = cbrtf(0.4122214708f * c[0] + 0.5363325363f * c[1] + 0.0514459929f * c[2]),
          m = cbrtf(0.2119034982f * c[0] + 0.6806995451f * c[1] + 0.1073969566f * c[2]),
          s3 = cbrtf(0.0883024619f * c[0] + 0.2817188376f * c[1] + 0.6299787005f * c[2]);
    lab[k][0] = 0.2104542553f * l + 0.7936177850f * m - 0.0040720468f * s3;
    lab[k][1] = 1.9779984951f * l - 2.4285922050f * m + 0.4505937099f * s3;
    lab[k][2] = 0.0259040371f * l + 0.7827717662f * m - 0.8086757660f * s3;
    keep[k] = accent(s, col[k]);
  }
  auto dist = [&lab](int a, int b) {
    float dl = lab[a][0] - lab[b][0], da = lab[a][1] - lab[b][1], db = lab[a][2] - lab[b][2];
    return dl * dl + 2 * (da * da + db * db);
  };
  while (n > 16) {
    int from = -1, to = -1;
    float best = 1e30f;
    for (int pass = 0; pass < 2 && from < 0; pass++)  // accents only if there's nothing else left to merge
      for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
          if (i == j || (keep[i] && !pass))
            continue;
          float cost = dist(i, j) * sqrtf((float) cnt[i]);
          if (cost < best) {
            best = cost;
            from = i;
            to = j;
          }
        }
    uint32_t a = col[from], b = col[to];
    each_colour(s, [a, b](uint32_t &c) {
      if (c == a)
        c = b;
    });
    cnt[to] += cnt[from];
    keep[to] = keep[to] || keep[from];
    n--;
    col[from] = col[n];
    cnt[from] = cnt[n];
    keep[from] = keep[n];
    for (int i = 0; i < 3; i++)
      lab[from][i] = lab[n][i];
  }
}

// How many bytes the picture packs into for the panel, as picture.h packs it: rows of 4-bit pixels, a run of three
// or more equal bytes as two bytes and the rest as they are. The panel holds PACK_MAX and a little more; a picture
// that doesn't fit has to go as a JPEG, which takes the ESP far longer.
static constexpr size_t PACK_MAX = 13824;
inline size_t packed_size(const Scene &s, uint32_t *line) {
  uint32_t pal[16];
  int np = 0;
  size_t total = 0;
  uint8_t b[W / 2];
  for (int y = 0; y < H; y++) {
    row(s, y, line);
    for (int x = 0; x < W; x++) {
      int i = 0;
      while (i < np && pal[i] != line[x])
        i++;
      if (i == np && np < 16)
        pal[np++] = line[x];
      i = std::min(i, 15);
      b[x / 2] = (uint8_t) (x & 1 ? b[x / 2] | i : i << 4);
    }
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

// The whole scene at t, UTC. Without w.ok there's no weather in it. utc_offset is local time minus UTC in
// seconds.
inline void build(Scene &s, const Weather &w, double lat, double lon, uint32_t t, bool with_forecast, int utc_offset,
                  const Options &o) {
  Sprite *sprite = s.sprite;
  uint8_t (*ov)[W / 4] = s.ov;
  memset(&s, 0, sizeof s);
  s.sprite = sprite;
  s.ov = ov;
  Now n(w, o, lat, lon, t, utc_offset);
  s.overlay = with_forecast && w.ok;  // drawn last, but the clouds make room for it
  sky(s, n);
  sun_moon(s, n);
  aurora(s, n);
  stars(s, n);
  clouds(s, n);
  precipitation(s, n);
  land(s, n);
  people(s, n);
  Rng rng{(uint32_t) (n.local / 3600) * 1664525u + 1013904223u};  // for life() and holidays(), new each hour
  life(s, n, rng);
  animals(s, n);
  holidays(s, n, rng);
  if (with_forecast && w.ok)
    overlay(s, n, utc_offset);
  fit_16_colours(s, s.scratch);
  // A storm, heavy rain and the forecast can be too much to send as 16 colours. Then the rain thins out and the bay
  // calms down: first the foam streaks and spray go, then half the waves, then all of them and the surf.
  int nprec = s.nprec;
  for (int step = 0; step < 4 && (s.overlay || s.nprec) && packed_size(s, s.scratch) > PACK_MAX; step++) {
    s.nprec = nprec * (3 - std::min(step, 2)) / 4;
    if (step >= 1)
      s.look.streak = s.look.spray = 0;
    if (step == 2)
      s.look.crest /= 2;
    if (step == 3) {
      s.look.crest = 0;
      s.surf = s.surf_rock = 0;
      s.bumps = 0;
    }
  }
}

}  // namespace flexit::panel::met