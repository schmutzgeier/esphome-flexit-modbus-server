// Draws the weather picture of packages/code/met for mocked weathers, wave heights, places, landscapes and days,
// as met.h makes it: build(), what the animations leave out, palette(), the heath's fit, then the show that fits
// it, run on animation_mock.h's model of the panel. Writes each case's picture and the frames of its animation as
// PPMs to OUT/<case>/, which render.py turns into PNGs, GIFs and a contact sheet. See run.sh.
// --check runs each heath case for check_minutes without frames and checks that the kayak keeps to the water and
// the dog's feet to the ground.
//   met_sim OUT [--only text] [--seconds 40] [--fps 10] [--no-anim] [--real-timers] [--check] [--lat 60.39
//   --lon 5.32] [-v]
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "met_art.h"
#include "met_heath.h"
#include "met_live_shows.h"
#include "met_raster.h"

using namespace flexit::panel::met;

// Norway's UTC offset: CEST from 01:00 UTC on the last Sunday in March to the same in October
static int norway(uint32_t t) {
  int y, m, d;
  civil_from_days((int32_t) (t / 86400), &y, &m, &d);
  auto last_sunday = [y](int month) {  // 1970-01-01 was a Thursday
    uint32_t day = unix_of(y, month, 31) / 86400;
    return (day - (day + 4) % 7) * 86400u + 3600;
  };
  return t >= last_sunday(3) && t < last_sunday(10) ? 7200 : 3600;
}

struct Case {
  const char *name;
  Ground ground;
  int y, mo, d, h, mi;  // local time
  const char *symbol;
  float temp, cloud, rain, wind, wind_from, wave;  // wave in m, -1 = none (inland)
  float fog = 0, thunder = 0;
  bool forecast = false;
  int heater = 30;  // %
  bool fireplace = false;
  Aurora aurora = A_AUTO;
};

// clang-format off: a case a line
static const Case CASES[] = {
    // the heath through the weather and the bay by the Beaufort scale
    {"heath_calm_morning",  G_HEATH, 2026, 6, 14,  7, 30, "clearsky_day",         14,  5, 0,    0.2f, 180, 0.1f},
    {"heath_fair_forecast", G_HEATH, 2026, 7, 10, 13,  0, "fair_day",             18, 30, 0,    4,    200, 0.6f, 0, 0, true},
    {"heath_breeze_waves",  G_HEATH, 2026, 8, 20, 15,  0, "partlycloudy_day",     16, 50, 0,    9,    250, 1.8f, 0, 0, true},
    {"heath_gale_rain",     G_HEATH, 2026, 10, 3, 12,  0, "rain",                  9, 95, 2.5f, 19,   230, 4.5f, 0, 0, true},
    {"heath_storm",         G_HEATH, 2026, 11, 8, 11,  0, "heavyrain",             7, 100, 6,  31,   260, 9.5f, 0, 0, true},
    {"heath_inland_wind",   G_HEATH, 2026, 5,  9, 14,  0, "cloudy",               11, 85, 0,    8,     90, -1},
    {"heath_thunder",       G_HEATH, 2026, 7, 28, 17,  0, "heavyrainandthunder",  21, 100, 4,  10,   200, 1.2f, 0, 80},
    {"heath_heavy_rain_in", G_HEATH, 2026, 9, 14, 15,  0, "heavyrain",            12, 100, 6,   6,   200, 0.8f},
    {"heath_fog",           G_HEATH, 2026, 10, 22, 9,  0, "fog",                   6, 100, 0,   1,   200, 0.3f, 90},
    {"heath_snow",          G_HEATH, 2026, 2,  3, 11,  0, "snow",                 -4, 90, 1,    3,    30, 0.5f},
    {"heath_winter_sun",    G_HEATH, 2026, 1, 20, 12,  0, "clearsky_day",         -8,  0, 0,    2,    60, 0.3f},
    {"heath_dusk_autumn",   G_HEATH, 2026, 9, 25, 19, 30, "partlycloudy_night",    9, 40, 0,    6,   300, 1.2f, 20},
    {"heath_night_aurora",  G_HEATH, 2026, 1, 12, 22,  0, "clearsky_night",       -5,  5, 0,    2,    90, 0.4f, 0, 0, false, 30, false, A_ALWAYS},
    {"heath_night_moon",    G_HEATH, 2026, 3,  3, 21,  0, "clearsky_night",        1, 10, 0,    3,   200, 0.6f, 0, 0, false, 50, false, A_OFF},
    {"heath_cosy_fire",     G_HEATH, 2026, 11, 20, 20, 0, "cloudy",                2, 80, 0,    5,   200, 1.0f, 0, 0, false, 90, true},
    {"heath_lights_out",    G_HEATH, 2026, 4, 18,  0, 30, "partlycloudy_night",    3, 40, 0,    3,   200, 0.5f},
    {"heath_autumn_leaves", G_HEATH, 2026, 10, 10, 11, 0, "partlycloudy_day",      9, 40, 0,    7,   270, 1.1f},
    {"heath_summer_butterflies", G_HEATH, 2026, 7, 2, 12, 0, "clearsky_day",     22,  5, 0,    2,   180, 0.2f},
    // the days
    {"heath_xmas_eve",      G_HEATH, 2026, 12, 24, 19, 0, "clearsky_night",       -3, 10, 0,    1,     0, 0.4f},
    {"heath_new_year",      G_HEATH, 2026, 12, 31, 23, 50, "clearsky_night",      -6,  5, 0,    2,     0, 0.4f},
    {"heath_easter",        G_HEATH, 2026, 4,  4, 12,  0, "fair_day",              8, 30, 0,    4,   200, 0.5f},
    {"heath_17_may",        G_HEATH, 2026, 5, 17, 12,  0, "fair_day",             15, 25, 0,    3,   200, 0.4f},
    {"heath_pride",         G_HEATH, 2026, 6, 28, 14,  0, "fair_day",             19, 20, 0,    4,   200, 0.5f},
    {"heath_sankthans",     G_HEATH, 2026, 6, 23, 22,  0, "fair_night",           16, 20, 0,    2,   200, 0.3f},
    {"heath_halloween",     G_HEATH, 2026, 10, 31, 20, 0, "partlycloudy_night",    5, 40, 0,    5,   200, 0.9f},
    {"heath_valentine",     G_HEATH, 2026, 2, 14, 13,  0, "cloudy",                1, 80, 0,    3,   200, 0.5f, 0, 0, false, 0},
    // panel's own landscapes, to compare
    {"hills_day",           G_HILLS, 2026, 7, 10, 13,  0, "fair_day",             18, 30, 0,    4,   200, -1, 0, 0, true},
    {"hills_dusk_moose",    G_HILLS, 2026, 9, 25, 19, 10, "partlycloudy_night",    9, 30, 0,    3,   300, -1},
    {"coast_sail",          G_COAST, 2026, 7, 15, 14,  0, "fair_day",             19, 20, 0,    5,   200, -1},
    {"coast_night",         G_COAST, 2026, 10, 5, 23,  0, "partlycloudy_night",    8, 30, 0,    6,   200, -1},
    {"mountains_reindeer",  G_MOUNTAINS, 2026, 8, 5, 12, 0, "partlycloudy_day",   12, 40, 0,    4,   200, -1},
    {"city_evening",        G_CITY,  2026, 11, 12, 18, 0, "cloudy",                4, 80, 0,    4,   200, -1},
};
// clang-format on

static Weather weather_of(const Case &c, uint32_t t) {
  Weather w;
  w.ok = true;
  w.time = t - t % 3600;
  snprintf(w.symbol, sizeof w.symbol, "%s", c.symbol);
  w.temp = c.temp;
  w.cloud = c.cloud;
  w.cloud_low = c.cloud * 0.6f;
  w.cloud_mid = c.cloud * 0.4f;
  w.cloud_high = c.cloud * 0.3f;
  w.rain = c.rain;
  w.wind = c.wind;
  w.wind_from = c.wind_from;
  w.fog = c.fog;
  w.thunder = c.thunder;
  w.wave = c.wave;
  for (int i = 0; i < HOURS; i++)  // the hours after drift a little
    w.hour[w.hours++] = {w.time + 3600u * i, c.temp + 0.4f * i - 0.03f * i * i, icon_of(c.symbol),
                         c.wind * (1 + 0.05f * (i % 4)), (int16_t) (((int) c.wind_from + 10 * i) % 360)};
  return w;
}

struct Args {
  std::string out, only;
  int seconds = 40, fps = 10;
  bool anim = true, real_timers = false, check = false;
  int check_minutes = 20;
  double lat = 60.39, lon = 5.32;  // Bergen
};

static void write_ppm(const std::string &path, const uint32_t *rgb) {
  FILE *f = fopen(path.c_str(), "wb");
  if (!f) {
    perror(path.c_str());
    exit(1);
  }
  fprintf(f, "P6\n%d %d\n255\n", W, H);
  for (int i = 0; i < W * H; i++) {
    uint8_t p[3] = {(uint8_t) (rgb[i] >> 16), (uint8_t) (rgb[i] >> 8), (uint8_t) rgb[i]};
    fwrite(p, 1, 3, f);
  }
  fclose(f);
}

static Sprite sprites[SPRITES];
static uint8_t ov[OV_H][W / 4];
static Land land;
static Scene scene;

// The scene for c at sim::now_ms, its 16 colours and what the animations leave out, as met.h's draw() makes it
static void draw(const Case &c, const Args &a, Options &o, const uint32_t *prev) {
  uint32_t t = sim::utc0 + sim::now_ms / 1000;
  Weather w = weather_of(c, t);
  o.frame++;
  scene.sprite = sprites;
  scene.ov = c.forecast ? ov : nullptr;
  scene.land = &land;
  build(scene, w, a.lat, a.lon, t, c.forecast, norway, o);
  scene.live = live::leaves_out(scene);
  palette(scene, prev);
  if (scene.ground == G_HEATH)
    heath_fit(scene, prev);
}

static void picture(uint32_t *rgb) {
  uint8_t r[W];
  for (int y = 0; y < H; y++) {
    row(scene, y, r);
    for (int x = 0; x < W; x++)
      rgb[y * W + x] = scene.pal[r[x]];
  }
}

// How many colours the picture has and its size packed for the panel, as picture.h packs it
static void stats(int *colours, size_t *packed) {
  uint8_t r[W], b[W / 2];
  bool seen[16] = {};
  *packed = 0;
  for (int y = 0; y < H; y++) {
    row(scene, y, r);
    for (int x = 0; x < W; x++)
      seen[r[x]] = true;
    for (int x = 0; x < W / 2; x++)
      b[x] = (uint8_t) (r[2 * x] << 4 | r[2 * x + 1]);
    for (int i = 0; i < W / 2;) {
      int run = 1;
      while (i + run < W / 2 && run < 129 && b[i + run] == b[i])
        run++;
      if (run >= 3) {
        *packed += 2;
        i += run;
        continue;
      }
      int len = 0;
      while (i < W / 2 && len < 128 && !(i + 2 < W / 2 && b[i] == b[i + 1] && b[i] == b[i + 2])) {
        i++;
        len++;
      }
      *packed += 1 + (size_t) len;
    }
  }
  *colours = 0;
  for (bool s : seen)
    *colours += s;
}

// The copy of the show's animation n, null if there's none
static flexit::panel::anim::Copy *copy_of(const char *n, flexit::panel::anim::Anim **an = nullptr) {
  for (auto &p : flexit::panel::anim::anims)
    if (p->name == std::string("w_") + n)
      for (auto &c : p->copies)
        if (c.used && !c.fading) {
          if (an)
            *an = p.get();
          return &c;
        }
  return nullptr;
}

static bool sea_at(int x, int y) {
  using namespace art;
  if (x < 0)
    return true;  // the bay goes on past the screen's edge
  if (y < GROUND_Y0 || y >= H || x >= W)
    return false;
  int at = 0;  // the run's first column
  for (int i = GROUND_ROW[y - GROUND_Y0]; i < GROUND_ROW[y - GROUND_Y0 + 1]; i += 2) {
    if (x < at + GROUND[i + 1])
      return GROUND[i] == S_SEA_FAR || GROUND[i] == S_SEA_NEAR;
    at += GROUND[i + 1];
  }
  return false;
}

// Runs the heath's show for a while and checks where its characters are. False if one's wrong.
static bool check(const Case &c) {
  using namespace flexit::panel;
  if (live::heath_bay(scene) == live::HK_NONE && scene.heath.nfig < 3)
    return true;
  live::HeathSprite dog = live::heath_sprite(scene, live::HK_DOG_SIT);
  // the highest ground under its legs, columns 3 to 9 of it, without single tufts of heather as heath_line()
  live::Line ground_line = live::heath_line(scene, 2, 0);
  auto legs_ground = [&](int x) {
    int g = H;
    for (int dx = 3; dx <= 9; dx++)
      g = std::min(g, ground_line.at(x + dx));
    return g;
  };
  int dog_d = dog.y + dog.feet - legs_ground(dog.x);
  int steps = 0, wet = 0, kayak_bad = 0, dog_worst = 0;
  bool kayak = live::heath_bay(scene) == live::HK_KAYAK;
  for (uint32_t end = sim::now_ms + 20 * 60000u; sim::now_ms < end; sim::now_ms += live::STEP) {
    live::loop(true);
    anim::step(live::STEP);
    steps++;
    if (anim::Copy *k = kayak ? copy_of("hbay") : nullptr) {
      int x = (int) lroundf(k->base[anim::X]), y = (int) lroundf(k->base[anim::Y]);
      if (k->wave[anim::Y].on)
        y += (int) lroundf(anim::wave_value(k->wave[anim::Y]));
      bool ok = true;
      for (int r = 12; r <= 14; r++)  // the hull and the stern, at the water
        for (int dx = 0; dx < 30; dx++)
          ok = ok && (art::KAYAKER.px[r * 8 + dx / 4] >> (dx % 4 * 2) & 3 ? sea_at(x + dx, y + r) : true);
      kayak_bad += !ok;
      wet++;
    }
    for (const char *n : live::DOG_NAMES)
      if (anim::Copy *d = copy_of(n)) {
        int x = (int) lroundf(d->base[anim::X]), y = (int) lroundf(d->base[anim::Y]);
        int ground = legs_ground(x) + dog_d;
        if (abs(y + dog.feet - ground) > dog_worst && sim::verbose)
          printf("dog %s at %.1f,%.1f: feet %d, ground %d\n", n, d->base[anim::X], d->base[anim::Y], y + dog.feet,
                 ground);
        dog_worst = std::max(dog_worst, abs(y + dog.feet - ground));
      }
  }
  bool ok = !kayak_bad && dog_worst <= 1;
  printf("%-26s check: %d steps, kayak out of the water in %d of %d, the dog's feet at most %d px off the ground "
         "%s\n",
         "", steps, kayak_bad, wet, dog_worst, ok ? "ok" : "FAILED");
  return ok;
}

static bool failed;

static void run(const Case &c, const Args &a) {
  std::string dir = a.out + "/" + c.name;
  mkdir(dir.c_str(), 0755);
  sim::offset = norway;
  uint32_t local = unix_of(c.y, c.mo, c.d, c.h, c.mi);
  sim::utc0 = local - (uint32_t) norway(local - 3600);
  sim::now_ms = 1000;
  Options o;
  o.ground = c.ground;
  o.aurora = c.aurora;
  o.heater_pct = c.heater;
  o.fireplace = c.fireplace;
  static uint32_t rgb[W * H];

  // the picture as it is without the animations
  live::stop();
  flexit::panel::anim::reset();
  live::R.on = false;
  draw(c, a, o, nullptr);
  picture(rgb);
  write_ppm(dir + "/picture.ppm", rgb);
  int colours;
  size_t packed;
  stats(&colours, &packed);
  uint32_t pal[16];
  memcpy(pal, scene.pal, sizeof pal);

  // and with them, as the panel shows it
  live::R.on = true;
  draw(c, a, o, pal);
  int show = live::pick(scene);
  live::apply(scene);
  if (!a.real_timers)  // what happens by chance happens soon, one after the other, so a short run shows it
    for (size_t i = 0, k = 0; i < live::R.timers.size(); i++)
      if (live::R.timers[i].mean)
        live::R.timers[i].at = std::min(live::R.timers[i].at, sim::now_ms + 1500 + 6000 * (uint32_t) k++);
  printf("%-26s %-9s %2d colours, %5zu bytes packed%s  %s\n", c.name, GROUNDS[c.ground], colours, packed,
         packed > 13824 ? " (too big, a JPEG)" : "", show >= 0 ? live::SHOWS[show].name : "no show");
  if (a.check && c.ground == G_HEATH && show >= 0 && !strcmp(live::SHOWS[show].name, "Life on the heath")) {
    failed |= !check(c);
    return;
  }
  if (!a.anim || show < 0)
    return;
  picture(rgb);
  static uint32_t base[W * H];
  memcpy(base, rgb, sizeof base);
  int frames = a.seconds * a.fps, every = 1000 / a.fps, most = 0;
  uint32_t next = sim::now_ms;
  for (int k = 0; k < frames;) {
    live::loop(true);
    flexit::panel::anim::step(live::STEP);
    sim::now_ms += live::STEP;
    if (sim::now_ms >= next) {
      memcpy(rgb, base, sizeof base);
      flexit::panel::anim::draw(rgb, W, H);
      char f[32];
      snprintf(f, sizeof f, "/f%04d.ppm", k++);
      write_ppm(dir + f, rgb);
      next += (uint32_t) every;
      most = std::max(most, flexit::panel::anim::copies_shown());
    }
  }
  FILE *f = fopen((dir + "/fps").c_str(), "w");
  fprintf(f, "%d\n", a.fps);
  fclose(f);
  printf("%-26s %d s of animation, up to %d copies at a time\n", "", a.seconds, most);
}

int main(int argc, char **argv) {
  Args a;
  for (int i = 1; i < argc; i++) {
    std::string s = argv[i];
    auto next = [&] {
      return i + 1 < argc ? argv[++i] : (fprintf(stderr, "%s needs a value\n", s.c_str()), exit(2), "");
    };
    if (s == "--only")
      a.only = next();
    else if (s == "--seconds")
      a.seconds = atoi(next());
    else if (s == "--fps")
      a.fps = std::max(1, atoi(next()));
    else if (s == "--no-anim")
      a.anim = false;
    else if (s == "--real-timers")
      a.real_timers = true;
    else if (s == "--lat")
      a.lat = atof(next());
    else if (s == "--lon")
      a.lon = atof(next());
    else if (s == "--check")
      a.check = true;
    else if (s == "-v")
      sim::verbose = true;
    else if (s == "--list") {
      for (const Case &c : CASES)
        printf("%s\n", c.name);
      return 0;
    } else if (a.out.empty() && s[0] != '-')
      a.out = s;
    else {
      fprintf(stderr, "met_sim OUT [--only text] [--seconds 40] [--fps 10] [--no-anim] [--real-timers] [--lat --lon] "
                      "[--list] [-v]\n");
      return 2;
    }
  }
  if (a.out.empty()) {
    fprintf(stderr, "met_sim OUT ..., see --list for the cases\n");
    return 2;
  }
  mkdir(a.out.c_str(), 0755);
  for (const Case &c : CASES)
    if (a.only.empty() || strstr(c.name, a.only.c_str()))
      run(c, a);
  return failed;
}
