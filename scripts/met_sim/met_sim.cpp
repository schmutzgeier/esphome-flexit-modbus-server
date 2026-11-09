// Draws the weather picture of packages/code/met for mocked weathers, wave heights, places, landscapes and days,
// as met.h makes it: build(), what the animations leave out, palette(), the heath's fit, then the show that fits
// it, run on overlay_mock.h's model of the panel. Writes each case's picture and the frames of its animation as
// PPMs to OUT/<case>/, which render.py turns into PNGs, GIFs and a contact sheet. See run.sh.
// --check runs each heath case for check_minutes without frames and checks that the kayak keeps to the water and
// the dog's feet to the ground. --rare brings the rare encounter whose name has text two seconds in, whatever the
// odds and whether or not it fits. --turn has the heath's visitors start at that turn: 0 the reindeer, 1 the
// walrus, 2 the sea eagle, 3 the ravens.
//   met_sim OUT [--only text] [--seconds 40] [--fps 10] [--no-anim] [--real-timers] [--check] [--lat 60.39
//   --lon 5.32] [--rare text] [--turn n] [-v]
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

using flexit::panel::civil_from_days;
using flexit::panel::unix_of;
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
                         c.wind * (1 + 0.05f * (i % 4)), (int16_t) (((int) c.wind_from + 10 * i) % 360),
                         c.wind * (1.5f + 0.1f * (i % 3))};
  return w;
}

struct Args {
  std::string out, only;
  int seconds = 40, fps = 10;
  bool anim = true, real_timers = false, check = false, costs = false;
  int check_minutes = 20;
  std::string rare;  // an encounter to bring, see --rare
  int turn = -1;     // the heath's visitor to start with
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

static Stamps stamps;
static uint8_t ov[OV_H][W / 4];
static Land land;
static Scene scene;

// The scene for c at sim::now_ms, its 16 colours and what the animations leave out, as met.h's draw() makes it
static void draw(const Case &c, const Args &a, Options &o, const uint32_t *prev) {
  uint32_t t = sim::utc0 + sim::now_ms / 1000;
  Weather w = weather_of(c, t);
  o.frame++;
  scene.stamps = &stamps;
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
  auto &ix = flexit::panel::overlay::pic_index;
  ix.assign((size_t) W * H, 0);
  for (int y = 0; y < H; y++) {
    row(scene, y, r);
    for (int x = 0; x < W; x++) {
      rgb[y * W + x] = scene.pal[r[x]];
      ix[(size_t) y * W + x] = r[x];
    }
  }
  flexit::panel::overlay::pic_parts.clear();
  for (const auto &p : parts_of(scene))
    if (p.colours && p.x0 < p.x1 && p.y0 < p.y1)
      flexit::panel::overlay::pic_parts.push_back({p.x0, p.y0, p.x1, p.y1, p.colours, p.depth});
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
static flexit::panel::overlay::Copy *copy_of(const char *n, flexit::panel::overlay::Overlay **an = nullptr) {
  for (auto &p : flexit::panel::overlay::overlays)
    if (p.name == std::string("w_") + n)
      for (auto &c : p.copies)
        if (c.used && !c.fading) {
          if (an)
            *an = &p;
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
    overlay::step(live::STEP);
    steps++;
    if (overlay::Copy *k = kayak ? copy_of("hbay") : nullptr) {
      int x = (int) lroundf(k->base[overlay::X]), y = (int) lroundf(k->base[overlay::Y]);
      if (k->wave[overlay::Y].on)
        y += (int) lroundf(overlay::wave_value(k->wave[overlay::Y]));
      bool ok = true;
      for (int r = 12; r <= 14; r++)  // the hull and the stern, at the water
        for (int dx = 0; dx < 30; dx++)
          ok = ok && (art::KAYAKER.px[r * 8 + dx / 4] >> (dx % 4 * 2) & 3 ? sea_at(x + dx, y + r) : true);
      kayak_bad += !ok;
      wet++;
    }
    for (const char *n : live::DOG_NAMES)
      if (overlay::Copy *d = copy_of(n)) {
        int x = (int) lroundf(d->base[overlay::X]), y = (int) lroundf(d->base[overlay::Y]);
        int ground = legs_ground(x) + dog_d;
        if (abs(y + dog.feet - ground) > dog_worst && sim::verbose)
          printf("dog %s at %.1f,%.1f: feet %d, ground %d\n", n, d->base[overlay::X], d->base[overlay::Y], y + dog.feet,
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

// --costs: each show that fits the case's picture, the heath's aside, run on its own for seconds, and the most it
// took of the panel at a time, so the heath knows what it can have next to its own
static void costs(const Case &c, const Args &a) {
  namespace overlay = flexit::panel::overlay;
  for (int i = 0; i < live::NSHOWS; i++) {
    if (!live::SHOWS[i].fits(scene) || live::SHOWS[i].start == live::life_on_heath)
      continue;
    live::stop();
    overlay::reset();
    live::R.show = i;
    live::SHOWS[i].start(scene);
    int places = 0, slots = 0;
    size_t bytes = 0;
    for (uint32_t end = sim::now_ms + (uint32_t) a.seconds * 1000; sim::now_ms < end; sim::now_ms += live::STEP) {
      live::loop(true);
      overlay::step(live::STEP);
      size_t b = 0;
      places = std::max(places, overlay::copies_shown());
      slots = std::max(slots, overlay::slots_used(&b));
      bytes = std::max(bytes, b);
    }
    printf("%-26s cost of %-36s %d places, %d slots, %5zu bytes of frames\n", c.name, live::SHOWS[i].name, places,
           slots, bytes);
  }
  live::stop();
  overlay::reset();
}

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
  flexit::panel::overlay::reset();
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
  if (a.turn >= 0)
    live::HEATH.turn = a.turn;
  if (!a.rare.empty())
    live::later(2000, [r = a.rare] {
      for (int k = 0; k < live::NRARE; k++)
        if (strstr(live::RARE[k].name, r.c_str())) {
          bool fits = live::RARE[k].fits(live::SEEN), on = live::encounter(k, false);
          printf("%-26s %s: %s%s\n", "", live::RARE[k].name, on ? "on" : "no room",
                 fits ? "" : ", though it doesn't fit the picture now");
          return;
        }
      printf("no encounter called %s\n", r.c_str());
    });
  if (!a.real_timers)  // what happens by chance happens soon, one after the other, so a short run shows it
    for (size_t i = 0, k = 0; i < live::R.timers.size(); i++)
      if (live::R.timers[i].mean)
        live::R.timers[i].at = std::min(live::R.timers[i].at, sim::now_ms + 1500 + 6000 * (uint32_t) k++);
  printf("%-26s %-9s %2d colours, %5zu bytes packed%s  %s\n", c.name, GROUNDS[c.ground], colours, packed,
         packed > 13824 ? " (too big, a JPEG)" : "", show >= 0 ? live::SHOWS[show].name : "no show");
  if (a.costs) {
    costs(c, a);
    return;
  }
  if (a.check && c.ground == G_HEATH && show >= 0 && !strcmp(live::SHOWS[show].name, "Life on the heath")) {
    failed |= !check(c);
    return;
  }
  if (!a.anim || show < 0)
    return;
  picture(rgb);
  static uint32_t base[W * H];
  memcpy(base, rgb, sizeof base);
  int frames = a.seconds * a.fps, every = 1000 / a.fps, most = 0, slots = 0;
  size_t bytes = 0;
  uint32_t next = sim::now_ms;
  for (int k = 0; k < frames;) {
    live::loop(true);
    flexit::panel::overlay::step(live::STEP);
    sim::now_ms += live::STEP;
    if (sim::now_ms >= next) {
      memcpy(rgb, base, sizeof base);
      flexit::panel::overlay::draw(rgb, W, H);
      char f[32];
      snprintf(f, sizeof f, "/f%04d.ppm", k++);
      write_ppm(dir + f, rgb);
      next += (uint32_t) every;
      most = std::max(most, flexit::panel::overlay::copies_shown());
      flexit::panel::overlay::look();
      size_t b = 0;
      slots = std::max(slots, flexit::panel::overlay::slots_used(&b));
      bytes = std::max(bytes, b);
    }
  }
  FILE *f = fopen((dir + "/fps").c_str(), "w");
  fprintf(f, "%d\n", a.fps);
  fclose(f);
  printf("%-26s %d s of animation, up to %d copies, %d of %d slots and %zu bytes of frames at a time\n", "",
         a.seconds, most, slots, OVERLAY_SLOTS, bytes);
  for (auto &[name, px] : flexit::panel::overlay::wrapped)
    printf("%-26s %s drawn round the screen's edge, %d px\n", "", name.c_str(), px);
  if (sim::verbose)
    for (auto &[name, w] : flexit::panel::overlay::where)
      printf("%-26s %s: its middle at x %.0f on average, at the screen's edge %d%% of the time\n", "", name.c_str(),
             w.x / w.n, 100 * w.edge / w.n);
  for (auto &[name, n] : flexit::panel::overlay::crossed)
    if (n)
      printf("%-26s %s out at one edge and back at the other %d times\n", "", name.c_str(), n);
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
    else if (s == "--rare")
      a.rare = next();
    else if (s == "--turn")
      a.turn = atoi(next());
    else if (s == "--check")
      a.check = true;
    else if (s == "--costs")
      a.costs = true;
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
                      "[--rare text] [--turn n] [--list] [-v]\n");
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
