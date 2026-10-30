// The heath's wave height under the screensaver's date: Oceanforecast's answer read an hour at a time, the hour
// wave_at() takes, and the text build() writes in the bay and row() draws, or doesn't. make -C tests met_wave_test
#include <cstring>
#include <string>
#include "check.h"
#include "met/met_heath.h"
#include "met/met_raster.h"

using namespace flexit::panel::met;

static int utc(uint32_t) { return 0; }

static Sprite sprites[SPRITES];
static uint8_t ov[OV_H][W / 4];
static Land land;
static Scene scene;

// An Oceanforecast hour, as its timeseries has them
static std::string hour(int h, const char *height) {
  char b[160];
  snprintf(b, sizeof b,
           "{\"time\":\"2026-08-20T%02d:00:00Z\",\"data\":{\"instant\":{\"details\":{%s\"sea_water_speed\":0.1}}}},", h,
           height);
  return b;
}

// What a SeaReader makes of s, handed to it in parts of k bytes
static Sea read_sea(const std::string &s, size_t k) {
  static char buf[SeaReader::SIZE + 1];
  SeaReader r(buf);
  for (size_t i = 0; i < s.size(); i += k) {
    size_t room, n = std::min(k, s.size() - i);
    char *at = r.space(room);
    CHECK(n <= room);
    memcpy(at, s.data() + i, n);
    if (!r.got(n))
      break;
  }
  return r.sea;
}

// The heath at 15:00 UTC on 20 August 2026 in Bergen, with the forecast, waves m high
static void build_heath(float waves, bool forecast = true, Ground g = G_HEATH) {
  Weather w;
  w.ok = true;
  w.time = unix_of(2026, 8, 20, 15);
  snprintf(w.symbol, sizeof w.symbol, "partlycloudy_day");
  w.temp = 16;
  w.cloud = 50;
  w.wind = 9;
  w.wind_from = 250;
  w.wave = waves;
  for (int i = 0; i < HOURS; i++)
    w.hour[w.hours++] = {w.time + 3600u * i, 16, icon_of(w.symbol), 9, 250};
  Options o;
  o.ground = g;
  scene.sprite = sprites;
  scene.ov = ov;
  scene.land = &land;
  build(scene, w, 60.39, 5.32, w.time, forecast, utc, o);
  palette(scene, nullptr);
  if (scene.ground == G_HEATH)
    heath_fit(scene, nullptr);
}

// The bay's text as heath_overlay() writes t, on its own
static const Heath &expected(const char *t) {
  static Scene s;
  memset(&s, 0, sizeof s);
  s.heath.sea_text = true;
  ov_text(s, SEA_TEXT_X, SEA_TEXT_BASE - (FONT_TINY.line_height - FONT_TINY.base_line), t, FONT_TINY);
  return s.heath;
}

static bool same_text(const char *t) { return !memcmp(scene.heath.sea_ov, expected(t).sea_ov, sizeof scene.heath.sea_ov); }

// The bay's pixels row() draws in the text's colour
static int text_pixels() {
  uint8_t r[W];
  int n = 0;
  for (int y = SEA_OV_Y; y < H; y++) {
    row(scene, y, r);
    for (int x = SEA_OV_X; x < SEA_OV_X + SEA_OV_W; x++)
      n += ov_get(scene, x, y) == 2 && r[x] == scene.ix[C_TEXT];
  }
  return n;
}

int main() {
  // Oceanforecast's hours, in parts that cut the keys in two too. The last hour is read once the next one starts,
  // so it waits for one more, and an hour without the height is skipped.
  std::string answer = "{\"type\":\"Feature\",\"properties\":{\"timeseries\":[";
  answer += hour(14, "\"sea_surface_wave_height\":1.2,") + hour(15, "\"sea_surface_wave_height\":1.8,") +
            hour(16, "") + hour(17, "\"sea_surface_wave_height\":-0.3,") + hour(18, "\"sea_surface_wave_height\":2.0,");
  answer += "{\"time\":\"2026-08-20T19:00:00Z\"}]}}";
  for (size_t k : {answer.size(), (size_t) 1, (size_t) 7, (size_t) 100}) {
    Sea sea = read_sea(answer, k);
    CHECK(sea.ok);
    CHECK(sea.hours == 4);
    CHECK(sea.time[0] == unix_of(2026, 8, 20, 14) && sea.height[0] == 1.2f);
    CHECK(sea.time[1] == unix_of(2026, 8, 20, 15) && sea.height[1] == 1.8f);
    CHECK(sea.time[2] == unix_of(2026, 8, 20, 17) && sea.height[2] == 0);  // never below 0
    CHECK(sea.time[3] == unix_of(2026, 8, 20, 18) && sea.height[3] == 2.0f);
  }
  CHECK(!read_sea("{\"type\":\"Feature\",\"properties\":{\"timeseries\":[]}}", 16).ok);  // inland

  // The hour nearest t, if t's within three hours of it
  Sea sea = read_sea(answer, answer.size());
  uint32_t t15 = unix_of(2026, 8, 20, 15);
  CHECK(wave_at(sea, t15 + 1800) == 1.8f);
  CHECK(wave_at(sea, t15 + 2400) == 1.8f);
  CHECK(wave_at(sea, t15 - 600) == 1.2f);
  CHECK(wave_at(sea, unix_of(2026, 8, 20, 22)) == 2.0f);  // 3 hours after the 18:00 hour ends
  CHECK(wave_at(sea, unix_of(2026, 8, 20, 22) + 1) == -1);
  CHECK(wave_at(sea, unix_of(2026, 8, 20, 11)) == 1.2f);  // and before the 14:00 one starts
  CHECK(wave_at(sea, unix_of(2026, 8, 20, 11) - 1) == -1);
  CHECK(wave_at(Sea{}, t15) == -1);

  // On the heath, with one decimal under 10 m and none from there, in the bay under the date
  for (auto [m, t] : {std::pair{1.8f, "1.8 m"}, {0.0f, "0.0 m"}, {9.94f, "9.9 m"}, {9.95f, "10 m"}, {12.4f, "12 m"}}) {
    build_heath(m);
    CHECK(scene.overlay && scene.heath.sea_text);
    if (!same_text(t))
      printf("%.2f m isn't written as \"%s\"\n", m, t);
    CHECK(same_text(t));
    CHECK(text_pixels() > 20);
  }
  build_heath(4.5f);
  CHECK(!same_text("1.8 m"));

  // and drawn only there: the bay's other rows and the rest of the picture keep theirs
  build_heath(1.8f);
  for (int y = OV_H; y < SEA_OV_Y; y++)
    for (int x = 0; x < W; x++)
      CHECK(ov_get(scene, x, y) == 0);
  for (int y = SEA_OV_Y; y < H; y++)
    for (int x = SEA_OV_X + SEA_OV_W; x < W; x++)
      CHECK(ov_get(scene, x, y) == 0);

  // None inland, where Oceanforecast has no waves, without the forecast, and on the other landscapes
  build_heath(-1);
  CHECK(!scene.heath.sea_text && text_pixels() == 0);
  build_heath(1.8f, false);
  CHECK(!scene.heath.sea_text && text_pixels() == 0);
  for (Ground g : {G_HILLS, G_COAST, G_MOUNTAINS, G_CITY}) {
    build_heath(1.8f, true, g);
    CHECK(!scene.heath.sea_text && text_pixels() == 0);
  }

  printf("%s\n", fails ? "FAILED" : "met_wave_test: all passed");
  return fails != 0;
}
