// The heath's wave height under the screensaver's date: Oceanforecast's answer read as it comes in, the hour
// wave_at() takes, and the text forecast_marks() writes in the bay, or doesn't. make -C tests met_wave_test
#include <cstring>
#include <string>
#include "check.h"
#include "met/met_font.h"
#include "met/met_heath.h"
#include "met/met_raster.h"

using namespace flexit::panel;
using namespace flexit::panel::met;

static int utc(uint32_t) { return 0; }

static Stamps stamps;
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

// What a Waves reader makes of s, handed to it in parts of k bytes
static Sea read_sea(const std::string &s, size_t k) {
  Waves r;
  for (size_t i = 0; i < s.size(); i += k)
    if (!r.feed(s.data() + i, std::min(k, s.size() - i)))
      break;
  return r.sea;
}

// The forecast over the heath at 15:00 UTC on 20 August 2026 in Bergen, waves m high
static marks::Layer build_heath(float waves, bool forecast = true, Ground g = G_HEATH) {
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
    w.hour[w.hours++] = {w.time + 3600u * i, 16, icon_of(w.symbol), 9, 250, 14};
  Options o;
  o.ground = g;
  scene.stamps = &stamps;
  scene.land = &land;
  build(scene, w, 60.39, 5.32, w.time, forecast, utc, o);
  palette(scene, nullptr);
  if (scene.ground == G_HEATH)
    heath_fit(scene, nullptr);
  return forecast_marks(scene, w, w.time, utc);
}

// The bay's ops: the ones under the forecast's rows, which end at OV_H. The text's and its shadow's.
static std::vector<const marks::Op *> bay_ops(const marks::Layer &l) {
  std::vector<const marks::Op *> v;
  for (const marks::Op &o : l.ops)
    if (o.box.y1 > OV_H)
      v.push_back(&o);
  return v;
}

// The pixels t is made of in FONT_TINY with its baseline at SEA_TEXT_BASE from x SEA_TEXT_X, as "x,y" keys, built
// straight from the font. dx, dy: moved.
static std::vector<std::pair<int, int>> expected(const char *t, int dx = 0, int dy = 0) {
  std::vector<std::pair<int, int>> px;
  const Font &f = FONT_TINY;
  int pen = SEA_TEXT_X;
  for (; *t; t++)
    for (int i = 0; i < f.n; i++) {
      const FontGlyph &g = f.glyphs[i];
      if (g.c != *t)
        continue;
      for (int r = 0; r < g.h; r++)
        for (int c = 0; c < g.w; c++)
          if (FONT_ROWS[g.row + r] >> (g.w - 1 - c) & 1)
            px.push_back({pen + g.x + c + dx, SEA_TEXT_BASE - g.y - g.h + r + dy});
      pen += g.adv;
    }
  std::sort(px.begin(), px.end());
  return px;
}

// The pixels the ops draw, sorted. Any op's pixels, in a box round the bay's text.
static std::vector<std::pair<int, int>> pixels(const std::vector<const marks::Op *> &ops, bool shadow) {
  std::vector<std::pair<int, int>> px;
  for (const marks::Op *o : ops) {
    if (o->kind != marks::Op::BITMAP || o->outline || (o->rgb == ops.back()->rgb) == shadow)
      continue;
    for (int y = H - 80; y < H; y++)
      for (int x = 0; x < W; x++)
        if (marks::bit(*o, x, y))
          px.push_back({x, y});
  }
  std::sort(px.begin(), px.end());
  return px;
}

// Whether the bay has t, and under its drop shadow: the text as the font draws it, last in the layer, and before
// the forecast's own letters the same again 2 px down and right in the shadow's colour
static bool same_text(const marks::Layer &l, const char *t) {
  auto ops = bay_ops(l);
  if (ops.size() != 2 || ops[0]->rgb == ops[1]->rgb)  // a run of the text and its shadow
    return false;
  return pixels(ops, false) == expected(t) && pixels(ops, true) == expected(t, 2, 2);
}

int main() {
  // Oceanforecast's hours, in parts that cut the keys in two too. The last hour is read once the next one starts,
  // so it waits for one more, and an hour without the height is skipped.
  std::string answer = "{\"type\":\"Feature\",\"properties\":{\"timeseries\":[";
  answer += hour(14, "\"sea_surface_wave_height\":1.2,") + hour(15, "\"sea_surface_wave_height\":1.8,") + hour(16, "") +
            hour(17, "\"sea_surface_wave_height\":-0.3,") + hour(18, "\"sea_surface_wave_height\":2.0,");
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

  // On the heath, with one decimal under 10 m and none from there, in the bay under the date: the text in the
  // font's tiny size with its baseline at SEA_TEXT_BASE, and its shadow
  for (auto [m, t] : {std::pair{1.8f, "1.8 m"}, {0.0f, "0.0 m"}, {9.94f, "9.9 m"}, {9.95f, "10 m"}, {12.4f, "12 m"}}) {
    marks::Layer l = build_heath(m);
    CHECK(scene.overlay);
    if (!same_text(l, t))
      printf("%.2f m isn't written as \"%s\"\n", m, t);
    CHECK(same_text(l, t));
    CHECK(expected(t).size() > 20);
  }
  CHECK(!same_text(build_heath(4.5f), "1.8 m"));

  // and drawn only there: the rest of the layer is the forecast, in the rows above
  marks::Layer with = build_heath(1.8f), without = build_heath(-1);
  CHECK(with.ops.size() == without.ops.size() + 2);
  for (const marks::Op *o : bay_ops(with)) {
    CHECK(o->box.y0 > OV_H && o->box.x0 >= SEA_TEXT_X - 4 && o->box.x1 <= SEA_TEXT_X + 64);
    CHECK(o->box.y1 <= H);
  }

  // None inland, where Oceanforecast has no waves, without the forecast, and on the other landscapes
  CHECK(bay_ops(without).empty());
  CHECK(build_heath(1.8f, false).ops.empty());
  for (Ground g : {G_HILLS, G_MOUNTAINS, G_CITY}) {
    CHECK(bay_ops(build_heath(1.8f, true, g)).empty());
    CHECK(build_heath(1.8f, true, g).ops.size() == build_heath(-1, true, g).ops.size());
  }

  printf("%s\n", fails ? "FAILED" : "met_wave_test: all passed");
  return fails != 0;
}
