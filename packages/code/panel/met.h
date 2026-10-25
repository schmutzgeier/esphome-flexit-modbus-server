// met_scene.h as a panel picture. met.yaml calls fetch() for the forecast, fetch_sea() for the waves, and redraw()
// every few minutes so the sun and moon move. Only changed rows are sent to the panel.
#pragma once
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include "items.h"
#include "met_scene.h"
#include "picture.h"

// True in fireplace mode. Weak, since packages/code/fireplace.h only defines it when that package is included.
extern "C" bool flexit_fireplace_active() __attribute__((weak));

namespace flexit::panel::met {

inline Weather weather;  // forecast for the coming hour
inline Sea sea;          // the waves, from Oceanforecast. Not ok inland, where MET Norway has none

// The dashboard tile's label and icon for each Icon
static const char *const KIND_NAMES[] = {"Weather|Vær",    "Clear|Klart",    "Clear|Klart", "Fair|Lettskyet",
                                         "Fair|Lettskyet", "Cloudy|Skyet",   "Rain|Regn",   "Snow|Snø",
                                         "Sleet|Sludd",    "Thunder|Torden", "Fog|Tåke"};
static const char *const KIND_ICONS[] = {"cloud", "sun",   "night", "sun",  "night", "cloud",
                                         "rain",  "snowy", "rain",  "bolt", "cloud"};
static_assert(sizeof KIND_NAMES / sizeof *KIND_NAMES == I_FOG + 1 &&
                  sizeof KIND_ICONS / sizeof *KIND_ICONS == I_FOG + 1,
              "a name and an icon for each of met_scene.h's Icons");
inline int now_item = -1;

inline void send_now() {
  if (!weather.ok)
    return;
  Icon k = icon_of(weather.symbol);
  item_label(now_item, KIND_NAMES[k]);
  item_icon(now_item, KIND_ICONS[k]);
  item_value(now_item, weather.temp);
}
inline std::string modified;  // Last-Modified of our forecast, for If-Modified-Since. Empty until we have one
inline uint32_t not_before;   // no request before this (Unix time): the last answer's Expires, or a back-off
inline uint32_t backoff_s;    // doubles with each refusal in a row
inline bool answered;         // MET Norway has answered since boot
inline uint8_t early_tries;   // quick retries before that
inline char url[128];
// The same for the waves. Inland MET Norway has none, so after a 422 we ask again the next day.
inline std::string sea_modified;
inline uint32_t sea_not_before, sea_backoff_s;
inline char sea_url[128];
inline double lat, lon;
inline std::string name;
inline bool with_forecast;  // the Panel Weather Forecast switch
inline Options options;     // set from met.yaml

inline Aurora aurora_of(const char *name) {
  return !strcmp(name, "off") ? A_OFF : !strcmp(name, "always") ? A_ALWAYS : A_AUTO;
}
// Panel Weather in met.yaml. Not published here directly, since this header is built without met.yaml too.
inline std::function<void(const std::string &)> on_status;

// The scene, only while picture.h makes the picture. Its sprites and overlay are blocks of their own.
struct Canvas {
  Scene scene;
  uint32_t rows[2][W];  // two cached rows, since at half size picture.h reads two at a time
  int row_y[2], row_next;
};
inline Canvas *canvas;
inline bool built;  // canvas has the scene for now

inline void free_canvas() {
  if (canvas) {
    RAMAllocator<Sprite>().deallocate(canvas->scene.sprite, SPRITES);
    RAMAllocator<uint8_t>().deallocate(canvas->scene.ov[0], OV_H * W / 4);
    RAMAllocator<Canvas>().deallocate(canvas, 1);
  }
  canvas = nullptr;
  built = false;
}

inline bool new_canvas() {
  Canvas *c = RAMAllocator<Canvas>().allocate(1);
  Sprite *sprite = RAMAllocator<Sprite>().allocate(SPRITES);
  uint8_t *ov = RAMAllocator<uint8_t>().allocate(OV_H * W / 4);
  if (!c || !sprite || !ov) {
    RAMAllocator<Canvas>().deallocate(c, 1);
    RAMAllocator<Sprite>().deallocate(sprite, SPRITES);
    RAMAllocator<uint8_t>().deallocate(ov, OV_H * W / 4);
    return false;
  }
  c->scene.sprite = sprite;
  c->scene.ov = (uint8_t (*)[W / 4]) ov;
  canvas = c;
  return true;
}

// Builds the scene for now if it isn't. False without the time or the RAM, picture.h asks again a second later.
inline bool draw() {
  if (built)
    return true;
  ESPTime t = id(flexit_panel_time).now();
  if (!t.is_valid())
    return false;
  if (!canvas && !new_canvas()) {
    picture::S.waiting = "no RAM to draw it";
    return false;
  }
#pragma GCC diagnostic push
// null without the fireplace package, but with it GCC warns the check is always true
#pragma GCC diagnostic ignored "-Waddress"
  options.fireplace = flexit_fireplace_active ? flexit_fireplace_active() : false;
#pragma GCC diagnostic pop
  weather.wave = wave_at(sea, (uint32_t) t.timestamp);
  build(canvas->scene, weather, lat, lon, (uint32_t) t.timestamp, with_forecast, ESPTime::timezone_offset(), options);
  canvas->row_y[0] = canvas->row_y[1] = -1;
  canvas->row_next = 0;
  built = true;
  return true;
}

// The time, the weather or a setting changed. Drawn again if it's the panel's picture.
inline void redraw() {
  built = false;
  picture::changed(name);
}

// The picture's pixel as 0xRRGGBB
inline uint32_t pixel(int x, int y) {
  if (!canvas || x < 0 || y < 0 || y >= H)
    return 0;
  Canvas &c = *canvas;
  int k = c.row_y[0] == y ? 0 : c.row_y[1] == y ? 1 : -1;
  if (k < 0) {
    k = c.row_next;
    c.row_next ^= 1;
    row(c.scene, y, c.rows[k]);
    c.row_y[k] = y;
  }
  return c.rows[k][x < W ? x : W - 1];
}

// An HTTP date like "Mon, 19 Oct 2026 11:34:38 GMT" as Unix time, 0 if it isn't one
inline uint32_t http_date(const std::string &s) {
  static const char MONTHS[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {};
  int d, y, h, mi, sec;
  if (sscanf(s.c_str(), "%*3s, %d %3s %d %d:%d:%d", &d, mon, &y, &h, &mi, &sec) != 6)
    return 0;
  const char *m = strstr(MONTHS, mon);
  if (!m || strlen(mon) != 3 || (m - MONTHS) % 3)
    return 0;
  return days_from_civil(y, (int) (m - MONTHS) / 3 + 1, d) * 86400u + (uint32_t) (h * 3600 + mi * 60 + sec);
}

// Whether to ask MET Norway now. They want no new request before the last answer's Expires.
// Offline it waits for the next check, a failed request would back off for an hour.
inline bool due() {
  ESPTime t = id(flexit_panel_time).now();
  return network::is_connected() && t.is_valid() && (uint32_t) t.timestamp >= not_before;
}

// After an error or no answer at all. Wait an hour, then twice as long each time, up to 12 hours.
inline void back_off() {
  backoff_s = backoff_s ? std::min<uint32_t>(2 * backoff_s, 12 * 3600) : 3600;
  not_before = (uint32_t) id(flexit_panel_time).now().timestamp + backoff_s;
}

// met.yaml's on_error, no answer at all. Each try holds up the main loop until it times out. Right after boot the
// network often isn't quite there yet, so the first few tries just go again at the next check.
inline void unreachable() {
  if (!answered && early_tries < 3) {
    early_tries++;
    not_before = 0;
  } else {
    back_off();
  }
  if (on_status)
    on_status("Couldn't reach MET Norway");
}

// api.met.no's answer, 2xx with the forecast read from it into w, or 304 if it hasn't changed. 203 is a product
// that's deprecated, its data is still good.
inline void fetched(int status, const Weather &w, const std::string &last_modified, const std::string &expires) {
  answered = true;
  bool ok = status >= 200 && status < 300 && w.ok;
  if (ok || status == 304) {
    backoff_s = 0;
    uint32_t now = (uint32_t) id(flexit_panel_time).now().timestamp, e = http_date(expires);
    not_before = e ? std::max(e, now + 600) : now + 3600;  // at least 10 min, in case our clock is off
  } else {
    back_off();  // 429 is too many requests, 403 refused
  }
  if (status == 304) {
    ESP_LOGI("met", "The forecast hasn't changed (HTTP 304)");
    return;
  }
  if (status == 203) {
    ESP_LOGW("met", "MET Norway says this product is deprecated");
  }
  if (!ok) {
    ESP_LOGW("met", "No forecast from MET Norway (HTTP %d)", status);
    if (on_status)
      on_status("No forecast (HTTP " + std::to_string(status) + ")");
    return;
  }
  weather = w;
  if (!last_modified.empty())
    modified = last_modified;
  char s[96];
  snprintf(s, sizeof s, "%s, %.1f °C, clouds %.0f %%, %.1f mm", w.symbol[0] ? w.symbol : "?", w.temp, w.cloud, w.rain);
  ESP_LOGI("met", "Weather: %s", s);
  if (on_status)
    on_status(s);
  send_now();
  redraw();
}

// A GET with If-Modified-Since once we have a Last-Modified, read into rd until it has what it wants. The HTTP
// status, or -1 without an answer.
template<typename R>
int request(const char *to, const std::string &since, R &rd, std::string &last_modified, std::string &expires) {
  std::vector<http_request::Header> headers;
  if (!since.empty())
    headers.push_back({"If-Modified-Since", since});
  auto c = id(panel_http).get(to, headers, {"last-modified", "expires"});
  if (!c)
    return -1;
  uint32_t last = millis();
  for (;;) {
    size_t room;
    char *p = rd.space(room);
    int r = c->read((uint8_t *) p, std::min<size_t>(room, 512));
    App.feed_wdt();
    auto res = http_request::http_read_loop_result(r, last, id(panel_http).get_timeout(), c->is_read_complete());
    if (res == http_request::HttpReadLoopResult::RETRY)
      continue;
    if (res != http_request::HttpReadLoopResult::DATA || !rd.got((size_t) r))
      break;
  }
  int status = c->status_code;
  last_modified = c->get_response_header("last-modified");
  expires = c->get_response_header("expires");
  c->end();
  return status;
}

// The forecast, read an hour at a time until there are HOURS of them, into a buffer we may not get rather than a
// std::string that aborts the ESP without the RAM.
inline void fetch() {
  RAMAllocator<char> ram;
  char *buf = ram.allocate(Reader::SIZE + 1);
  if (!buf) {
    ESP_LOGW("met", "No RAM for the forecast, %u bytes", (unsigned) Reader::SIZE);
    not_before = (uint32_t) id(flexit_panel_time).now().timestamp + 600;
    if (on_status)
      on_status("No RAM for the forecast");
    return;
  }
  Reader rd(buf);
  std::string last_modified, expires;
  int status = request(url, modified, rd, last_modified, expires);
  ram.deallocate(buf, Reader::SIZE + 1);
  if (status < 0)
    unreachable();
  else
    fetched(status, rd.w, last_modified, expires);
}

// Whether to ask for the waves now: once MET Norway has answered for the forecast, and not before the last
// answer's Expires
inline bool sea_due() {
  ESPTime t = id(flexit_panel_time).now();
  return answered && network::is_connected() && t.is_valid() && (uint32_t) t.timestamp >= sea_not_before;
}

// Oceanforecast's answer, read into s, as fetched() for the forecast. -1 is no answer. Inland it's a 422, or a
// document without waves; then there are none and we ask again tomorrow.
inline void sea_fetched(int status, const Sea &s, const std::string &last_modified, const std::string &expires) {
  uint32_t now = (uint32_t) id(flexit_panel_time).now().timestamp, e = http_date(expires);
  bool ok = status >= 200 && status < 300 && s.ok;
  if (ok || status == 304) {
    sea_backoff_s = 0;
    sea_not_before = e ? std::max(e, now + 600) : now + 3600;
  } else if (status == 422 || status == 400 || status == 404 || (status >= 200 && status < 300)) {
    sea_backoff_s = 0;
    sea_not_before = now + 86400;
    bool had = sea.ok;
    sea = Sea{};
    if (had)
      redraw();
    ESP_LOGI("met", "No waves here (HTTP %d), MET Norway has them for points at sea", status);
    return;
  } else {
    sea_backoff_s = sea_backoff_s ? std::min<uint32_t>(2 * sea_backoff_s, 12 * 3600) : 3600;
    sea_not_before = now + sea_backoff_s;
    ESP_LOGW("met", "No waves from MET Norway (HTTP %d)", status);
    return;
  }
  if (status == 304)
    return;
  sea = s;
  if (!last_modified.empty())
    sea_modified = last_modified;
  ESP_LOGI("met", "Waves: %.1f m", wave_at(sea, now));
  redraw();
}

inline void fetch_sea() {
  RAMAllocator<char> ram;
  char *buf = ram.allocate(SeaReader::SIZE + 1);
  if (!buf) {
    sea_not_before = (uint32_t) id(flexit_panel_time).now().timestamp + 600;
    return;
  }
  SeaReader rd(buf);
  std::string last_modified, expires;
  int status = request(sea_url, sea_modified, rd, last_modified, expires);
  ram.deallocate(buf, SeaReader::SIZE + 1);
  sea_fetched(status, rd.sea, last_modified, expires);
}

inline void setup(double latitude, double longitude, const char *picture, const char *encoding, const char *quality) {
  lat = latitude;
  lon = longitude;
  // MET Norway wants at most 4 decimals, more can get a 403
  snprintf(url, sizeof url, "https://api.met.no/weatherapi/locationforecast/2.0/complete?lat=%.4f&lon=%.4f", lat, lon);
  snprintf(sea_url, sizeof sea_url, "https://api.met.no/weatherapi/oceanforecast/2.0/complete?lat=%.4f&lon=%.4f", lat,
           lon);
  name = picture;
  now_item = Item("weather", ITEM_SENSOR)
                 .view(false)
                 .menu(false)
                 .icon(KIND_ICONS[0])
                 .label(KIND_NAMES[0])
                 .unit("°C")
                 .decimals(1)
                 .range(-50, 50)
                 .add();
  picture::Source s;
  s.size = [](int &w, int &h) {
    w = W;
    h = H;
    return draw();
  };
  s.pixel = pixel;
  s.done = free_canvas;
  s.encoding = picture::encoding_of(encoding);
  s.quality = picture::quality_of(quality);
  picture::add(name, s);
}

}  // namespace flexit::panel::met
