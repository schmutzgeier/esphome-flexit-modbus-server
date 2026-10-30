// MET Norway's Locationforecast 2.0, read as it comes in. Plain C++ without ESPHome, see met_scene.h.
#pragma once
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace flexit::panel::met {

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

inline constexpr int HOURS = 18;
// MET symbol codes go up to 42 chars, lightssleetshowersandthunder_polartwilight (sic). Longer ones are cut.
inline constexpr size_t SYMBOL = 48;

struct Weather {
  bool ok = false;
  uint32_t time = 0;  // forecast hour, UTC seconds since 1970
  float temp = 10, cloud = 0, cloud_low = 0, cloud_mid = 0, cloud_high = 0, fog = 0;  // %
  float rain = 0;                                                                     // mm in the coming hour
<<<<<<< ours
  float wind = 0, wind_from = 0, thunder = 0;                                         // m/s, degrees, %
  float wave = -1;           // significant wave height in m, from Oceanforecast (see Sea). -1 = none
=======
  float wind = 0, gust = 0, wind_from = 0, thunder = 0;                               // m/s, m/s, degrees, %
>>>>>>> theirs
  char symbol[SYMBOL] = "";  // MET symbol for the coming hour, like "lightrainshowers_day"
  Hour hour[HOURS] = {};     // this hour onwards, as far as the response went
  int hours = 0;
};

// MET symbol code to our Icon, see https://api.met.no/weatherapi/weathericon/2.0/documentation
Icon icon_of(const char *s);

// The forecast hour that covers t, the first hour's weather before the hours start
Hour now_of(const Weather &w, uint32_t t);

// When the forecast has run out, an hour after its last hour
inline uint32_t ends(const Weather &w) { return (w.hours ? w.hour[w.hours - 1].time : w.time) + 3600; }

// Reads a Locationforecast 2.0 answer as it comes in, an hour at a time, so it needs only a small buffer. The
// first hour is the weather now, it and the ones after it are for the overlay. "complete" has the cloud layers
// and fog, "compact" doesn't.
struct Reader {
  static constexpr size_t SIZE = 4096;  // an hour is about 1.2 KB
  char *buf;                            // SIZE + 1 bytes
  size_t n = 0;
  bool series = false;  // past "timeseries":[
  Weather w;            // w.ok once it has the first hour

  explicit Reader(char *b) : buf(b) { buf[0] = 0; }
  // Where the next bytes go, and how many fit
  char *space(size_t &room) {
    room = SIZE - n;
    return buf + n;
  }
  // n bytes came in at space(). False once it has HOURS hours, or what came can't be read.
  bool got(size_t k);

 private:
  void drop(size_t k);
  // One hour, from its {"time" to the next one. False when no more are needed.
  bool hour(std::string_view e);
};

// The waves from Oceanforecast 2.0: the significant wave height, hourly from about now. MET Norway has them for
// points at sea, and points near the coast go to the nearest one; inland there are none.
inline constexpr int SEA_HOURS = 24;
struct Sea {
  bool ok = false;
  int hours = 0;
  uint32_t time[SEA_HOURS] = {};  // UTC
  float height[SEA_HOURS] = {};   // m, the mean of the highest third of the waves
};

// The wave height at t, UTC: the hour nearest it, if that's within three hours. -1 without one.
float wave_at(const Sea &sea, uint32_t t);

// Reads an Oceanforecast 2.0 answer as Reader does a forecast, an hour at a time
struct SeaReader {
  static constexpr size_t SIZE = 1024;  // an hour is about 250 bytes
  char *buf;                            // SIZE + 1 bytes
  size_t n = 0;
  Sea sea;

  explicit SeaReader(char *b) : buf(b) { buf[0] = 0; }
  char *space(size_t &room) {
    room = SIZE - n;
    return buf + n;
  }
  // n bytes came in at space(). False once it has SEA_HOURS hours, or what came can't be read.
  bool got(size_t k);

 private:
  void drop(size_t k);
};

}  // namespace flexit::panel::met
