// Stands in for packages/code/panel/animation.h when met_sim builds the weather animations on a PC: the ESP's side
// of the link becomes a small model of what the panel does with an animation's copies, after plink_keys.h's
// K_ANIM_* (place, glide, drift, wander, face, flip, wave, map, rounds, fades), on a clock of its own. It also gives
// the little of ESPHome that met_live*.h use: millis(), the logs, the time and picture::has_ram().
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include "plink_keys.h"  // the panel's ANIM_* limits, copied next to it by run.sh

// ---------------------------------------------------------------- ESPHome's

namespace sim {
inline uint32_t now_ms = 1000;   // millis()
inline uint32_t utc0;            // the scene's time, UTC, at now_ms = 0
inline int (*offset)(uint32_t);  // local time minus UTC
inline bool verbose;
inline void log(char level, const char *tag, const char *fmt, ...) {
  if (!verbose && level != 'W')
    return;
  va_list a;
  va_start(a, fmt);
  fprintf(stderr, "[%c][%s] ", level, tag);
  vfprintf(stderr, fmt, a);
  fputc('\n', stderr);
  va_end(a);
}
}  // namespace sim

inline uint32_t millis() { return sim::now_ms; }
#define ESP_LOGI(tag, ...) sim::log('I', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) sim::log('W', tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) sim::log('D', tag, __VA_ARGS__)

struct ESPTime {
  int year, month, day_of_month, hour, minute, second;
  uint32_t timestamp;
  bool is_valid() const { return true; }
  static ESPTime from_epoch_local(time_t t) {
    time_t l = t + sim::offset((uint32_t) t);
    tm g;
    gmtime_r(&l, &g);
    return {g.tm_year + 1900, g.tm_mon + 1, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec, (uint32_t) t};
  }
};
struct SimClock {
  ESPTime now() const { return ESPTime::from_epoch_local(sim::utc0 + sim::now_ms / 1000); }
};
inline SimClock sim_clock;
inline SimClock *flexit_panel_time = &sim_clock;
template<class T> T &id(T *p) { return *p; }

namespace flexit::panel::picture {
inline bool has_ram(size_t, size_t) { return true; }
}  // namespace flexit::panel::picture

// ---------------------------------------------------------------- the panel's animations

namespace flexit::panel::anim {

struct Source {
  int w, h, frames, frame_ms, x, y, scale;
  std::function<uint32_t(int, int, int)> pixel;
  int vx = 0, vy = 0;
  std::function<void(bool)> on_shown = {};
  int fade_ms = 0;
  std::vector<uint16_t> delays = {};
};
inline std::map<std::string, Source> sources;
inline void add(const std::string &name, Source s) { sources[name] = std::move(s); }

enum Phase { IDLE, COUNT, MAKE, SEND };
struct Work {
  Phase phase = COUNT;  // so met_live.h keeps the frame it drew
};
inline Work W;

enum { X, Y, TURN, SIZE, OPACITY, SPEED, CHANNELS };
inline const char *const CHANNEL_NAMES[] = {"x", "y", "turn", "size", "opacity", "speed"};
inline int channel_of(const std::string &s) {
  for (int i = 0; i < CHANNELS; i++)
    if (s == CHANNEL_NAMES[i])
      return i;
  return -1;
}
// 0 none, 1 in, 2 out, 3 both
inline int ease_of(const std::string &s) { return s == "in" ? 1 : s == "out" ? 2 : s == "both" ? 3 : 0; }
inline float eased(float t, int e) {
  t = std::clamp(t, 0.0f, 1.0f);
  if (e == 3)
    return t * t * (3 - 2 * t);
  if (e == 1)
    return t * t;
  if (e == 2)
    return 1 - (1 - t) * (1 - t);
  return t;
}

inline std::mt19937 rng{7};
inline float rnd(float a, float b) { return std::uniform_real_distribution<float>(a, std::max(a, b))(rng); }

struct Glide {
  bool on = false;
  float from = 0, to = 0;
  uint32_t t0 = 0, ms = 0;
  int ease = 0;
};
struct Wave {
  bool on = false;
  int lo, hi, rise, fall, hold_hi, hold_lo, jitter;
  bool random;
  int slow;  // 1 eases at low, 2 at high
  // the leg it's on: from a to b over ms, or holding
  float a = 0, b = 0, cur = 0;
  uint32_t t0 = 0, ms = 0;
  int step = 0;  // 0 up, 1 hold high, 2 down, 3 hold low
};
struct Map {
  bool on = false;
  int l, r, t, b;
};

// A copy on the panel
struct Copy {
  bool used = false, gone = false;
  float base[CHANNELS] = {0, 0, 0, 100, 100, 100};
  Glide glide[CHANNELS];
  float drift[3] = {};  // px or degrees a second
  bool own = false;
  int start = 0;
  uint32_t placed = 0;
  int fade_in = 300;
  // fading out: from fade_t0 over fade_ms, then gone
  bool fading = false;
  uint32_t fade_t0 = 0;
  int fade_ms = 0;
  int ax0 = -32768, ay0 = -32768, ax1 = 32767, ay1 = 32767, ex = 0, ey = 0;  // area and edges, 0 screen
  bool area = false;
  struct {
    bool on = false;
    float s0, s1;
    int heading, spread, l0, l1, r0, r1;
    float vx = 0, vy = 0;
    bool resting = true;
    uint32_t until = 0;
  } wander;
  bool face = false, face_mirror = false;
  int facing = 0;
  bool mirrored = false;  // by face
  bool flip_x = false, flip_y = false;
  Wave wave[CHANNELS];
  Map map[CHANNELS];
  int rounds = 0;
  uint32_t rounds_end = 0;
  float px = 0, py = 0;  // where it was a step ago, for face
};

struct Anim {
  std::string name;
  Source src;
  uint32_t t0;
  bool cleared = false;
  Copy copies[8];
  std::map<int, std::vector<uint32_t>> frames;  // drawn, by frame
  const std::vector<uint32_t> &frame(int f) {
    auto it = frames.find(f);
    if (it == frames.end()) {
      std::vector<uint32_t> px((size_t) src.w * src.h);
      for (int y = 0; y < src.h; y++)
        for (int x = 0; x < src.w; x++)
          px[(size_t) y * src.w + x] = src.pixel(f, x, y);
      it = frames.emplace(f, std::move(px)).first;
    }
    return it->second;
  }
  int frame_ms(int f) const { return src.delays.empty() ? src.frame_ms : src.delays[f]; }
  int total_ms() const {
    int t = 0;
    for (int f = 0; f < src.frames; f++)
      t += frame_ms(f);
    return std::max(1, t);
  }
  // the frame at ms into its frames
  int frame_at(uint32_t ms) const {
    int t = (int) (ms % (uint32_t) total_ms());
    for (int f = 0; f < src.frames; f++) {
      if (t < frame_ms(f))
        return f;
      t -= frame_ms(f);
    }
    return 0;
  }
};
inline std::vector<std::unique_ptr<Anim>> anims;

inline Anim *find(const std::string &name) {
  for (auto &a : anims)
    if (a->name == name && !a->cleared)
      return a.get();
  return nullptr;
}
inline Anim *want(const std::string &name) {
  if (Anim *a = find(name))
    return a;
  auto s = sources.find(name);
  if (s == sources.end())
    return nullptr;
  anims.push_back(std::make_unique<Anim>());
  Anim *a = anims.back().get();
  a->name = name;
  a->src = s->second;
  a->t0 = millis();
  return a;
}
inline Copy *copy_of(const std::string &name, int i) {
  Anim *a = find(name);
  return a && i >= 0 && i < 8 && a->copies[i].used && !a->copies[i].fading ? &a->copies[i] : nullptr;
}

inline int slot_for(const std::string &name) { return want(name) ? 0 : -1; }

inline void place(const std::string &name, int i, int x, int y, bool own, int start, int fade = -1) {
  Anim *a = want(name);
  if (!a || i < 0 || i >= 8)
    return;
  Copy &c = a->copies[i];
  c = Copy();
  c.used = true;
  c.base[X] = c.px = (float) x;
  c.base[Y] = c.py = (float) y;
  c.own = own;
  c.start = start;
  c.placed = millis();
  c.fade_in = fade >= 0 ? fade : a->src.fade_ms;
}
inline void fade_out(Anim &a, Copy &c, int fade) {
  c.fading = true;
  c.fade_t0 = millis();
  c.fade_ms = fade >= 0 ? fade : a.src.fade_ms;
}
inline void remove(const std::string &name, int i, int fade = -1) {
  Anim *a = find(name);
  if (a && i >= 0 && i < 8 && a->copies[i].used && !a->copies[i].fading)
    fade_out(*a, a->copies[i], fade);
}
inline void clear(const std::string &name) {
  if (Anim *a = find(name)) {
    for (Copy &c : a->copies)
      if (c.used && !c.fading)
        fade_out(*a, c, -1);
    a->cleared = true;
  }
}
inline void glide(const std::string &name, int i, const std::string &what, int to, int ms, const std::string &ease) {
  Copy *c = copy_of(name, i);
  int ch = channel_of(what);
  if (!c || ch < 0)
    return;
  c->glide[ch] = {true, c->base[ch], (float) to, millis(), (uint32_t) ms, ease_of(ease)};
  if (ch <= TURN)
    c->drift[ch] = 0;
  if (ch <= Y)
    c->wander.on = false;
  if (!ms) {
    c->base[ch] = (float) to;
    c->glide[ch].on = false;
  }
}
inline void move(const std::string &name, int i, int x, int y, int ms, const std::string &ease) {
  glide(name, i, "x", x, ms, ease);
  glide(name, i, "y", y, ms, ease);
}
inline void drift(const std::string &name, int i, float vx, float vy) {
  if (Copy *c = copy_of(name, i)) {
    c->drift[X] = vx;
    c->drift[Y] = vy;
    c->glide[X].on = c->glide[Y].on = c->wander.on = false;
  }
}
inline void flip(const std::string &name, int i, bool x, bool y) {
  if (Copy *c = copy_of(name, i)) {
    c->flip_x = x;
    c->flip_y = y;
  }
}
inline void area(const std::string &name, int i, int l, int t, int r, int b, const std::string &ex,
                 const std::string &ey) {
  auto edge = [](const std::string &e) { return e == "cross" ? 1 : e == "bounce" ? 2 : 0; };
  if (Copy *c = copy_of(name, i)) {
    c->area = true;
    c->ax0 = l, c->ay0 = t, c->ax1 = r, c->ay1 = b;
    c->ex = edge(ex), c->ey = edge(ey);
  }
}
inline void wander(const std::string &name, int i, float s0, float s1, int heading, int spread, int l0, int l1, int r0,
                   int r1) {
  if (Copy *c = copy_of(name, i)) {
    auto &w = c->wander;
    w = {};
    w.on = true;
    w.s0 = s0, w.s1 = s1, w.heading = heading, w.spread = spread, w.l0 = l0, w.l1 = l1, w.r0 = r0, w.r1 = r1;
    c->glide[X].on = c->glide[Y].on = false;
    c->drift[X] = c->drift[Y] = 0;
  }
}
inline void face(const std::string &name, int i, int facing, bool mirror, int, int, bool) {
  if (Copy *c = copy_of(name, i)) {
    c->face = true;
    c->face_mirror = mirror;
    c->facing = facing;
  }
}
inline void start_leg(Wave &v, int step) {
  v.step = step;
  v.t0 = millis();
  float k = 1 + rnd(-v.jitter, v.jitter) / 100.0f;
  if (step == 0 || step == 2) {
    v.a = v.cur;
    float to = step == 0 ? (float) v.hi : (float) v.lo;
    if (v.random)
      to = step == 0 ? rnd(v.cur, (float) v.hi) : rnd((float) v.lo, v.cur);
    v.b = to;
    float span = v.hi > v.lo ? std::fabs(to - v.cur) / (float) (v.hi - v.lo) : 1;
    v.ms = (uint32_t) ((step == 0 ? v.rise : v.fall) * span * k);
  } else {
    v.ms = (uint32_t) ((step == 1 ? v.hold_hi : v.hold_lo) * k);
  }
}
inline void wave(const std::string &name, int i, const std::string &what, int lo, int hi, int rise, int fall,
                 int hold_high, int hold_low, int jitter, bool random, const std::string &slow) {
  Copy *c = copy_of(name, i);
  int ch = channel_of(what);
  if (!c || ch < 0)
    return;
  Wave &v = c->wave[ch];
  v = {};
  v.on = true;
  v.lo = lo, v.hi = hi, v.rise = rise, v.fall = fall, v.hold_hi = hold_high, v.hold_lo = hold_low;
  v.jitter = jitter, v.random = random;
  v.slow = slow == "low" ? 1 : slow == "high" ? 2 : slow == "both" ? 3 : 0;
  v.cur = (float) lo;
  start_leg(v, 0);
}
inline void map(const std::string &name, int i, const std::string &what, int l, int r, int t, int b) {
  Copy *c = copy_of(name, i);
  int ch = channel_of(what);
  if (c && ch >= TURN)
    c->map[ch] = {true, l, r, t, b};
}
inline void rounds(const std::string &name, int i, int n) {
  Anim *a = find(name);
  Copy *c = copy_of(name, i);
  if (!a || !c || n <= 0)
    return;
  uint32_t total = (uint32_t) a->total_ms(), into = (millis() - (c->own ? c->placed : a->t0)) % total;
  c->rounds = n;
  c->rounds_end = millis() + (total - into) + (uint32_t) (n - 1) * total;
}

// ---------------------------------------------------------------- the model

// Keeps v to lo..hi as the edge says, turning the velocity round on a bounce. size is the copy's width or height.
inline void keep(float &v, float &vel, float lo, float hi, float size, int edge) {
  if (edge == 2) {  // bounce
    if (v < lo) {
      v = lo;
      vel = std::fabs(vel);
    } else if (v + size > hi) {
      v = hi - size;
      vel = -std::fabs(vel);
    }
  } else if (edge == 1) {  // out across one edge, back in at the other
    if (vel > 0 && v > hi)
      v = lo - size;
    else if (vel < 0 && v + size < lo)
      v = hi;
  } else {  // round the screen
    if (v > hi)
      v -= hi - lo + size;
    else if (v + size < lo)
      v += hi - lo + size;
  }
}

// The wave's value now
inline float wave_value(Wave &v) {
  uint32_t now = millis();
  for (int guard = 0; guard < 8 && now - v.t0 >= v.ms; guard++) {
    if (v.step == 0 || v.step == 2)
      v.cur = v.b;
    uint32_t end = v.t0 + v.ms;
    start_leg(v, (v.step + 1) % 4);
    v.t0 = end;
  }
  if (v.step == 1 || v.step == 3)
    return v.cur;
  float t = v.ms ? (float) (now - v.t0) / (float) v.ms : 1;
  // eased at the end it slows at: low is where a fall ends and a rise starts
  bool at_lo = v.slow & 1, at_hi = v.slow & 2;
  int e = v.step == 0 ? (at_lo ? 1 : 0) | (at_hi ? 2 : 0) : (at_hi ? 1 : 0) | (at_lo ? 2 : 0);
  return v.a + (v.b - v.a) * eased(t, e);
}

// Moves every copy on by dt ms
inline void step(uint32_t dt) {
  uint32_t now = millis();
  float s = dt / 1000.0f;
  for (auto &ap : anims) {
    Anim &a = *ap;
    for (Copy &c : a.copies) {
      if (!c.used)
        continue;
      if (c.fading && now - c.fade_t0 >= (uint32_t) c.fade_ms) {
        c = Copy();
        continue;
      }
      if (c.rounds && !c.fading && now >= c.rounds_end)
        fade_out(a, c, -1);
      float speed = c.base[SPEED] / 100.0f * (c.wave[SPEED].on ? wave_value(c.wave[SPEED]) / 100.0f : 1);
      for (int ch = 0; ch < CHANNELS; ch++) {
        Glide &g = c.glide[ch];
        if (g.on) {
          float t = g.ms ? (float) (now - g.t0) / (float) g.ms : 1;
          c.base[ch] = g.from + (g.to - g.from) * eased(t, g.ease);
          if (t >= 1)
            g.on = false;
        }
      }
      float w = (float) a.src.w, h = (float) a.src.h;
      float x0 = c.area ? (float) c.ax0 : 0, x1 = c.area ? (float) c.ax1 : 320;
      float y0 = c.area ? (float) c.ay0 : 0, y1 = c.area ? (float) c.ay1 : 240;
      auto &wd = c.wander;
      if (wd.on) {
        if (now >= wd.until) {
          if (wd.resting || (wd.r0 == 0 && wd.r1 == 0)) {
            float sp = rnd(wd.s0, wd.s1), hd = (wd.heading + rnd(-wd.spread, wd.spread)) * 3.14159265f / 180;
            wd.vx = sp * cosf(hd);
            wd.vy = sp * sinf(hd);
            wd.resting = false;
            wd.until = now + (uint32_t) rnd(wd.l0, wd.l1);
          } else {
            wd.vx = wd.vy = 0;
            wd.resting = true;
            wd.until = now + (uint32_t) rnd(wd.r0, wd.r1);
          }
        }
        c.base[X] += wd.vx * speed * s;
        c.base[Y] += wd.vy * speed * s;
        keep(c.base[X], wd.vx, x0, x1, w, c.ex);
        keep(c.base[Y], wd.vy, y0, y1, h, c.ey);
      }
      if (c.drift[X] || c.drift[Y]) {
        c.base[X] += c.drift[X] * speed * s;
        c.base[Y] += c.drift[Y] * speed * s;
        keep(c.base[X], c.drift[X], x0, x1, w, c.ex);
        keep(c.base[Y], c.drift[Y], y0, y1, h, c.ey);
      }
      c.base[TURN] += c.drift[TURN] * s;
      if (c.face && c.face_mirror) {  // faces the way it goes
        float dx = c.base[X] - c.px;
        bool facing_right = c.facing < 90 || c.facing > 270;
        if (std::fabs(dx) > 0.01f)
          c.mirrored = (dx < 0) == facing_right;
      }
      c.px = c.base[X];
      c.py = c.base[Y];
    }
  }
  anims.erase(std::remove_if(anims.begin(), anims.end(),
                             [](auto &a) {
                               if (!a->cleared)
                                 return false;
                               for (Copy &c : a->copies)
                                 if (c.used)
                                   return false;
                               return true;
                             }),
              anims.end());
}

// Draws every copy over rgb, W x H of 0xRRGGBB
inline void draw(uint32_t *rgb, int W, int H) {
  uint32_t now = millis();
  for (auto &ap : anims) {
    Anim &a = *ap;
    for (Copy &c : a.copies) {
      if (!c.used)
        continue;
      float v[CHANNELS];
      for (int ch = 0; ch < CHANNELS; ch++)
        v[ch] = c.base[ch];
      for (int ch = 0; ch < CHANNELS; ch++)
        if (c.wave[ch].on && ch != SPEED)
          v[ch] = ch <= TURN ? v[ch] + wave_value(c.wave[ch]) : v[ch] * wave_value(c.wave[ch]) / 100;
      float mx = v[X] + a.src.w / 2.0f, my = v[Y] + a.src.h / 2.0f;
      for (int ch = TURN; ch < CHANNELS; ch++) {
        const Map &m = c.map[ch];
        if (!m.on || !c.area)
          continue;
        float fx = std::clamp((mx - c.ax0) / (float) (c.ax1 - c.ax0), 0.0f, 1.0f),
              fy = std::clamp((my - c.ay0) / (float) (c.ay1 - c.ay0), 0.0f, 1.0f);
        float along = m.l + (m.r - m.l) * fx, down = m.t + (m.b - m.t) * fy;
        v[ch] = ch == TURN ? v[ch] + along + down : v[ch] * along / 100 * down / 100;
      }
      float op = std::clamp(v[OPACITY], 0.0f, 100.0f) / 100;
      if (c.fade_in > 0 && now - c.placed < (uint32_t) c.fade_in)
        op *= (float) (now - c.placed) / (float) c.fade_in;
      if (c.fading && c.fade_ms > 0)
        op *= 1 - std::min(1.0f, (float) (now - c.fade_t0) / (float) c.fade_ms);
      if (op <= 0.01f)
        continue;
      // a frame of its own starts start frames in, the others go in step from when the animation came
      uint32_t ms = now - a.t0;
      if (c.own) {
        ms = now - c.placed;
        for (int k = 0; k < c.start % std::max(1, a.src.frames); k++)
          ms += (uint32_t) a.frame_ms(k);
      }
      int f = a.frame_at(ms);
      if (c.rounds && now >= c.rounds_end)
        f = a.src.frames - 1;
      const std::vector<uint32_t> &px = a.frame(f);
      float size = std::max(0.1f, v[SIZE] / 100), turn = v[TURN] * 3.14159265f / 180;
      float ct = cosf(turn), st = sinf(turn), hw = a.src.w / 2.0f, hh = a.src.h / 2.0f;
      bool fx = c.flip_x != c.mirrored, fy = c.flip_y;
      int r = (int) ceilf(std::hypot(hw, hh) * size) + 1;
      // each wrap of the screen
      for (int wrap = -1; wrap <= 1; wrap++)
        for (int y = (int) my - r; y <= (int) my + r; y++) {
          if (y < 0 || y >= H)
            continue;
          for (int x = (int) mx - r; x <= (int) mx + r; x++) {
            int sx0 = x + wrap * W;
            if (sx0 < 0 || sx0 >= W)
              continue;
            float dx = (x + 0.5f - mx) / size, dy = (y + 0.5f - my) / size;
            float u = dx * ct + dy * st + hw, w = -dx * st + dy * ct + hh;
            int ix = (int) floorf(u), iy = (int) floorf(w);
            if (ix < 0 || iy < 0 || ix >= a.src.w || iy >= a.src.h)
              continue;
            if (fx)
              ix = a.src.w - 1 - ix;
            if (fy)
              iy = a.src.h - 1 - iy;
            uint32_t p = px[(size_t) iy * a.src.w + ix];
            if (p >> 24 < 128)
              continue;
            uint32_t &d = rgb[(size_t) y * W + sx0];
            auto ch = [&](int sh) {
              float b = (float) (d >> sh & 255), t = (float) (p >> sh & 255);
              return (uint32_t) lroundf(b + (t - b) * op) << sh;
            };
            d = ch(16) | ch(8) | ch(0);
          }
        }
    }
  }
}

// Copies on the panel now, and how many animations they're of
inline int copies_shown(int *n_anims = nullptr) {
  int n = 0, k = 0;
  for (auto &a : anims) {
    int m = 0;
    for (Copy &c : a->copies)
      m += c.used;
    n += m;
    k += m > 0;
  }
  if (n_anims)
    *n_anims = k;
  return n;
}

// Everything off, for the next picture
inline void reset() {
  anims.clear();
  sources.clear();
}

}  // namespace flexit::panel::anim
