// Stands in for packages/code/panel/animation.h when met_sim builds the weather animations on a PC: the ESP's side
// of the link becomes a small model of what the panel does with an animation's copies, after plink_keys.h's
// K_ANIM_* (place, glide, drift, wander, face, flip, wave, map, rounds, fades), on a clock of its own. It also gives
// the little of ESPHome that met_live*.h use: millis(), the logs, the time and picture::has_ram().
#pragma once
#include <algorithm>
#include <array>
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
  Phase phase = IDLE;  // each animation is sent at once, and the encounters find the link free
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
  bool torus = false;  // a drift or wander moved it: on a screen edge, what's over one edge shows at the other
  Wave wave[CHANNELS];
  Map map[CHANNELS];
  int rounds = 0;
  uint32_t rounds_end = 0;
  float px = 0, py = 0;  // where it was a step ago, for face
  uint8_t depth = ANIM_NEAREST;  // K_ANIM_DEPTH's: behind the picture's parts nearer than it
};

enum State { WANT, SENT, FAILED };

// A slot on the panel, ANIM_SLOTS of them. It's sent at once here, or FAILED when its frame doesn't fit ANIM_POOL.
struct Anim {
  std::string name;  // "" = free slot
  Source src;
  uint32_t t0 = 0;
  bool cleared = false;
  int state = WANT;
  std::array<Copy, ANIM_PLACES> copies;
  size_t frame_bytes() const { return (size_t) src.w / 2 * src.h; }
  // The records' bytes: not modelled, they depend on how well pdeflate packs the frames. The shows budget for them
  // themselves, see met_live_sky.h's SKY_POOL.
  size_t data_bytes() const { return 0; }
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
inline Anim anims[ANIM_SLOTS];

inline Anim *find(const std::string &name) {
  for (auto &a : anims)
    if (!a.name.empty() && a.name == name && !a.cleared)
      return &a;
  return nullptr;
}

// Bytes of ANIM_POOL the animations on the panel take, with their copies' own frames, other than a's
inline size_t used_by_others(const Anim *a) {
  size_t n = 0;
  for (auto &o : anims) {
    if (&o == a || o.name.empty() || o.state != SENT)
      continue;
    n += o.frame_bytes() + o.data_bytes();
    for (auto &c : o.copies)
      n += c.used && c.own ? o.frame_bytes() : 0;
  }
  return n;
}

// The slot of name, taking a free one: null when all ANIM_SLOTS are taken. One that doesn't fit the pool next to
// the others is FAILED, as the panel's PIC_TOO_BIG would have it.
inline Anim *want(const std::string &name) {
  if (Anim *a = find(name))
    return a->state == FAILED ? nullptr : a;
  auto s = sources.find(name);
  if (s == sources.end())
    return nullptr;
  Anim *a = nullptr;
  for (auto &o : anims)
    if (o.name.empty()) {
      a = &o;
      break;
    }
  for (auto &o : anims)  // animation.h frees a cleared one's slot at once, the panel fades its copies on its own
    if (!a && o.cleared)
      a = &o;
  if (!a) {
    sim::log('W', "anim", "%s: all %d slots taken", name.c_str(), ANIM_SLOTS);
    return nullptr;
  }
  *a = Anim();
  a->name = name;
  a->src = s->second;
  a->t0 = millis();
  size_t need = a->frame_bytes() + a->data_bytes(), used = used_by_others(a);
  a->state = used + need <= ANIM_POOL ? SENT : FAILED;
  if (a->state == FAILED) {
    sim::log('W', "anim", "%s: %zu bytes don't fit next to the others' %zu of %d", name.c_str(), need, used,
             ANIM_POOL);
    return nullptr;
  }
  return a;
}
inline Copy *copy_of(const std::string &name, int i) {
  Anim *a = find(name);
  return a && i >= 0 && i < ANIM_PLACES && a->copies[i].used && !a->copies[i].fading ? &a->copies[i] : nullptr;
}

inline Anim *slot_for(const std::string &name) { return want(name); }

// Copies on the panel, each in one of its ANIM_PLACES. One fading out has let go of its place, as animation.h's
// remove() has it, the panel fades it on its own.
inline int places_used() {
  int n = 0;
  for (auto &a : anims)
    for (auto &c : a.copies)
      n += c.used && !c.fading;
  return n;
}

inline bool place(const std::string &name, int i, int x, int y, bool own, int start, int fade = -1,
                  uint8_t depth = ANIM_NEAREST, int = -1) {
  Anim *a = want(name);
  if (!a || i < 0 || i >= ANIM_PLACES)
    return false;
  Copy &c = a->copies[i];
  if (!c.used && places_used() >= ANIM_PLACES) {
    sim::log('W', "anim", "%s: all %d places taken", name.c_str(), ANIM_PLACES);
    return false;
  }
  c = Copy();
  c.used = true;
  c.base[X] = c.px = (float) x;
  c.base[Y] = c.py = (float) y;
  c.own = own;
  c.start = start;
  c.placed = millis();
  c.fade_in = fade >= 0 ? fade : a->src.fade_ms;
  c.depth = depth;
  return true;
}
// Copy i of name and its animation, nulls if it isn't shown
inline std::pair<Anim *, Copy *> copy_for(const std::string &name, int i) {
  Anim *a = find(name);
  if (a && i >= 0 && i < ANIM_PLACES && a->copies[i].used && !a->copies[i].fading)
    return {a, &a->copies[i]};
  return {};
}
inline void fade_out(Anim &a, Copy &c, int fade) {
  c.fading = true;
  c.fade_t0 = millis();
  c.fade_ms = fade >= 0 ? fade : a.src.fade_ms;
}
inline void remove(const std::string &name, int i, int fade = -1) {
  Anim *a = find(name);
  if (a && i >= 0 && i < ANIM_PLACES && a->copies[i].used && !a->copies[i].fading)
    fade_out(*a, a->copies[i], fade);
}
inline void depth(const std::string &name, int i, uint8_t d) {
  if (Copy *c = copy_for(name, i).second)
    c->depth = d;
}
// Copy i of from becomes copy i of to at x, y, at its depth, with nothing between them
inline void swap(const std::string &from, int i, const std::string &to, int x, int y, bool own, int start) {
  Copy *c = copy_for(from, i).second;
  uint8_t d = c ? c->depth : ANIM_NEAREST;
  if (c)
    *c = Copy();
  place(to, i, x, y, own, start, 0, d);
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
  if (ch <= Y)
    c->torus = false;  // the panel's glide can end partly off the screen, and that's all
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
inline void spin(const std::string &name, int i, float speed) {
  if (Copy *c = copy_of(name, i))
    c->drift[TURN] = speed;  // degrees a second, 0 stops it where it is
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

// Times a copy went out at one edge and came back at the other, by animation
inline std::map<std::string, int> crossed;

// Keeps v to lo..hi as the edge says, turning the velocity round on a bounce. size is the copy's width or height.
// True when it went out at one edge and comes back at the other.
inline bool keep(float &v, float &vel, float lo, float hi, float size, int edge) {
  if (edge == 2) {  // bounce
    if (v < lo) {
      v = lo;
      vel = std::fabs(vel);
    } else if (v + size > hi) {
      v = hi - size;
      vel = -std::fabs(vel);
    }
  } else if (edge == 1) {  // out across one edge, back in at the other
    if (vel > 0 && v > hi) {
      v = lo - size;
      return true;
    }
    if (vel < 0 && v + size < lo) {
      v = hi;
      return true;
    }
  } else {  // round the screen
    if (v > hi) {
      v -= hi - lo + size;
      return true;
    }
    if (v + size < lo) {
      v += hi - lo + size;
      return true;
    }
  }
  return false;
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
  for (Anim &a : anims) {
    if (a.name.empty())
      continue;
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
        c.torus = true;
        crossed[a.name] += keep(c.base[X], wd.vx, x0, x1, w, c.ex);
        keep(c.base[Y], wd.vy, y0, y1, h, c.ey);
      }
      if (c.drift[X] || c.drift[Y]) {
        c.base[X] += c.drift[X] * speed * s;
        c.base[Y] += c.drift[Y] * speed * s;
        c.torus = true;
        crossed[a.name] += keep(c.base[X], c.drift[X], x0, x1, w, c.ex);
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
  for (Anim &a : anims) {  // a cleared one lets go of its slot once its copies have faded
    if (a.name.empty() || !(a.cleared || a.state == FAILED))
      continue;
    bool any = false;
    for (Copy &c : a.copies)
      any |= c.used;
    if (!any)
      a = Anim();
  }
}

// Pixels drawn round the screen's edge by animation, the part of a copy past one edge showing at the other
inline std::map<std::string, int> wrapped;

// The picture's parts, K_PIC_PART's, and its palette index at each pixel, for draw(): a copy is behind a part
// nearer than it where the picture has one of the part's colours. Set by met_sim.
struct PicPart {
  int x0, y0, x1, y1;
  uint16_t colours;
  uint8_t depth;
};
inline std::vector<PicPart> pic_parts;
inline std::vector<uint8_t> pic_index;
inline bool hidden(int x, int y, int W, uint8_t depth) {
  if (pic_index.empty())
    return false;
  uint8_t i = pic_index[(size_t) y * W + x];
  for (const PicPart &p : pic_parts)
    if (p.depth > depth && x >= p.x0 && x < p.x1 && y >= p.y0 && y < p.y1 && (p.colours >> i & 1))
      return true;
  return false;
}

// Draws every copy over rgb, W x H of 0xRRGGBB
inline void draw(uint32_t *rgb, int W, int H) {
  uint32_t now = millis();
  for (Anim &a : anims) {
    if (a.name.empty())
      continue;
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
      // each wrap of the screen, once a drift or wander has it going round it
      int wraps = c.torus && (!c.area || c.ex == 0) ? 1 : 0;
      for (int wrap = -wraps; wrap <= wraps; wrap++)
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
            if (p >> 24 < 128 || hidden(sx0, y, W, c.depth))
              continue;
            if (wrap)
              wrapped[a.name]++;
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
    for (Copy &c : a.copies)
      m += c.used;
    n += m;
    k += m > 0;
  }
  if (n_anims)
    *n_anims = k;
  return n;
}

// Slots taken now, and the bytes of ANIM_POOL their frames take
inline int slots_used(size_t *bytes = nullptr) {
  int n = 0;
  for (auto &a : anims)
    n += !a.name.empty() && a.state == SENT;
  if (bytes)
    *bytes = used_by_others(nullptr);
  return n;
}

// Where each animation's copies were, a look a frame: how often, the sum of their middles' x, and how often they
// were within 16 px of the screen's left or right edge or past it
struct Where {
  int n = 0, edge = 0;
  double x = 0;
};
inline std::map<std::string, Where> where;
inline void look() {
  for (Anim &a : anims)
    for (Copy &c : a.copies)
      if (!a.name.empty() && c.used && !c.fading) {
        Where &w = where[a.name];
        float l = c.base[X], r = l + a.src.w;
        w.n++;
        w.x += (l + r) / 2;
        w.edge += l < 16 || r > 320 - 16;
      }
}

// Everything off, for the next picture
inline void reset() {
  for (Anim &a : anims)
    a = Anim();
  sources.clear();
  wrapped.clear();
  crossed.clear();
  where.clear();
}

}  // namespace flexit::panel::anim
