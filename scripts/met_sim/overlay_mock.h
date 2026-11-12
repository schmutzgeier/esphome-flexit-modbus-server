// Stands in for packages/code/panel/overlay.h when met_sim builds the weather animations on a PC: the ESP's side of
// the link becomes a small model of what the panel does with an animation's copies, after plink_keys.h's K_OVERLAY_*
// (place, glide, drift, wander, face, area, view, bind, tint) and the scripts the copies run, compiled by the real
// overlay_script.h and stepped here as the panel would, on a clock of its own. The interface is overlay.h's, so
// packages/code/met builds as it is: add(), field(), place(), remove(), clear(), act(), told(), glide() and the rest,
// copy_for(), find(), overlays, W, used_by_others(), emptied and marked. It also gives the little of ESPHome that
// met_live*.h use: millis(), the logs and the time.
//
// What the model does: channels (x, y, pan_x, pan_y, w, h, turn, phase, size, size_x, size_y, opacity, speed, light,
// level, pivot_x, pivot_y, depth) with glides, drifts, wander, face, area and its edges; the binds (:=, +=, *=, wave,
// map, frame, registers, row.shift, row.size, row.opacity, row.light, row.phase) worked out with the panel's
// expression steps each motion step; views (flip, repeat, mirror, blend mix/add/screen/multiply), shape and soft
// edges, tint, light and glow, fades, fields (linear, radial, cone, noise) and a phase round Source::cycles.
// What it leaves out or approximates: smooth (a pixel is the nearest one), the panel's quantisation of a field and
// of the opacities, the bytes of the records (a frame counts as its 4-bit pixels whatever it packs to), the ESP's
// making and sending (every animation is there at once, W stays IDLE) and Source::ram.
#pragma once
#include <strings.h>
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
#include "overlay_script.h"  // the scripts' compiler, plink.h and plink_keys.h, copied next to it by run.sh

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

namespace flexit::panel::overlay {

// As overlay.h's
struct Source {
  int w, h, frames, frame_ms, x, y, scale;
  std::function<uint32_t(int, int, int)> pixel;
  int vx = 0, vy = 0;
  std::function<void(bool)> on_shown = {};
  int fade_ms = 0;
  std::vector<uint16_t> delays = {};
  std::vector<uint32_t> colours = {};
  Clips clips = {};
  Program script = {};
  std::vector<uint32_t> glows = {};
  uint32_t shade = 0;
  std::string label = {};
  std::vector<uint32_t> cycles = {};  // 0xRRGGBB of its own colours, those a copy's phase takes round
  int field = -1, field_a = 0, field_b = 0;
  size_t ram = 0;
};
inline uint32_t frame_ms(const Source &s, int f) { return s.delays.empty() ? s.frame_ms : s.delays[f]; }
inline std::map<std::string, Source> sources;

inline constexpr int MAX_FRAME_MS = OVERLAY_DELAY_MAX * OVERLAY_TICK_MS;
inline void add(const std::string &name, Source s) {
  int ms = std::clamp(s.frame_ms, OVERLAY_FRAME_MIN_MS, MAX_FRAME_MS);
  if (ms != s.frame_ms) {
    ESP_LOGW("panel", "Animation %s: frame_ms %d is outside %d..%d, it plays at %d", name.c_str(), s.frame_ms,
             OVERLAY_FRAME_MIN_MS, MAX_FRAME_MS, ms);
  }
  s.frame_ms = ms;
  sources[name] = std::move(s);
}
inline Source field(int w, int h, int kind, int a, int b, std::vector<uint32_t> colours) {
  Source s{w, h, 1, MAX_FRAME_MS, 0, 0, 1, nullptr};
  s.colours = std::move(colours);
  s.field = kind;
  s.field_a = a;
  s.field_b = b;
  return s;
}

// The channels, OVERLAY_X..OVERLAY_DEPTH, by the names met_sim uses
inline constexpr int NCH = OVERLAY_CHANNELS;
enum : int { X = OVERLAY_X, Y = OVERLAY_Y, TURN = OVERLAY_TURN, SIZE = OVERLAY_SIZE, OPACITY = OVERLAY_OPACITY };
inline int channel_of(const std::string &s) { return text::index(CHANNELS, s); }
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
struct Bind {
  uint8_t what;
  std::vector<uint8_t> v;  // the value's bytes, as OVERLAY_DO_BIND has them
};

// A copy on the panel, in one of its places, with what overlay.h's Copy keeps on the ESP
struct Copy {
  // the ESP's
  bool used = false, placed = false;
  uint8_t place = 0;
  int16_t x = 0, y = 0;
  bool own = false;
  uint8_t depth = OVERLAY_NEAREST;
  int fade = -1;
  std::string doing;
  Program script;
  uint8_t tag = 0;
  struct Told {
    uint16_t key;
    uint32_t at;
    std::vector<uint8_t> m;
  };
  std::vector<Told> told;

  // the panel's
  float base[NCH] = {};  // what glides and drifts make each channel
  float val[NCH] = {};   // as shown: with the binds, as of the last motion step
  Glide glide[NCH];
  float drift[NCH] = {};  // per second: px, percent or colours, degrees on turn
  std::vector<Bind> binds;
  unsigned view = 0;   // OVERLAY_VIEW_*
  bool torus = false;  // a drift or wander moved it: on a screen edge, what's over one edge shows at the other
  int ax0 = 0, ay0 = 0, ax1 = PIC_MAX_W, ay1 = PIC_MAX_H, ex = 0, ey = 0;
  bool area = false;
  struct {
    bool on = false;
    float s0, s1;
    int heading, spread, l0, l1, r0, r1;
    float vx = 0, vy = 0;
    bool resting = true;
    uint32_t until = 0;
  } wander;
  bool face = false, face_mirror = false, face_level = false;
  int facing = 0, tilt = 180, rate = 0;
  bool mirrored = false;  // by face
  float px = 0, py = 0;   // where it was a step ago, for face
  bool tint = false;
  uint32_t tint_rgb = 0;
  std::vector<int> shape;  // runs of w, top, bottom
  int soft[4] = {};
  uint32_t seed = 0;
  // its frames, with one of its own: the clip, first to last, the frame it's on and how far into it, its rate and
  // the rounds of its clip so far
  int first = 0, last = 0, frame = 0, frame_rate = 100, rounds = 0;
  float frame_t = 0;
  bool bound_frame = false;
  bool next = false, wrapped = false;
  int nfirst = 0, nlast = 0, ncw = 0, nch = 0;
  // its script
  std::vector<uint8_t> code;
  bool running = false, ending = false;  // ending: its END's fade
  size_t pc = 0;
  int32_t reg[OVERLAY_OWN_REGS] = {};
  enum { NONE, MS, STEP, ROUNDS, CLIP } wait = NONE;
  uint32_t until = 0;
  int rounds_to = 0;
  uint32_t placed_at = 0, ms = 0;  // ms: since it was placed as of the last motion step
  int fade_in = 300;
  bool fading = false;
  uint32_t fade_t0 = 0;
  int fade_ms = 0;
};

enum State { WANT, SENT, FAILED };

// An animation's frames, kept as the model draws them, for the slot and for the copies still fading out
struct Anim {
  std::string name;
  Source src;
  uint32_t t0 = 0;
  std::map<int, std::vector<uint32_t>> frames;
  int fw() const { return src.w * src.scale; }
  int fh() const { return src.h * src.scale; }
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
  int total_ms() const {
    int t = 0;
    for (int f = 0; f < src.frames; f++)
      t += (int) frame_ms(src, f);
    return std::max(1, t);
  }
  int frame_at(uint32_t ms) const {
    int t = (int) (ms % (uint32_t) total_ms());
    for (int f = 0; f < src.frames; f++) {
      if (t < (int) frame_ms(src, f))
        return f;
      t -= (int) frame_ms(src, f);
    }
    return 0;
  }
};

// A slot on the panel, the index is the slot
struct Overlay {
  std::string name;  // "" = free slot
  std::shared_ptr<Anim> an;
  int state = WANT;
  std::vector<Copy> copies;
  size_t frame_bytes() const { return an ? (size_t) (an->src.w + 1) / 2 * an->src.h : 0; }
  // what the panel keeps of its frame: a field is its 4 bytes. The records' bytes aren't modelled, they depend on how
  // well pdeflate packs the frames, and the acts budget for them themselves.
  size_t panel_frame() const { return an ? (an->src.field >= 0 ? 4 : frame_bytes()) : 0; }
};
inline Overlay overlays[OVERLAY_SLOTS];

// A copy fading out after remove() or clear(), which the ESP has let go of and the panel hasn't yet
struct Ghost {
  std::shared_ptr<Anim> an;
  Copy c;
};
inline std::vector<Ghost> ghosts;

enum Phase { IDLE, COUNT, MAKE, SEND };
struct Work {
  Phase phase = IDLE;  // each animation is made and sent at once
};
inline Work W;

inline Overlay *find(const std::string &name) {
  for (auto &a : overlays)
    if (!a.name.empty() && a.name == name)
      return &a;
  return nullptr;
}

inline size_t pool_bytes(const Copy &c) {
  size_t n = c.script.bytes.empty() ? 0 : OVERLAY_SCRIPT_HEAD + 4 * c.script.regs + c.script.bytes.size();
  bool roams = c.script.roams;
  for (auto &t : c.told) {
    n += t.key == K_OVERLAY_BIND ? t.m.size() - OVERLAY_BIND_VALUE + 1 : 0;
    roams |= t.key == K_OVERLAY_WANDER || t.key == K_OVERLAY_FACE;
  }
  return n + (roams ? OVERLAY_ROAM : 0) + c.script.gives;
}
inline size_t used_by_others(const Overlay *a) {
  size_t n = 0;
  for (auto &o : overlays) {
    if (&o == a || o.name.empty() || o.state != SENT)
      continue;
    n += o.panel_frame();
    for (auto &c : o.copies)
      n += c.used ? (c.own ? o.panel_frame() : 0) + pool_bytes(c) : 0;
  }
  return n;
}
inline size_t room() { return OVERLAY_POOL - std::min<size_t>(OVERLAY_POOL, used_by_others(nullptr)); }

inline int bound() {
  int n = 0;
  for (auto &a : overlays)
    for (auto &c : a.copies) {
      n += c.used ? c.script.binds + c.script.gives_binds : 0;
      for (auto &t : c.told)
        n += t.key == K_OVERLAY_BIND;
    }
  return n;
}

inline int32_t shared[OVERLAY_REGS - OVERLAY_OWN_REGS];  // the registers all scripts share
inline uint8_t tags;
inline uint32_t placed_so_far;  // copies placed, for their seeds

// Called with the animation when its script ended the last copy it had, and with a copy's marks
inline std::function<void(const std::string &)> emptied;
inline std::function<void(const std::string &, int, int)> marked;

// A free place on the panel, -1 with a warning about name's copy if all are taken
inline int free_place(const std::string &name) {
  for (int p = 0; p < OVERLAY_PLACES; p++) {
    bool taken = false;
    for (auto &a : overlays)
      for (auto &c : a.copies)
        taken |= c.used && c.place == p;
    if (!taken)
      return p;
  }
  ESP_LOGW("panel", "Animation %s: the panel shows %d copies at most", name.c_str(), OVERLAY_PLACES);
  return -1;
}
inline Copy *copy_of(Overlay &a, int i) {
  if ((int) a.copies.size() <= i)
    a.copies.resize((size_t) i + 1);
  Copy &c = a.copies[(size_t) i];
  if (!c.used) {
    int p = free_place(a.name);
    if (p < 0)
      return nullptr;
    c = Copy();
    c.used = true;
    c.place = (uint8_t) p;
  }
  return &c;
}
inline const Source *source(const std::string &name) {
  auto it = sources.find(name);
  if (it != sources.end())
    return &it->second;
  ESP_LOGW("panel", "No animation called '%s'", name.c_str());
  return nullptr;
}

// Copy c of a fading out to go, over ms
inline void fade_ghost(const std::shared_ptr<Anim> &an, const Copy &c, int ms) {
  if (!c.placed || ms < 0)
    return;
  Ghost g{an, c};
  g.c.fading = true;
  g.c.running = false;
  g.c.fade_t0 = millis();
  g.c.fade_ms = ms;
  ghosts.push_back(std::move(g));
}

// The slot for name, taken if it hasn't one. nullptr if there's no such animation, or no slot or place left for it,
// or it doesn't fit next to the others in OVERLAY_POOL, as the panel's PIC_TOO_BIG has it.
inline Overlay *want(const std::string &name) {
  if (Overlay *a = find(name))
    return a;
  const Source *s = source(name);
  if (!s)
    return nullptr;
  // as overlay.h's start(): the source itself on the screen, whatever its copies' views do after
  if (s->x < 0 || s->y < 0 || s->x + s->w * s->scale > PIC_MAX_W || s->y + s->h * s->scale > PIC_MAX_H) {
    ESP_LOGW("panel", "Animation %s: doesn't fit on the screen at %d,%d", name.c_str(), s->x, s->y);
    return nullptr;
  }
  Overlay *a = nullptr;
  for (auto &o : overlays)
    if (o.name.empty() && !a)
      a = &o;
  if (!a) {
    ESP_LOGW("panel", "No room for animation %s: the panel holds %d at a time", name.c_str(), OVERLAY_SLOTS);
    return nullptr;
  }
  if (free_place(name) < 0)
    return nullptr;
  *a = Overlay();
  a->name = name;
  a->an = std::make_shared<Anim>();
  a->an->name = name;
  a->an->src = *s;
  a->an->t0 = millis();
  size_t need = a->panel_frame(), used = used_by_others(a);
  if (used + need > OVERLAY_POOL) {
    ESP_LOGW("panel", "Animation %s: %zu bytes don't fit next to the others' %zu of %d", name.c_str(), need, used,
             OVERLAY_POOL);
    *a = Overlay();
    return nullptr;
  }
  a->state = SENT;
  if (s->on_shown)
    s->on_shown(true);
  return a;
}

inline void free_slot(Overlay &a) {
  for (Copy &c : a.copies)
    if (c.used)
      fade_ghost(a.an, c, a.an->src.fade_ms);
  if (a.an && a.an->src.on_shown)
    a.an->src.on_shown(false);
  a = Overlay();
}
inline void clear(const std::string &name) {
  if (name.empty() || name == "None") {
    for (auto &a : overlays)
      if (!a.name.empty())
        free_slot(a);
  } else if (Overlay *a = find(name)) {
    free_slot(*a);
  }
}

inline uint8_t place_of(const std::string &name, int i) {
  for (auto &a : overlays)
    if (a.name == name && i >= 0 && i < (int) a.copies.size() && a.copies[(size_t) i].used)
      return a.copies[(size_t) i].place;
  return OVERLAY_NOBODY;
}
// The copy at place p, null if none is there
inline std::pair<Overlay *, Copy *> at_place(uint8_t p) {
  for (auto &a : overlays)
    for (auto &c : a.copies)
      if (!a.name.empty() && c.used && c.place == p)
        return {&a, &c};
  return {};
}
inline std::pair<Overlay *, Copy *> copy_for(const std::string &name, int i, const char *what = nullptr) {
  Overlay *a = find(name);
  if (a && i >= 0 && i < (int) a->copies.size() && a->copies[(size_t) i].used)
    return {a, &a->copies[(size_t) i]};
  if (what) {
    ESP_LOGW("panel", "panel_%s_animation: no copy %d of animation '%s' shown", what, i, name.c_str());
  }
  return {};
}
inline int fade_of(const Overlay &a, const Copy &c) { return c.fade >= 0 ? c.fade : a.an->src.fade_ms; }

// ---------------------------------------------------------------- what the panel works out

inline float shown(const Copy &c, int ch) { return c.val[ch]; }
inline int box_w(const Copy &c) { return (int) lroundf(c.val[OVERLAY_W]); }
inline int box_h(const Copy &c) { return (int) lroundf(c.val[OVERLAY_H]); }

inline int frame_of(const Overlay &a, const Copy &c) { return c.own ? c.frame : a.an->frame_at(millis() - a.an->t0); }
inline int rounds_of(const Overlay &a, const Copy &c) {
  return c.own ? c.rounds : (int) ((millis() - a.an->t0) / (uint32_t) a.an->total_ms());
}

inline uint32_t hash32(uint32_t a, uint32_t b = 0, uint32_t c = 0) {
  uint32_t h = a * 0x9E3779B1u ^ b * 0x85EBCA77u ^ c * 0xC2B2AE3Du;
  h ^= h >> 15;
  h *= 0x2C1B3C6Du;
  h ^= h >> 12;
  h *= 0x297A2D39u;
  h ^= h >> 15;
  return h;
}
inline float hash01(uint32_t a, uint32_t b = 0, uint32_t c = 0) {
  return (float) (hash32(a, b, c) & 0xFFFFFF) / 16777216.0f;
}

inline int row_now;  // the row of a row bind being worked out

// What a GET reads of copy c of a: the channels as shown, then the rest
inline int32_t get(Overlay &a, Copy &c, int what) {
  if (what < NCH) {
    float v = c.val[what];
    if (what == OVERLAY_TURN)
      return ((int32_t) lroundf(v) % 360 + 360) % 360;
    return (int32_t) lroundf(v);
  }
  switch (what) {
    case OVERLAY_GET_FRAME:
      return frame_of(a, c);
    case OVERLAY_GET_FRAMES:
      return a.an->src.frames;
    case OVERLAY_GET_ROUNDS:
      return rounds_of(a, c);
    case OVERLAY_GET_MS:
      return (int32_t) c.ms;
    case OVERLAY_GET_FRAME_W:
      return a.an->fw();
    case OVERLAY_GET_FRAME_H:
      return a.an->fh();
    case OVERLAY_GET_HERE:
      return 1;
    case OVERLAY_GET_AT_X:
    case OVERLAY_GET_AT_Y: {
      bool xs = what == OVERLAY_GET_AT_X;
      float p = xs ? c.val[OVERLAY_X] + (float) box_w(c) * c.val[OVERLAY_PIVOT_X] / 1000
                   : c.val[OVERLAY_Y] + (float) box_h(c) * c.val[OVERLAY_PIVOT_Y] / 1000;
      float lo = (float) (xs ? c.ax0 : c.ay0), hi = (float) (xs ? c.ax1 : c.ay1);
      return (int32_t) std::clamp<float>(lroundf((p - lo) * OVERLAY_AT_MAX / std::max(1.0f, hi - lo)), 0,
                                         OVERLAY_AT_MAX);
    }
    case OVERLAY_GET_SEED:
      return (int32_t) (c.seed % (OVERLAY_SEED_MAX + 1));
    case OVERLAY_GET_ROW:
      return row_now;
  }
  return 0;
}
inline void pivot_of(const Copy &c, float &x, float &y) {
  x = c.val[OVERLAY_X] + (float) box_w(c) * c.val[OVERLAY_PIVOT_X] / 1000;
  y = c.val[OVERLAY_Y] + (float) box_h(c) * c.val[OVERLAY_PIVOT_Y] / 1000;
}

// wave(): between low and high as t goes by, the panel's rise, holds and fall, with jitter and the random ways
inline int32_t wave_of(int32_t t, int32_t lo, int32_t hi, int32_t rise, int32_t fall, int32_t hold_hi, int32_t hold_lo,
                       int32_t jitter, int32_t how, uint32_t seed) {
  if (hi <= lo)
    return lo;
  rise = std::max(rise, 0), fall = std::max(fall, 0), hold_hi = std::max(hold_hi, 0), hold_lo = std::max(hold_lo, 0);
  bool at_lo = how & OVERLAY_WAVE_SLOW_LOW, at_hi = how & OVERLAY_WAVE_SLOW_HIGH, random = how & OVERLAY_WAVE_RANDOM;
  int64_t period = (int64_t) rise + fall + hold_hi + hold_lo;
  if (period <= 0)
    return lo;
  double now = std::max(t, 0);
  double cur = lo, time = 0;
  uint32_t cycle = 0;
  if (!jitter && !random) {  // the same each time round
    cycle = (uint32_t) ((int64_t) now / period);
    time = (double) cycle * (double) period;
  }
  for (int rounds_done = 0; rounds_done < 4000; rounds_done++, cycle++)
    for (int step = 0; step < 4; step++) {
      float k = 1 + (float) jitter / 100 * (2 * hash01(seed, cycle, (uint32_t) step) - 1);
      double ms, from = cur, to = cur;
      if (step == 0 || step == 2) {
        to = step == 0 ? hi : lo;
        if (random)
          to = step == 0 ? cur + (hi - cur) * hash01(seed, cycle, 10) : lo + (cur - lo) * hash01(seed, cycle, 11);
        double span = (double) (hi - lo);
        ms = (step == 0 ? rise : fall) * std::fabs(to - cur) / span * k;
      } else {
        ms = (step == 1 ? hold_hi : hold_lo) * k;
      }
      if (now < time + ms) {
        if (step == 1 || step == 3)
          return (int32_t) lround(cur);
        float u = ms > 0 ? (float) ((now - time) / ms) : 1;
        int e = step == 0 ? (at_lo ? 1 : 0) | (at_hi ? 2 : 0) : (at_hi ? 1 : 0) | (at_lo ? 2 : 0);
        return (int32_t) lround(from + (to - from) * eased(u, e));
      }
      time += ms;
      cur = to;
    }
  return (int32_t) lround(cur);
}

// noise(): 0..n, smoothly on to another at random each period of t
inline int32_t noise_of(int32_t t, int32_t period, int32_t n, uint32_t seed) {
  if (period <= 0)
    return 0;
  int32_t i = (t >= 0 ? t : 0) / period;
  float f = (float) ((t >= 0 ? t : 0) % period) / (float) period, s = f * f * (3 - 2 * f);
  float a = hash01(seed, (uint32_t) i, 77), b = hash01(seed, (uint32_t) i + 1, 77);
  return (int32_t) lroundf((a + (b - a) * s) * (float) n);
}

// A value or an expression in b from pos, worked out for copy c of a. Ops are OVERLAY_OP_*, taking what the ones
// before them left; a bare value is one of them.
inline int32_t value(Overlay &a, Copy &c, const std::vector<uint8_t> &b, size_t &pos) {
  std::vector<int32_t> st;
  auto byte = [&]() -> int { return pos < b.size() ? b[pos++] : 0; };
  auto two = [&]() {
    int lo = byte();
    return (int32_t) (int16_t) (uint16_t) (lo | byte() << 8);
  };
  auto pop = [&]() -> int32_t {
    if (st.empty())
      return 0;
    int32_t v = st.back();
    st.pop_back();
    return v;
  };
  auto reg = [&](int r) -> int32_t & {
    static int32_t none;
    return r < OVERLAY_OWN_REGS ? c.reg[r] : r < OVERLAY_REGS ? shared[r - OVERLAY_OWN_REGS] : (none = 0);
  };
  auto place = [&]() -> std::pair<Overlay *, Copy *> {
    int p = byte();
    return p == OVERLAY_SELF ? std::pair<Overlay *, Copy *>{&a, &c} : at_place((uint8_t) p);
  };
  auto wrap32 = [](int64_t v) { return (int32_t) (uint32_t) (uint64_t) v; };
  auto op = [&](int o) {
    if (o == OVERLAY_V16) {
      st.push_back(two());
    } else if (o == OVERLAY_V32) {
      uint32_t v = 0;
      for (int k = 0; k < 4; k++)
        v |= (uint32_t) byte() << 8 * k;
      st.push_back((int32_t) v);
    } else if (o == OVERLAY_V8) {
      st.push_back((int32_t) (int8_t) byte());
    } else if (o >= OVERLAY_VREG && o < OVERLAY_VREG + OVERLAY_REGS) {
      st.push_back(reg(o - OVERLAY_VREG));
    } else if (o == OVERLAY_OP_OWN) {
      st.push_back(get(a, c, byte()));
    } else if (o == OVERLAY_OP_GET) {
      int what = byte();
      auto [pa, pc] = place();
      st.push_back(pc ? get(*pa, *pc, what) : 0);
    } else if (o == OVERLAY_OP_DIST || o == OVERLAY_OP_ANGLE) {
      auto [a1, c1] = place();
      auto [a2, c2] = place();
      if (!c1 || !c2) {
        st.push_back(0);
        return;
      }
      float x1, y1, x2, y2;
      pivot_of(*c1, x1, y1);
      pivot_of(*c2, x2, y2);
      if (o == OVERLAY_OP_DIST)
        st.push_back((int32_t) lroundf(std::hypot(x2 - x1, y2 - y1)));
      else
        st.push_back(((int32_t) lroundf(atan2f(y2 - y1, x2 - x1) * 180 / 3.14159265f) + 360) % 360);
    } else if (o == OVERLAY_OP_PICK || o == OVERLAY_OP_CURVE) {
      int32_t n_ = o == OVERLAY_OP_CURVE ? pop() : 0, k = pop();
      int n = byte();
      size_t at = pos;
      pos += 2 * (size_t) n;
      auto item = [&](int i) {
        return (int32_t) (int16_t) (uint16_t) (b[at + 2 * (size_t) i] | b[at + 2 * (size_t) i + 1] << 8);
      };
      if (!n) {
        st.push_back(0);
      } else if (o == OVERLAY_OP_PICK) {
        st.push_back(item(std::clamp<int32_t>(k, 0, n - 1)));
      } else {  // k is the value and n_ its end, along the list
        float u = n_ > 0 ? std::clamp((float) k / (float) n_, 0.0f, 1.0f) * (float) (n - 1) : 0;
        int i = std::min((int) u, n - 1), j = std::min(i + 1, n - 1);
        st.push_back((int32_t) lroundf((float) item(i) + (float) (item(j) - item(i)) * (u - (float) i)));
      }
    } else {
      switch (o) {
        case OVERLAY_OP_NEG:
        case OVERLAY_OP_ABS:
        case OVERLAY_OP_NOT: {
          int32_t v = pop();
          st.push_back(o == OVERLAY_OP_NEG   ? wrap32(-(int64_t) v)
                       : o == OVERLAY_OP_ABS ? (v < 0 ? wrap32(-(int64_t) v) : v)
                                             : !v);
          break;
        }
        case OVERLAY_OP_IF: {
          int32_t y = pop(), x = pop(), cond = pop();
          st.push_back(cond ? x : y);
          break;
        }
        case OVERLAY_OP_SCALE: {
          int32_t z = pop(), y = pop(), x = pop();
          st.push_back(z ? wrap32((int64_t) x * y / z) : 0);
          break;
        }
        case OVERLAY_OP_NOISE: {
          int32_t n = pop(), period = pop(), t = pop();
          st.push_back(noise_of(t, period, n, c.seed));
          break;
        }
        case OVERLAY_OP_EASE: {
          int32_t how = pop(), n = pop(), v = pop();
          st.push_back(n ? (int32_t) lroundf((float) n * eased((float) v / (float) n, how & 3)) : 0);
          break;
        }
        case OVERLAY_OP_WAVE: {
          int32_t how = pop(), jitter = pop(), hl = pop(), hh = pop(), fall = pop(), rise = pop(), hi = pop(),
                  lo = pop(), t = pop();
          st.push_back(wave_of(t, lo, hi, rise, fall, hh, hl, jitter, how, c.seed));
          break;
        }
        default: {  // two numbers
          int32_t y = pop(), x = pop();
          int64_t r = 0;
          switch (o) {
            case OVERLAY_OP_ADD:
              r = (int64_t) (uint32_t) x + (uint32_t) y;
              break;
            case OVERLAY_OP_SUB:
              r = (int64_t) (uint32_t) x - (uint32_t) y;
              break;
            case OVERLAY_OP_MUL:
              r = (int64_t) (uint32_t) x * (uint32_t) y;
              break;
            case OVERLAY_OP_DIV:
              r = y ? (int64_t) x / y : 0;
              break;
            case OVERLAY_OP_MOD:
              r = y ? (int64_t) x % y : 0;
              break;
            case OVERLAY_OP_MIN:
              r = std::min(x, y);
              break;
            case OVERLAY_OP_MAX:
              r = std::max(x, y);
              break;
            case OVERLAY_OP_LT:
              r = x < y;
              break;
            case OVERLAY_OP_LE:
              r = x <= y;
              break;
            case OVERLAY_OP_EQ:
              r = x == y;
              break;
            case OVERLAY_OP_NE:
              r = x != y;
              break;
            case OVERLAY_OP_GE:
              r = x >= y;
              break;
            case OVERLAY_OP_GT:
              r = x > y;
              break;
            case OVERLAY_OP_AND:
              r = x && y;
              break;
            case OVERLAY_OP_OR:
              r = x || y;
              break;
            case OVERLAY_OP_WRAP:
              r = y ? ((int64_t) x % y + y) % y : 0;
              break;
            case OVERLAY_OP_SIN:
              r = lroundf((float) y * sinf((float) x * 3.14159265f / 180));
              break;
            case OVERLAY_OP_COS:
              r = lroundf((float) y * cosf((float) x * 3.14159265f / 180));
              break;
            case OVERLAY_OP_ATAN:
              r = ((int32_t) lroundf(atan2f((float) y, (float) x) * 180 / 3.14159265f) + 360) % 360;
              break;
            case OVERLAY_OP_HYPOT:
              r = lroundf(std::hypot((float) x, (float) y));
              break;
            case OVERLAY_OP_RAND: {
              int32_t lo = std::min(x, y), hi = std::max(x, y);
              r = std::uniform_int_distribution<int32_t>(lo, hi)(rng);
              break;
            }
            case OVERLAY_OP_HASH:
              r = y > 0 ? (int64_t) (hash32((uint32_t) x, c.seed, 5) % (uint32_t) y) : 0;
              break;
            default:
              sim::log('W', "anim", "the mock doesn't know expression op %d", o);
          }
          st.push_back(wrap32(r));
        }
      }
    }
  };
  int t = byte();
  if (t == OVERLAY_VEXPR) {
    for (int guard = 0; pos < b.size() && guard < 512; guard++) {
      int o = byte();
      if (o == OVERLAY_OP_END)
        break;
      op(o);
    }
  } else {
    op(t);
  }
  return st.empty() ? 0 : st.back();
}

// How many bytes the value at pos in b takes
inline size_t value_len(const std::vector<uint8_t> &b, size_t pos) {
  size_t at = pos;
  auto one = [&](int o) {
    if (o == OVERLAY_V16)
      at += 2;
    else if (o == OVERLAY_V32)
      at += 4;
    else if (o == OVERLAY_V8)
      at += 1;
    else if (o == OVERLAY_OP_GET)
      at += 2;
    else if (o == OVERLAY_OP_OWN)
      at += 1;
    else if (o == OVERLAY_OP_DIST || o == OVERLAY_OP_ANGLE)
      at += 2;
    else if ((o == OVERLAY_OP_PICK || o == OVERLAY_OP_CURVE) && at < b.size())
      at += 1 + 2 * (size_t) b[at];
  };
  if (at >= b.size())
    return 0;
  int t = b[at++];
  if (t == OVERLAY_VEXPR) {
    while (at < b.size()) {
      int o = b[at++];
      if (o == OVERLAY_OP_END)
        break;
      one(o);
    }
  } else {
    one(t);
  }
  return at - pos;
}

// ---- the messages, as the panel takes them

inline void glide_ch(Copy &c, int ch, float to, int ms, int ease) {
  if (ch < 0 || ch >= NCH)
    return;
  if (ch == OVERLAY_TURN) {  // the short way round
    float d = std::fmod(to - c.base[OVERLAY_TURN], 360.0f);
    if (d > 180)
      d -= 360;
    else if (d < -180)
      d += 360;
    to = c.base[OVERLAY_TURN] + d;
    c.face = false;
  }
  c.glide[ch] = {true, c.base[ch], to, millis(), (uint32_t) std::max(0, ms), ease};
  if (ch <= OVERLAY_Y) {
    c.torus = false;  // the panel's glide can end partly off the screen, and that's all
    c.wander.on = false;
  }
  if (ch < OVERLAY_SIZE)
    c.drift[ch] = 0;
  if (!ms) {
    c.base[ch] = to;
    c.glide[ch].on = false;
  }
}

inline void set_bind(Copy &c, unsigned what, std::vector<uint8_t> v) {
  c.binds.erase(std::remove_if(c.binds.begin(), c.binds.end(),
                               [&](const Bind &b) {
                                 return !((b.what ^ what) & ~OVERLAY_BIND_OP) &&
                                        ((what & OVERLAY_BIND_OP) == OVERLAY_BIND_FREE || b.what == what);
                               }),
                c.binds.end());
  if ((what & OVERLAY_BIND_OP) != OVERLAY_BIND_FREE)
    c.binds.push_back({(uint8_t) what, std::move(v)});
  c.bound_frame = false;
  for (auto &b : c.binds)
    if ((b.what & ~OVERLAY_BIND_OP) == OVERLAY_BIND_FRAME)
      c.bound_frame = true;
}

// Message key, laid out as in plink_keys.h, to copy c
inline void apply(Copy &c, uint16_t key, const std::vector<uint8_t> &m) {
  auto u16 = [&](int at) { return (int) plink_get16(&m[(size_t) at]); };
  auto i16 = [&](int at) { return (int) plink_i16(&m[(size_t) at]); };
  switch (key) {
    case K_OVERLAY_GLIDE:
      for (size_t at = OVERLAY_GLIDE_HEAD; at + OVERLAY_GLIDE_EACH <= m.size(); at += OVERLAY_GLIDE_EACH)
        glide_ch(c, m[at], (float) i16((int) at + 1), u16(OVERLAY_GLIDE_MS), m[OVERLAY_GLIDE_EASE]);
      break;
    case K_OVERLAY_DRIFT:
      for (size_t at = OVERLAY_DRIFT_HEAD; at + OVERLAY_GLIDE_EACH <= m.size(); at += OVERLAY_GLIDE_EACH) {
        int ch = m[at], v = i16((int) at + 1);
        if (ch >= OVERLAY_SIZE)
          continue;
        c.drift[ch] = ch == OVERLAY_TURN ? (float) v / 10 : (float) v / 100;
        c.glide[ch].on = false;
        if (ch <= OVERLAY_Y)
          c.wander.on = false;
        if (ch == OVERLAY_TURN)
          c.face = false;
      }
      break;
    case K_OVERLAY_AREA:
      c.area = true;
      c.ax0 = i16(OVERLAY_AREA_X0), c.ay0 = i16(OVERLAY_AREA_Y0), c.ax1 = i16(OVERLAY_AREA_X1),
      c.ay1 = i16(OVERLAY_AREA_Y1);
      c.ex = m[OVERLAY_AREA_EDGES], c.ey = m[OVERLAY_AREA_EDGES + 1];
      break;
    case K_OVERLAY_WANDER: {
      auto &w = c.wander;
      w = {};
      w.on = true;
      w.s0 = (float) u16(OVERLAY_WANDER_SPEED) / 10, w.s1 = (float) u16(OVERLAY_WANDER_SPEED + 2) / 10;
      w.heading = i16(OVERLAY_WANDER_HEADING), w.spread = i16(OVERLAY_WANDER_SPREAD);
      w.l0 = u16(OVERLAY_WANDER_LEG), w.l1 = u16(OVERLAY_WANDER_LEG + 2);
      w.r0 = u16(OVERLAY_WANDER_REST), w.r1 = u16(OVERLAY_WANDER_REST + 2);
      c.glide[OVERLAY_X].on = c.glide[OVERLAY_Y].on = false;
      c.drift[OVERLAY_X] = c.drift[OVERLAY_Y] = 0;
      break;
    }
    case K_OVERLAY_FACE:
      c.face = true;
      c.face_mirror = m[OVERLAY_FACE_FLAGS] & OVERLAY_FACE_MIRROR;
      c.face_level = m[OVERLAY_FACE_FLAGS] & OVERLAY_FACE_LEVEL;
      c.facing = u16(OVERLAY_FACE_FACING);
      c.tilt = u16(OVERLAY_FACE_TILT);
      c.rate = u16(OVERLAY_FACE_RATE);
      c.glide[OVERLAY_TURN].on = false;
      c.drift[OVERLAY_TURN] = 0;
      break;
    case K_OVERLAY_VIEW:
      c.view = (c.view & ~(unsigned) m[OVERLAY_VIEW_MASK]) | (m[OVERLAY_VIEW_FLAGS] & m[OVERLAY_VIEW_MASK]);
      break;
    case K_OVERLAY_BIND:
      if (m.size() > OVERLAY_BIND_WHAT)
        set_bind(c, m[OVERLAY_BIND_WHAT],
                 std::vector<uint8_t>(m.begin() + std::min<size_t>(OVERLAY_BIND_VALUE, m.size()), m.end()));
      break;
    case K_OVERLAY_TINT:
      c.tint = m[OVERLAY_TINT_ON];
      c.tint_rgb =
          (uint32_t) m[OVERLAY_TINT_RGB] << 16 | (uint32_t) m[OVERLAY_TINT_RGB + 1] << 8 | m[OVERLAY_TINT_RGB + 2];
      break;
    default:
      sim::log('W', "anim", "a script sent key 0x%X, which the mock doesn't take", key);
  }
}

// Sends m to copy c of a, now, and keeps it like overlay.h does, to count what the panel binds
inline void tell(Overlay &a, Copy &c, uint16_t key, std::vector<uint8_t> m) {
  m[OVERLAY_MSG_PLACE] = c.place;
  c.told.erase(std::remove_if(c.told.begin(), c.told.end(),
                              [&](const Copy::Told &t) { return t.key == key && msg::replaces(key, m, t.m); }),
               c.told.end());
  bool binds = key == K_OVERLAY_BIND && !msg::frees(m);
  if (binds && bound() >= OVERLAY_BINDS) {
    ESP_LOGW("panel", "Animation %s: the panel binds %d at a time", a.name.c_str(), OVERLAY_BINDS);
    return;
  }
  apply(c, key, m);
  if (key != K_OVERLAY_BIND || binds)
    c.told.push_back({key, millis(), std::move(m)});
}

inline bool tell_text(Overlay &a, Copy &c, const std::string &statement, std::string &error) {
  Program p = compile(statement, error, a.an->src.clips);
  if (error.empty() && !p.bytes.empty())
    error = "it can't be told, it's a script";
  if (!error.empty())
    return false;
  for (auto &t : p.told)
    tell(a, c, t.key, std::move(t.m));
  return true;
}

// ---- the scripts, stepped as the panel runs them

inline void set_clip(Copy &c, int first, int last, int w, int h, const Overlay &a) {
  int n = std::max(1, a.an->src.frames);
  c.first = ((first % n) + n) % n;
  c.last = ((last % n) + n) % n;
  if (c.last < c.first)
    c.last = c.first;
  if (w)
    c.base[OVERLAY_W] = (float) w;
  if (h)
    c.base[OVERLAY_H] = (float) h;
  c.frame = c.first;
  c.frame_t = 0;
}

inline void fade_out(Copy &c, int fade) {
  c.fading = true;
  c.ending = true;
  c.running = false;
  c.fade_t0 = millis();
  c.fade_ms = fade;
}

// Runs c's script from where it is, up to OVERLAY_SCRIPT_STEPS steps, until it waits, stops or ends
inline void run(Overlay &a, Copy &c) {
  if (!c.running || c.fading)
    return;
  uint32_t now = millis();
  switch (c.wait) {
    case Copy::MS:
      if ((int32_t) (now - c.until) < 0)
        return;
      break;
    case Copy::ROUNDS:
      if (c.rounds < c.rounds_to)
        return;
      break;
    case Copy::CLIP:
      if (c.next)
        return;
      break;
    default:
      break;
  }
  c.wait = Copy::NONE;
  const std::vector<uint8_t> &b = c.code;
  auto byte = [&]() -> int { return c.pc < b.size() ? b[c.pc++] : 0; };
  auto two = [&]() {
    int lo = byte();
    return (int) (uint16_t) (lo | byte() << 8);
  };
  auto val = [&]() -> int32_t { return value(a, c, b, c.pc); };
  auto reg = [&](int r) -> int32_t & {
    static int32_t none;
    return r < OVERLAY_OWN_REGS ? c.reg[r] : r < OVERLAY_REGS ? shared[r - OVERLAY_OWN_REGS] : (none = 0);
  };
  for (int steps = 0; steps < OVERLAY_SCRIPT_STEPS; steps++) {
    if (c.pc >= b.size()) {
      c.running = false;
      return;
    }
    size_t start = c.pc;
    int op = byte();
    switch (op) {
      case OVERLAY_DO_SET: {
        int r = byte();
        int32_t v = val();
        reg(r) = v;
        break;
      }
      case OVERLAY_DO_GOTO:
        c.pc = (size_t) two();
        break;
      case OVERLAY_DO_IF: {
        int32_t v = val();
        size_t to = (size_t) two();
        if (v)
          c.pc = to;
        break;
      }
      case OVERLAY_DO_WAIT: {
        int32_t ms = val();
        if (ms > 0) {
          c.until = now + (uint32_t) ms;
          c.wait = Copy::MS;
          return;
        }
        break;
      }
      case OVERLAY_DO_UNTIL: {
        if (!val()) {
          c.pc = start;  // worked out again at the next motion step
          c.wait = Copy::STEP;
          return;
        }
        break;
      }
      case OVERLAY_DO_ROUNDS: {
        int32_t n = val();
        if (n > 0) {
          c.rounds_to = c.rounds + n;
          c.wait = Copy::ROUNDS;
          return;
        }
        break;
      }
      case OVERLAY_DO_CLIP: {
        int32_t first = val(), last = val();
        int mode = byte(), w = two(), h = two();
        if (mode == OVERLAY_CLIP_NOW || c.wrapped) {
          set_clip(c, first, last, w, h, a);
        } else {
          c.next = true;
          c.nfirst = first, c.nlast = last, c.ncw = w, c.nch = h;
          c.wait = Copy::CLIP;
          return;
        }
        break;
      }
      case OVERLAY_DO_FRAME: {
        int n = std::max(1, a.an->src.frames);
        c.frame = ((val() % n) + n) % n;
        c.frame_t = 0;
        break;
      }
      case OVERLAY_DO_RATE:
        c.frame_rate = std::clamp<int32_t>(val(), 0, OVERLAY_RATE_MAX);
        break;
      case OVERLAY_DO_GLIDE: {
        int head = byte(), n = head & 63, ease = head >> 6;
        int32_t ms = val();
        std::vector<std::pair<int, int32_t>> to;
        for (int k = 0; k < n; k++) {
          int ch = byte();
          to.push_back({ch, val()});
        }
        for (auto &[ch, v] : to)
          glide_ch(c, ch, (float) (int16_t) (uint16_t) v, (int) (uint16_t) ms, ease);
        break;
      }
      case OVERLAY_DO_SEND: {
        int p = byte();
        uint16_t key = (uint16_t) (K_OVERLAY_FRAMES + byte());
        int n = byte();
        std::vector<uint8_t> m(b.begin() + (long) c.pc, b.begin() + (long) std::min(b.size(), c.pc + (size_t) n));
        c.pc += (size_t) n;
        int k = byte();
        for (int f = 0; f < k; f++) {
          int at = byte();
          int32_t v = val();
          if (at + 1 < (int) m.size() && at >= 0)
            plink_put16(&m[(size_t) at], (uint16_t) v);
        }
        auto [ta, tc] = p == OVERLAY_SELF ? std::pair<Overlay *, Copy *>{&a, &c} : at_place((uint8_t) p);
        if (tc) {
          (void) ta;
          apply(*tc, key, m);
        }
        break;
      }
      case OVERLAY_DO_BIND: {
        int what = byte();
        std::vector<uint8_t> v;
        if ((what & OVERLAY_BIND_OP) != OVERLAY_BIND_FREE) {
          size_t len = value_len(b, c.pc);
          v.assign(b.begin() + (long) c.pc, b.begin() + (long) (c.pc + len));
          c.pc += len;
        }
        set_bind(c, (unsigned) what, std::move(v));
        break;
      }
      case OVERLAY_DO_MARK: {
        int tag = byte();
        sim::log('D', "anim", "%s: copy %d at mark %d", a.name.c_str(), (int) (&c - a.copies.data()), tag);
        if (marked)
          marked(a.name, (int) (&c - a.copies.data()), tag);
        break;
      }
      case OVERLAY_DO_END: {
        int32_t fade = val();
        c.tag = (uint8_t) byte();
        fade_out(c, (int) std::max<int32_t>(0, fade));
        return;
      }
      case OVERLAY_DO_SHAPE: {
        int n = byte();
        c.shape.clear();
        for (int k = 0; k < n; k++)
          c.shape.push_back((int16_t) two());
        break;
      }
      case OVERLAY_DO_SOFT:
        for (int k = 0; k < 4; k++)
          c.soft[k] = two();
        break;
      default:
        sim::log('W', "anim", "%s: a bad step %d at %zu", a.name.c_str(), op, c.pc - 1);
        c.running = false;
        return;
    }
  }
  c.wait = Copy::STEP;  // on at the next motion step
}

// Copy c of a runs p from now on, told what p sends before it does anything else, with the places it looks at, its
// END's tag and fade filled in
inline void set_script(Overlay &a, Copy &c, Program p) {
  for (auto &t : p.told)
    tell(a, c, t.key, std::move(t.m));
  p.told.clear();
  c.script = std::move(p);
  c.tag = 0;
  if (c.script.ends() && !(c.tag = ++tags))
    c.tag = ++tags;  // 0 is none
  if (c.script.binds && bound() > OVERLAY_BINDS) {
    ESP_LOGW("panel", "Animation %s: its script binds more than the panel's %d at a time", a.name.c_str(),
             OVERLAY_BINDS);
  }
  c.code = c.script.bytes;
  for (auto &q : c.script.patches) {
    if (q.at >= c.code.size())
      continue;
    if (q.kind == Program::PLACE)
      c.code[q.at] = place_of(q.name, q.copy);
    else if (q.kind == Program::TAG)
      c.code[q.at] = c.tag;
    else if ((size_t) q.at + 3 < c.code.size())
      plink_put32(&c.code[q.at], (uint32_t) fade_of(a, c));
  }
  c.pc = 0;
  c.wait = Copy::NONE;
  c.next = false;
  c.ending = false;
  c.fading = false;
  std::fill(std::begin(c.reg), std::end(c.reg), 0);
  c.running = !c.code.empty();
  run(a, c);  // at once
}

// ---- what overlay.h offers

inline Program started(int start) {
  std::string error;
  return start > 0 ? compile("frame " + std::to_string(start), error) : Program();
}

inline bool own_room(const Overlay &a, const Copy *c) {
  if (used_by_others(nullptr) + a.panel_frame() - (c && c->own ? a.panel_frame() : 0) <= OVERLAY_POOL)
    return true;
  ESP_LOGW("panel", "Animation %s: no room in the panel's %u bytes for a copy with a frame of its own", a.name.c_str(),
           (unsigned) OVERLAY_POOL);
  return false;
}

inline bool place(const std::string &name, int i, int x, int y, Program p, bool own = false, int fade = -1,
                  uint8_t depth = OVERLAY_NEAREST) {
  if (i < 0 || i >= OVERLAY_PLACES) {
    ESP_LOGW("panel", "Animation %s: copy %d isn't one of 0 to %d", name.c_str(), i, OVERLAY_PLACES - 1);
    return false;
  }
  Overlay *a = want(name);
  if (!a)
    return false;
  own |= p.frames;
  if (own && !own_room(*a, i < (int) a->copies.size() ? &a->copies[(size_t) i] : nullptr))
    return false;
  Copy *c = copy_of(*a, i);
  if (!c)
    return false;
  Copy n;
  n.used = n.placed = true;
  n.place = c->place;
  n.x = clamp16(x);
  n.y = clamp16(y);
  n.own = own;
  n.fade = std::min(65535, fade);
  n.depth = depth;
  const Source &s = a->an->src;
  float *v = n.base;
  v[OVERLAY_X] = n.px = (float) n.x;
  v[OVERLAY_Y] = n.py = (float) n.y;
  v[OVERLAY_W] = (float) (s.w * s.scale);
  v[OVERLAY_H] = (float) (s.h * s.scale);
  v[OVERLAY_SIZE] = v[OVERLAY_SIZE_X] = v[OVERLAY_SIZE_Y] = v[OVERLAY_OPACITY] = v[OVERLAY_SPEED] = v[OVERLAY_LIGHT] =
      v[OVERLAY_LEVEL] = 100;
  v[OVERLAY_PIVOT_X] = v[OVERLAY_PIVOT_Y] = 500;
  v[OVERLAY_DEPTH] = depth;
  std::copy(std::begin(n.base), std::end(n.base), std::begin(n.val));
  n.placed_at = millis();
  n.fade_in = fade >= 0 ? std::min(65535, fade) : s.fade_ms;
  n.seed = hash32(++placed_so_far, (uint32_t) s.w, (uint32_t) i);
  n.last = std::max(0, s.frames - 1);
  *c = std::move(n);
  set_script(*a, *c, std::move(p));
  return true;
}
inline bool place(const std::string &name, int i, int x, int y, bool own, int start, int fade = -1,
                  uint8_t depth = OVERLAY_NEAREST) {
  return place(name, i, x, y, own ? started(start) : Program(), own, fade, depth);
}

// panel_show_animation: shows it where its yaml puts it, copy 0 there with its drift and its script
inline void show(const std::string &name) {
  Overlay *a = want(name);
  if (!a)
    return;
  if (!a->copies.empty() && a->copies[0].used)
    return;
  const Source &s = a->an->src;
  std::string error;
  Program p = s.script;
  if (s.vx || s.vy) {
    Program d = compile("drift " + std::to_string(s.vx) + ", " + std::to_string(s.vy), error);
    p.told.insert(p.told.begin(), d.told.begin(), d.told.end());
  }
  place(name, 0, s.x, s.y, std::move(p), s.script.frames);
}
inline void switched_on(const std::string &name) {
  if (sources.count(name))
    show(name);
}

inline void remove(const std::string &name, int i, int fade = -1) {
  auto [a, c] = copy_for(name, i);
  if (!c)
    return;
  fade_ghost(a->an, *c, fade >= 0 ? std::min(65535, fade) : fade_of(*a, *c));
  *c = Copy();
}
inline void depth(const std::string &name, int i, uint8_t d) {
  auto [a, c] = copy_for(name, i);
  if (!c || c->depth == d)
    return;
  c->depth = d;
  c->base[OVERLAY_DEPTH] = d;
  c->glide[OVERLAY_DEPTH].on = false;
}

template<size_t N> int option(const char *const (&names)[N], const std::string &s, const char *what) {
  for (size_t k = 0; k < N; k++)
    if (strcasecmp(names[k], s.c_str()) == 0)
      return (int) k;
  std::string all;
  for (size_t k = 0; k < N; k++)
    all += (k ? ", " : "") + std::string(names[k]);
  ESP_LOGW("panel", "panel_%s_animation: '%s' isn't one of %s", what, s.c_str(), all.c_str());
  return -1;
}
inline Copy *act(const std::string &name, int i, const std::string &statement, const char *what) {
  auto [a, c] = copy_for(name, i, what);
  std::string error;
  if (!c || tell_text(*a, *c, statement, error))
    return c;
  ESP_LOGW("panel", "panel_%s_animation: %s", what, error.substr(error.rfind("line 1: ", 0) == 0 ? 8 : 0).c_str());
  return nullptr;
}
inline std::string num(int v) { return std::to_string(v); }

inline void glide(const std::string &name, int i, const std::string &what, int to, int ms, const std::string &ease) {
  int ch = option(CHANNELS, what, __func__);
  Copy *c = ch >= 0 && option(EASES, ease, __func__) >= 0
                ? act(name, i, "glide " + what + " " + num(to) + " over " + num(ms) + " ease " + ease, __func__)
                : nullptr;
  if (c && ch <= OVERLAY_Y)
    c->doing = ", moved";
}
inline void move(const std::string &name, int i, int x, int y, int ms, const std::string &ease) {
  Copy *c = option(EASES, ease, __func__) >= 0
                ? act(name, i, "move " + num(x) + ", " + num(y) + " over " + num(ms) + " ease " + ease, __func__)
                : nullptr;
  if (c)
    c->doing = ", moved to " + num(x) + "," + num(y);
}
inline void drift(const std::string &name, int i, float vx, float vy) {
  if (Copy *c = act(name, i, "drift " + decimal(vx) + ", " + decimal(vy), __func__))
    c->doing = vx || vy ? ", drifting" : "";
}
inline void tint(const std::string &name, int i, const std::string &colour) {
  std::string hex = colour.substr(!colour.empty() && colour[0] == '#');
  bool none = hex.empty() || strcasecmp(hex.c_str(), "none") == 0;
  if (!none && (hex.size() != 6 || !std::all_of(hex.begin(), hex.end(), [](char k) { return isxdigit(k); }))) {
    ESP_LOGW("panel", "panel_%s_animation: '%s' isn't a colour like FF77BB", __func__, colour.c_str());
    return;
  }
  act(name, i, none ? "tint none" : "tint 0x" + hex, __func__);
}
inline void area(const std::string &name, int i, int left, int top, int right, int bottom, const std::string &edge_x,
                 const std::string &edge_y) {
  if (option(EDGES, edge_x, __func__) >= 0 && option(EDGES, edge_y, __func__) >= 0)
    act(name, i,
        "area " + num(left) + ", " + num(top) + ", " + num(right) + ", " + num(bottom) + " " + edge_x + " " + edge_y,
        __func__);
}
inline void wander(const std::string &name, int i, float speed_min, float speed_max, int heading, int spread,
                   int leg_min, int leg_max, int rest_min, int rest_max) {
  if (Copy *c =
          act(name, i,
              "wander " + decimal(speed_min) + ", " + decimal(speed_max) + ", " + num(heading) + ", " + num(spread) +
                  ", " + num(leg_min) + ", " + num(leg_max) + ", " + num(rest_min) + ", " + num(rest_max),
              __func__))
    c->doing = ", wandering";
}
inline void face(const std::string &name, int i, int facing, bool mirror, int tilt, int rate, bool level) {
  act(name, i,
      "face " + num(facing) + (mirror ? " mirror" : "") + (level ? " level" : "") + " tilt " + num(tilt) + " rate " +
          num(rate),
      __func__);
}
inline void told(const std::string &name, int i, const std::string &statement) { act(name, i, statement, "tell"); }

inline bool run_script(Overlay &a, Copy &c, const std::string &text, const Consts &consts, const char *what) {
  std::string error;
  Program p = compile(text, error, a.an->src.clips, consts);
  if (error.empty() && p.frames && !c.own)
    error = "it plays frames, and copy " + std::to_string(&c - a.copies.data()) + " has no frame of its own";
  if (!error.empty()) {
    ESP_LOGW("panel", "panel_%s_animation: %s's script, %s", what, a.name.c_str(), error.c_str());
    return false;
  }
  set_script(a, c, std::move(p));
  return true;
}
inline bool script(const std::string &name, int i, const std::string &text, const Consts &consts = {}) {
  auto [a, c] = copy_for(name, i, __func__);
  return c && run_script(*a, *c, text, consts, __func__);
}
inline void rounds(const std::string &name, int i, int n) {
  if (auto [a, c] = copy_for(name, i, __func__); c)
    run_script(*a, *c, n > 0 ? "wait rounds " + std::to_string(clamp_u16(n)) + "; end" : "", {}, __func__);
}
inline void frame_rate(const std::string &, int) {}  // not modelled

// ---------------------------------------------------------------- the model

// Times a copy went out at one edge and came back at the other, by animation
inline std::map<std::string, int> crossed;

// Keeps v to lo..hi as the edge says, turning the velocity round on a bounce. size is the copy's width or height.
// True when it went out at one edge and comes back at the other.
inline bool keep(float &v, float &vel, float lo, float hi, float size, int edge) {
  if (edge == OVERLAY_EDGE_BOUNCE) {
    if (v < lo) {
      v = lo;
      vel = std::fabs(vel);
    } else if (v + size > hi) {
      v = hi - size;
      vel = -std::fabs(vel);
    }
  } else if (edge == OVERLAY_EDGE_CROSS) {  // out across one edge, back in at the other
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

// A copy with a frame of its own plays its clip on by dt ms
inline void play(Overlay &a, Copy &c, uint32_t dt) {
  c.wrapped = false;
  if (!c.own || a.an->src.frames <= 0 || c.bound_frame)
    return;
  c.frame_t += (float) dt * (float) c.frame_rate / 100;
  for (int guard = 0; guard < 64 && c.frame_rate > 0 && c.frame_t >= (float) frame_ms(a.an->src, c.frame); guard++) {
    c.frame_t -= (float) frame_ms(a.an->src, c.frame);
    if (c.frame < c.last && c.frame >= c.first) {
      c.frame++;
      continue;
    }
    c.rounds++;  // a round of the clip ended
    c.wrapped = true;
    if (c.next) {
      c.next = false;
      set_clip(c, c.nfirst, c.nlast, c.ncw, c.nch, a);
    } else {
      c.frame = c.first;
    }
  }
}

inline float norm180(float d) {
  d = std::fmod(d, 360.0f);
  return d > 180 ? d - 360 : d < -180 ? d + 360 : d;
}

// What each channel of c is now: where glides and drifts have it, set, times, plus what's bound; the registers first
inline void work_out(Overlay &a, Copy &c) {
  for (const Bind &b : c.binds) {
    unsigned to = b.what & ~OVERLAY_BIND_OP;
    if (to >= OVERLAY_BIND_REG) {
      size_t pos = 0;
      int r = (int) to - OVERLAY_BIND_REG;
      int32_t v = value(a, c, b.v, pos);
      if (r < OVERLAY_OWN_REGS)
        c.reg[r] = v;
      else if (r < OVERLAY_REGS)
        shared[r - OVERLAY_OWN_REGS] = v;
    }
  }
  float set[NCH], mul[NCH], plus[NCH];
  bool has_set[NCH] = {}, has_mul[NCH] = {}, has_plus[NCH] = {};
  for (const Bind &b : c.binds) {
    unsigned to = b.what & ~OVERLAY_BIND_OP, op = b.what & OVERLAY_BIND_OP;
    if (to >= OVERLAY_BIND_REG || (to >= OVERLAY_BIND_ROW_SHIFT && to < OVERLAY_BIND_TARGETS))
      continue;
    size_t pos = 0;
    int32_t v = value(a, c, b.v, pos);
    if (to == OVERLAY_BIND_FRAME) {
      int n = std::max(1, a.an->src.frames);
      c.frame = ((v % n) + n) % n;
      continue;
    }
    if (to >= (unsigned) NCH)
      continue;
    if (op == OVERLAY_BIND_SET)
      set[to] = (float) v / (float) fine_of((int) to), has_set[to] = true;
    else if (op == OVERLAY_BIND_MUL)
      mul[to] = (float) v / 1000, has_mul[to] = true;
    else
      plus[to] = (float) v / (float) fine_of((int) to), has_plus[to] = true;
  }
  for (int ch = 0; ch < NCH; ch++) {
    float v = has_set[ch] ? set[ch] : c.base[ch];
    if (has_mul[ch])
      v *= mul[ch];
    if (has_plus[ch])
      v += plus[ch];
    const msg::Range &r = msg::RANGES[ch];
    if (ch != OVERLAY_X && ch != OVERLAY_Y && ch != OVERLAY_TURN && ch != OVERLAY_PAN_X && ch != OVERLAY_PAN_Y &&
        ch != OVERLAY_PHASE)
      v = std::clamp(v, (float) r.lo, (float) r.hi);
    c.val[ch] = v;
  }
}

// Moves every copy on by dt ms (the panel's motion step is 50), and runs their scripts
inline void step(uint32_t dt) {
  uint32_t now = millis();
  float s = (float) dt / 1000.0f;
  std::vector<std::string> empty;  // animations a script's END left without a copy
  ghosts.erase(std::remove_if(ghosts.begin(), ghosts.end(),
                              [&](const Ghost &g) { return now - g.c.fade_t0 >= (uint32_t) g.c.fade_ms; }),
               ghosts.end());
  for (Overlay &a : overlays) {
    if (a.name.empty())
      continue;
    for (Copy &c : a.copies) {
      if (!c.used)
        continue;
      if (c.fading && now - c.fade_t0 >= (uint32_t) c.fade_ms) {  // the panel's GONE
        uint8_t tag = c.tag;
        (void) tag;
        c = Copy();
        if (emptied && std::none_of(a.copies.begin(), a.copies.end(), [](const Copy &o) { return o.used; }))
          empty.push_back(a.name);
        continue;
      }
      c.ms = now - c.placed_at;
      play(a, c, dt);
      run(a, c);
      float speed = c.val[OVERLAY_SPEED] / 100.0f;
      for (int ch = 0; ch < NCH; ch++) {
        Glide &g = c.glide[ch];
        if (g.on) {
          float t = g.ms ? (float) (now - g.t0) / (float) g.ms : 1;
          c.base[ch] = g.from + (g.to - g.from) * eased(t, g.ease);
          if (t >= 1)
            g.on = false;
        }
      }
      float w = (float) box_w(c), h = (float) box_h(c);
      float x0 = c.area ? (float) c.ax0 : 0, x1 = c.area ? (float) c.ax1 : PIC_MAX_W;
      float y0 = c.area ? (float) c.ay0 : 0, y1 = c.area ? (float) c.ay1 : PIC_MAX_H;
      auto &wd = c.wander;
      if (wd.on) {
        if (now >= wd.until) {
          if (wd.resting || (wd.r0 == 0 && wd.r1 == 0)) {
            float sp = rnd(wd.s0, wd.s1),
                  hd = ((float) wd.heading + rnd((float) -wd.spread, (float) wd.spread)) * 3.14159265f / 180;
            wd.vx = sp * cosf(hd);
            wd.vy = sp * sinf(hd);
            wd.resting = false;
            wd.until = now + (uint32_t) rnd((float) wd.l0, (float) wd.l1);
          } else {
            wd.vx = wd.vy = 0;
            wd.resting = true;
            wd.until = now + (uint32_t) rnd((float) wd.r0, (float) wd.r1);
          }
        }
        c.base[OVERLAY_X] += wd.vx * speed * s;
        c.base[OVERLAY_Y] += wd.vy * speed * s;
        c.torus = true;
        crossed[a.name] += keep(c.base[OVERLAY_X], wd.vx, x0, x1, w, c.ex);
        keep(c.base[OVERLAY_Y], wd.vy, y0, y1, h, c.ey);
      }
      for (int ch = 0; ch < OVERLAY_SIZE; ch++) {
        float d = c.drift[ch];
        if (!d)
          continue;
        if (ch == OVERLAY_TURN) {
          c.base[ch] += d * s;
        } else if (ch == OVERLAY_X) {
          c.base[ch] += d * speed * s;
          c.torus = true;
          crossed[a.name] += keep(c.base[ch], c.drift[ch], x0, x1, w, c.ex);
        } else if (ch == OVERLAY_Y) {
          c.base[ch] += d * speed * s;
          c.torus = true;
          keep(c.base[ch], c.drift[ch], y0, y1, h, c.ey);
        } else {
          c.base[ch] += d * speed * s;
          if (ch == OVERLAY_W || ch == OVERLAY_H)
            c.base[ch] = std::clamp(c.base[ch], 0.0f, (float) OVERLAY_BOX_MAX);
        }
      }
      // face: toward where it goes
      if (c.face) {
        float dx = c.base[OVERLAY_X] - c.px, dy = c.base[OVERLAY_Y] - c.py;
        bool facing_right = c.facing < 90 || c.facing > 270;
        if (c.face_mirror && std::fabs(dx) > 0.01f)
          c.mirrored = (dx < 0) == facing_right;
        float want_turn = c.base[OVERLAY_TURN];
        if (std::hypot(dx, dy) > 0.05f) {
          float heading = atan2f(dy, dx) * 180 / 3.14159265f;
          float eff = c.mirrored ? 180.0f - (float) c.facing : (float) c.facing;
          want_turn = std::clamp(norm180(heading - eff), (float) -c.tilt, (float) c.tilt);
        } else if (c.face_level) {
          want_turn = 0;
        }
        float d = norm180(want_turn - c.base[OVERLAY_TURN]);
        float stepmax = c.rate ? (float) c.rate * s : 360.0f;
        c.base[OVERLAY_TURN] += std::clamp(d, -stepmax, stepmax);
      }
      c.px = c.base[OVERLAY_X];
      c.py = c.base[OVERLAY_Y];
      work_out(a, c);
    }
  }
  for (Ghost &g : ghosts)
    (void) g;
  for (const std::string &n : empty)  // as the panel's GONE has it, which may clear it
    if (emptied)
      emptied(n);
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

inline uint32_t mix(uint32_t from, uint32_t to, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  uint32_t r = 0;
  for (int sh = 0; sh <= 16; sh += 8) {
    float f = (float) (from >> sh & 255), g = (float) (to >> sh & 255);
    r |= (uint32_t) lroundf(f + (g - f) * t) << sh;
  }
  return r;
}

// 0xRRGGBB p in the copy's tint and light, but for the colours that glow
inline uint32_t lit(const Anim &a, const Copy &c, uint32_t p, float light) {
  if (std::find(a.src.glows.begin(), a.src.glows.end(), p) != a.src.glows.end())
    return p;
  if (c.tint) {  // a shade of the tint, black through it to white as it's dark or light
    float l = (0.299f * (float) (p >> 16 & 255) + 0.587f * (float) (p >> 8 & 255) + 0.114f * (float) (p & 255)) / 255;
    p = l < 0.5f ? mix(0, c.tint_rgb, 2 * l) : mix(c.tint_rgb, 0xFFFFFF, 2 * l - 1);
  }
  if (light < 100)
    p = mix(a.src.shade, p, light / 100);
  else if (light > 100)
    p = mix(p, 0xFFFFFF, (light - 100) / (OVERLAY_LIGHT_MAX - 100));
  return p;
}

// Value noise 0..255 at x, y px of a field, layers of it, the first with bumps every grain px
inline float field_noise(uint32_t seed, float x, float y, int layers, float grain) {
  float sum = 0, amp = 1, norm = 0;
  for (int l = 0; l < layers; l++) {
    float g = std::max(1.0f, grain / (float) (1 << l)), fx = x / g, fy = y / g;
    int ix = (int) floorf(fx), iy = (int) floorf(fy);
    float u = fx - (float) ix, v = fy - (float) iy;
    u = u * u * (3 - 2 * u), v = v * v * (3 - 2 * v);
    auto h = [&](int i, int j) { return hash01(seed + (uint32_t) l, (uint32_t) (i & 63), (uint32_t) (j & 63)); };
    float n =
        (h(ix, iy) * (1 - u) + h(ix + 1, iy) * u) * (1 - v) + (h(ix, iy + 1) * (1 - u) + h(ix + 1, iy + 1) * u) * v;
    sum += n * amp;
    norm += amp;
    amp /= 2;
  }
  return sum / norm * 255;
}

// A field's colour at u, v of its box, 0xAARRGGBB
inline uint32_t field_at(const Anim &a, const Copy &c, float u, float v, float bw, float bh) {
  const Source &s = a.src;
  float k = 0;
  switch (s.field) {
    case OVERLAY_FIELD_LINEAR: {
      float t = std::clamp((s.field_a & OVERLAY_LINEAR_DOWN ? v / bh : u / bw), 0.0f, 1.0f);
      if (s.field_a & OVERLAY_LINEAR_MIRROR)
        t = t < 0.5f ? 2 * t : 2 - 2 * t;
      k = t * 255;
      break;
    }
    case OVERLAY_FIELD_RADIAL: {
      float d = std::hypot((u - bw / 2) / (bw / 2), (v - bh / 2) / (bh / 2));
      k = std::clamp(1 - d, 0.0f, 1.0f) * 255;
      break;
    }
    case OVERLAY_FIELD_CONE: {
      float apex = (float) s.field_a / 255, t = std::clamp(u / bw, 0.0f, 1.0f);
      float half = bh / 2 * (apex + (1 - apex) * t);
      k = half > 0 ? std::clamp(1 - std::fabs(v - bh / 2) / half, 0.0f, 1.0f) * 255 : 0;
      break;
    }
    case OVERLAY_FIELD_NOISE:
      k = field_noise(c.seed, u - c.val[OVERLAY_PAN_X], v - c.val[OVERLAY_PAN_Y], std::clamp(s.field_a, 1, 3),
                      (float) std::max(4, s.field_b));
      break;
  }
  k = std::clamp(k * c.val[OVERLAY_LEVEL] / 100, 0.0f, 255.0f);
  int n = std::max<int>(1, (int) s.colours.size());
  float pos = k / 255 * (float) (n - 1) + c.val[OVERLAY_PHASE];
  pos = std::fmod(pos, (float) n);
  if (pos < 0)
    pos += (float) n;
  int i = (int) pos, j = (i + 1) % n;
  float f = pos - (float) i;
  auto col = [&](int x) { return x < (int) s.colours.size() ? s.colours[(size_t) x] : 0u; };
  uint32_t p = col(i), q = col(j);
  uint32_t al = (uint32_t) lroundf((float) (p >> 24) + (float) ((int) (q >> 24) - (int) (p >> 24)) * f);
  return al << 24 | mix(p & 0xFFFFFF, q & 0xFFFFFF, f);
}

// Draws every copy and the ghosts over rgb, W x H of 0xRRGGBB, the ones at less depth first
inline void draw_copy(Anim &an, const Copy &c, uint32_t *rgb, int W, int H, int frame) {
  uint32_t now = millis();
  const Source &src = an.src;
  float v[NCH];
  std::copy(std::begin(c.val), std::end(c.val), std::begin(v));
  float op = std::clamp(v[OVERLAY_OPACITY], 0.0f, 100.0f) / 100;
  if (c.fade_in > 0 && now - c.placed_at < (uint32_t) c.fade_in)
    op *= (float) (now - c.placed_at) / (float) c.fade_in;
  if (c.fading && c.fade_ms > 0)
    op *= 1 - std::min(1.0f, (float) (now - c.fade_t0) / (float) c.fade_ms);
  else if (c.fading)
    return;
  if (op <= 0.003f)
    return;
  int bw = (int) lroundf(v[OVERLAY_W]), bh = (int) lroundf(v[OVERLAY_H]);
  if (bw <= 0 || bh <= 0)
    return;
  const std::vector<uint32_t> *px = src.field >= 0 ? nullptr : &an.frame(frame);
  int fw = an.fw(), fh = an.fh();
  float size = v[OVERLAY_SIZE] / 100, sx = size * v[OVERLAY_SIZE_X] / 100, sy = size * v[OVERLAY_SIZE_Y] / 100;
  if (std::fabs(sx) < 0.001f || std::fabs(sy) < 0.001f)
    return;
  float turn = v[OVERLAY_TURN] * 3.14159265f / 180, ct = cosf(turn), st = sinf(turn);
  float pu = (float) bw * v[OVERLAY_PIVOT_X] / 1000, pv = (float) bh * v[OVERLAY_PIVOT_Y] / 1000;
  float mx = v[OVERLAY_X] + pu, my = v[OVERLAY_Y] + pv;
  bool flip_x = ((c.view & OVERLAY_VIEW_FLIP_X) != 0) != c.mirrored, flip_y = c.view & OVERLAY_VIEW_FLIP_Y;
  bool rep_x = c.view & OVERLAY_VIEW_REPEAT_X, rep_y = c.view & OVERLAY_VIEW_REPEAT_Y,
       mirror = c.view & OVERLAY_VIEW_MIRROR;
  int blend = c.view & OVERLAY_VIEW_BLEND;
  // the box's corners, for how far it reaches
  float lo_x = 1e9f, hi_x = -1e9f, lo_y = 1e9f, hi_y = -1e9f;
  for (int k = 0; k < 4; k++) {
    float dx = ((k & 1 ? (float) bw : 0.0f) - pu) * sx, dy = ((k & 2 ? (float) bh : 0.0f) - pv) * sy;
    float rx = dx * ct - dy * st, ry = dx * st + dy * ct;
    lo_x = std::min(lo_x, mx + rx), hi_x = std::max(hi_x, mx + rx);
    lo_y = std::min(lo_y, my + ry), hi_y = std::max(hi_y, my + ry);
  }
  // row binds: each row of the box its own
  struct Row {
    float shift = 0, size = 100, opacity = 100, light = 100, phase = 0;
  };
  std::vector<Row> rows;
  bool upright = std::fabs(std::fmod(v[OVERLAY_TURN], 360.0f)) < 0.01f;
  bool row_binds = false;
  for (const Bind &b : c.binds) {
    unsigned to = b.what & ~OVERLAY_BIND_OP;
    row_binds |= to >= OVERLAY_BIND_ROW_SHIFT && to < OVERLAY_BIND_TARGETS;
  }
  if (row_binds && upright && !c.fading) {
    rows.assign((size_t) bh, Row{});
    Overlay *owner = nullptr;
    for (auto &o : overlays)
      if (o.an.get() == &an)
        owner = &o;
    Overlay dummy;
    dummy.an = std::shared_ptr<Anim>(std::shared_ptr<Anim>(), &an);
    Copy &cc = const_cast<Copy &>(c);
    for (int r = 0; r < bh; r++) {
      row_now = r;
      Row &o = rows[(size_t) r];
      for (const Bind &b : c.binds) {
        unsigned to = b.what & ~OVERLAY_BIND_OP, op = b.what & OVERLAY_BIND_OP;
        if (to < OVERLAY_BIND_ROW_SHIFT || to >= OVERLAY_BIND_TARGETS)
          continue;
        size_t pos = 0;
        float x = (float) value(owner ? *owner : dummy, cc, b.v, pos);
        float *t = to == OVERLAY_BIND_ROW_SHIFT     ? &o.shift
                   : to == OVERLAY_BIND_ROW_SIZE    ? &o.size
                   : to == OVERLAY_BIND_ROW_OPACITY ? &o.opacity
                   : to == OVERLAY_BIND_ROW_LIGHT   ? &o.light
                                                    : &o.phase;
        float fine = (float) text::BOUND_FINE[to - OVERLAY_BIND_FRAME];
        float f = x / fine;
        if (op == OVERLAY_BIND_SET)
          *t = f;
        else if (op == OVERLAY_BIND_MUL)
          *t = *t * x / 1000;
        else
          *t += f;
      }
    }
    row_now = 0;
  }
  // shape: runs of columns from the left
  struct Run {
    int x0, x1, top, bottom;
  };
  std::vector<Run> runs;
  for (size_t k = 0, at = 0; k + 2 < c.shape.size(); k += 3) {
    runs.push_back({(int) at, (int) at + c.shape[k], c.shape[k + 1], c.shape[k + 2]});
    at += (size_t) c.shape[k];
  }
  // each wrap of the screen, once a drift or wander has it going round it
  int wraps = c.torus && (!c.area || c.ex == OVERLAY_EDGE_SCREEN) ? 1 : 0;
  for (int wrap = -wraps; wrap <= wraps; wrap++)
    for (int y = std::max(0, (int) floorf(lo_y)); y <= std::min(H - 1, (int) ceilf(hi_y)); y++) {
      for (int x = (int) floorf(lo_x) - 1; x <= (int) ceilf(hi_x) + 1; x++) {
        int sx0 = x + wrap * W;
        if (sx0 < 0 || sx0 >= W)
          continue;
        float rx = (float) x + 0.5f - mx, ry = (float) y + 0.5f - my;
        float dx = (rx * ct + ry * st) / sx, dy = (-rx * st + ry * ct) / sy;
        if (flip_x)
          dx = -dx;
        if (flip_y)
          dy = -dy;
        float u = pu + dx, w = pv + dy;
        if (u < 0 || w < 0 || u >= (float) bw || w >= (float) bh)
          continue;
        int row = (int) floorf(w);
        float row_op = 1, row_light = v[OVERLAY_LIGHT], row_phase = v[OVERLAY_PHASE], fu = u;
        if (!rows.empty()) {
          const Row &o = rows[(size_t) row];
          fu = u - o.shift;
          if (o.size != 100 && o.size > 0)
            fu = pu + (fu - pu) * 100 / o.size;
          row_op = std::clamp(o.opacity, 0.0f, 100.0f) / 100;
          row_light = v[OVERLAY_LIGHT] * o.light / 100;
          row_phase += o.phase;
        }
        float soft = 1;
        int top = 0, bottom = bh;
        if (!runs.empty()) {
          bool in = false;
          for (const Run &r : runs)
            if (u >= (float) r.x0 && u < (float) r.x1 && w >= (float) r.top && w < (float) r.bottom) {
              in = true, top = r.top, bottom = r.bottom;
              break;
            }
          if (!in)
            continue;
        }
        if (c.soft[0] > 0)
          soft = std::min(soft, u / (float) c.soft[0]);
        if (c.soft[1] > 0)
          soft = std::min(soft, ((float) bw - u) / (float) c.soft[1]);
        if (c.soft[2] > 0)
          soft = std::min(soft, (w - (float) top) / (float) c.soft[2]);
        if (c.soft[3] > 0)
          soft = std::min(soft, ((float) bottom - w) / (float) c.soft[3]);
        soft = std::clamp(soft, 0.0f, 1.0f);
        uint32_t p;
        if (!px) {
          p = field_at(an, c, u, w, (float) bw, (float) bh);
          if ((p >> 24) == 0)
            continue;
          p = (p & 0xFF000000) | lit(an, c, p & 0xFFFFFF, row_light);
        } else {
          float gx = fu - v[OVERLAY_PAN_X], gy = w - v[OVERLAY_PAN_Y];
          int tx = (int) floorf(gx / (float) fw), ty = (int) floorf(gy / (float) fh);
          if (!rep_x && tx != 0)
            continue;
          if (!rep_y && ty != 0)
            continue;
          float lx = gx - (float) tx * (float) fw, ly = gy - (float) ty * (float) fh;
          if (rep_x && mirror && (tx & 1))
            lx = (float) fw - lx;
          if (rep_y && mirror && (ty & 1))
            ly = (float) fh - ly;
          int ix = std::clamp((int) floorf(lx), 0, fw - 1) / src.scale,
              iy = std::clamp((int) floorf(ly), 0, fh - 1) / src.scale;
          p = (*px)[(size_t) iy * src.w + ix];
          if (!(p >> 24))
            continue;
          if (!src.cycles.empty() && row_phase) {  // its cycling colours go round
            auto at = std::find(src.cycles.begin(), src.cycles.end(), p & 0xFFFFFF);
            if (at != src.cycles.end()) {
              int n = (int) src.cycles.size(), i = (int) (at - src.cycles.begin());
              float q = std::fmod(row_phase, (float) n);
              if (q < 0)
                q += (float) n;
              int whole = (int) q;
              float f = q - (float) whole;
              p = (p & 0xFF000000) |
                  mix(src.cycles[(size_t) ((i + whole) % n)], src.cycles[(size_t) ((i + whole + 1) % n)], f);
            }
          }
          p = (p & 0xFF000000) | lit(an, c, p & 0xFFFFFF, row_light);
        }
        if (hidden(sx0, y, W, (uint8_t) std::clamp(lroundf(v[OVERLAY_DEPTH]), 0l, 255l)))
          continue;
        float a = (float) (p >> 24) / 255 * op * row_op * soft;
        if (a <= 0.002f)
          continue;
        if (wrap)
          wrapped[an.name]++;
        uint32_t &d = rgb[(size_t) y * W + sx0];
        uint32_t out = 0;
        for (int sh = 0; sh <= 16; sh += 8) {
          float bk = (float) (d >> sh & 255), t = (float) (p >> sh & 255), r;
          switch (blend) {
            case OVERLAY_BLEND_ADD:
              r = std::min(255.0f, bk + t * a);
              break;
            case OVERLAY_BLEND_SCREEN:
              r = bk + (255 - bk) * (t / 255) * a;
              break;
            case OVERLAY_BLEND_MULTIPLY:
              r = bk * (1 - a + a * t / 255);
              break;
            default:
              r = bk + (t - bk) * a;
          }
          out |= (uint32_t) lroundf(std::clamp(r, 0.0f, 255.0f)) << sh;
        }
        d = out;
      }
    }
}

inline void draw(uint32_t *rgb, int W, int H) {
  struct Item {
    Anim *an;
    const Copy *c;
    int frame;
    Overlay *a;
  };
  std::vector<Item> items;
  for (Overlay &a : overlays)
    if (!a.name.empty())
      for (Copy &c : a.copies)
        if (c.used)
          items.push_back({a.an.get(), &c, frame_of(a, c), &a});
  for (Ghost &g : ghosts)
    items.push_back({g.an.get(), &g.c, g.c.own ? g.c.frame : g.an->frame_at(millis() - g.an->t0), nullptr});
  std::stable_sort(items.begin(), items.end(), [](const Item &p, const Item &q) {
    float dp = p.c->val[OVERLAY_DEPTH], dq = q.c->val[OVERLAY_DEPTH];
    if (dp != dq)
      return dp < dq;
    return p.c->val[OVERLAY_SIZE] < q.c->val[OVERLAY_SIZE];
  });
  for (Item &it : items)
    draw_copy(*it.an, *it.c, rgb, W, H, it.frame);
}

// Copies on the panel now, and how many animations they're of
inline int copies_shown(int *n_anims = nullptr) {
  int n = 0, k = 0;
  for (auto &a : overlays) {
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

// Slots taken now, and the bytes of OVERLAY_POOL their frames and scripts take
inline int slots_used(size_t *bytes = nullptr) {
  int n = 0;
  for (auto &a : overlays)
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
  for (Overlay &a : overlays)
    for (Copy &c : a.copies)
      if (!a.name.empty() && c.used && !c.fading) {
        Where &w = where[a.name];
        float l = c.base[OVERLAY_X], r = l + (float) box_w(c);
        w.n++;
        w.x += (l + r) / 2;
        w.edge += l < 16 || r > PIC_MAX_W - 16;
      }
}

// Everything off, for the next picture
inline void reset() {
  for (Overlay &a : overlays)
    a = Overlay();
  ghosts.clear();
  sources.clear();
  wrapped.clear();
  crossed.clear();
  where.clear();
  std::fill(std::begin(shared), std::end(shared), 0);
  tags = 0;
}

}  // namespace flexit::panel::overlay
