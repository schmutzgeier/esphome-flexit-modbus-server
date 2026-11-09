// Stands in for packages/code/panel/overlay.h when met_sim builds the weather animations on a PC: the ESP's side
// of the link becomes a small model of what the panel does with an animation's copies, after plink_keys.h's
// K_OVERLAY_* (place, glide, drift, wander, face, flip, wave, map, depth, tint, fades) and the scripts the copies
// run, compiled by the real overlay_script.h and stepped here as the panel would, on a clock of its own. It also
// gives the little of ESPHome that met_live*.h use: millis(), the logs, the time and picture::has_ram().
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

// As overlay.h's, which the shows fill in
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
  std::vector<uint32_t> glows = {};  // 0xRRGGBB, its pixels in them glow: no light or tint on them
  uint32_t shade = 0;                // 0xRRGGBB, where the light below 100 takes its colours
};
inline std::map<std::string, Source> sources;
inline void add(const std::string &name, Source s) { sources[name] = std::move(s); }

enum Phase { IDLE, COUNT, MAKE, SEND };
struct Work {
  Phase phase = IDLE;  // each animation is sent at once, and the encounters find the link free
};
inline Work W;

// The channels, OVERLAY_X..OVERLAY_LIGHT
enum { X, Y, TURN, SIZE, OPACITY, SPEED, LIGHT, NCH };
static_assert((int) NCH == (int) OVERLAY_CHANNELS, "the panel's channels");
inline int channel_of(const std::string &s) { return text::index(CHANNELS, s); }
inline int no_change(int ch) { return ch >= SIZE ? 100 : 0; }
// OVERLAY_EASE_* bits: 1 in, 2 out, 3 both
inline int ease_of(const std::string &s) { return std::max(0, text::index(EASES, s)); }
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

// A copy on the panel, in one of its places
struct Copy {
  bool used = false, gone = false;
  uint8_t place = 0;  // the panel's place, for the scripts that look at it
  float base[NCH] = {0, 0, 0, 100, 100, 100, 100};
  Glide glide[NCH];
  float drift[3] = {};  // px or degrees a second
  bool own = false;     // a frame of its own, played by its clip
  uint32_t placed = 0;
  int fade = -1;  // as placed, -1 the animation's
  int fade_in = 300;
  // fading out: from fade_t0 over fade_ms, then gone
  bool fading = false;
  uint32_t fade_t0 = 0;
  int fade_ms = 0;
  int ax0 = -32768, ay0 = -32768, ax1 = 32767, ay1 = 32767, ex = 0, ey = 0;  // area and edges, OVERLAY_EDGE_*
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
  Wave wave[NCH];
  Map map[NCH];
  float px = 0, py = 0;             // where it was a step ago, for face
  uint8_t depth = OVERLAY_NEAREST;  // K_OVERLAY_DEPTH's: behind the picture's parts nearer than it
  bool tint = false;
  uint32_t tint_rgb = 0;
  // its frames, with one of its own: the clip, first to last, w x h of each from its top left (0 all of it), the
  // frame it's on and how far into it, its rate, and the rounds of its clip so far. A clip at the end of a round.
  int first = 0, last = 0, cw = 0, ch = 0, frame = 0, rate = 100, rounds = 0;
  float frame_t = 0;
  bool next = false, wrapped = false;  // wrapped: a round ended this step
  int nfirst = 0, nlast = 0, ncw = 0, nch = 0;
  // its script: the steps, where it is, its registers, what it waits for, and its END's tag
  std::vector<uint8_t> code;
  bool running = false, ended = false;
  size_t pc = 0;
  int32_t reg[OVERLAY_OWN_REGS] = {};
  enum { NONE, MS, STEP, ROUNDS, CLIP } wait = NONE;
  uint32_t until = 0, started = 0;
  int rounds_to = 0;
  uint8_t tag = 0;
};

enum State { WANT, SENT, FAILED };

// A slot on the panel, OVERLAY_SLOTS of them. It's sent at once here, or FAILED when its frame doesn't fit
// OVERLAY_POOL.
struct Overlay {
  std::string name;  // "" = free slot
  Source src;
  uint32_t t0 = 0;
  bool cleared = false;
  int state = WANT;
  std::array<Copy, OVERLAY_PLACES> copies;
  size_t frame_bytes() const { return (size_t) (src.w + 1) / 2 * src.h; }
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
inline Overlay overlays[OVERLAY_SLOTS];
inline int32_t shared[OVERLAY_REGS - OVERLAY_OWN_REGS];  // the registers all scripts share
inline uint8_t tags;

// Called with the animation when its script ended the last copy it had, and with a copy's marks
inline std::function<void(const std::string &)> emptied;
inline std::function<void(const std::string &, int, int)> marked;

inline Overlay *find(const std::string &name) {
  for (auto &a : overlays)
    if (!a.name.empty() && a.name == name && !a.cleared)
      return &a;
  return nullptr;
}

// Bytes of OVERLAY_POOL the animations on the panel take, with their copies' own frames and scripts, other than a's
inline size_t used_by_others(const Overlay *a) {
  size_t n = 0;
  for (auto &o : overlays) {
    if (&o == a || o.name.empty() || o.state != SENT)
      continue;
    n += o.frame_bytes() + o.data_bytes();
    for (auto &c : o.copies)
      if (c.used)
        n += (c.own ? o.frame_bytes() : 0) + (c.code.empty() ? 0 : OVERLAY_SCRIPT_POOL + c.code.size());
  }
  return n;
}

// The slot of name, taking a free one: null when all OVERLAY_SLOTS are taken. One that doesn't fit the pool next to
// the others is FAILED, as the panel's PIC_TOO_BIG would have it.
inline Overlay *want(const std::string &name) {
  if (Overlay *a = find(name))
    return a->state == FAILED ? nullptr : a;
  auto s = sources.find(name);
  if (s == sources.end())
    return nullptr;
  Overlay *a = nullptr;
  for (auto &o : overlays)
    if (o.name.empty()) {
      a = &o;
      break;
    }
  for (auto &o : overlays)  // overlay.h frees a cleared one's slot at once, the panel fades its copies on its own
    if (!a && o.cleared)
      a = &o;
  if (!a) {
    sim::log('W', "anim", "%s: all %d slots taken", name.c_str(), OVERLAY_SLOTS);
    return nullptr;
  }
  *a = Overlay();
  a->name = name;
  a->src = s->second;
  a->t0 = millis();
  size_t need = a->frame_bytes() + a->data_bytes(), used = used_by_others(a);
  a->state = used + need <= OVERLAY_POOL ? SENT : FAILED;
  if (a->state == FAILED) {
    sim::log('W', "anim", "%s: %zu bytes don't fit next to the others' %zu of %d", name.c_str(), need, used,
             OVERLAY_POOL);
    return nullptr;
  }
  return a;
}
inline Copy *copy_of(const std::string &name, int i) {
  Overlay *a = find(name);
  return a && i >= 0 && i < OVERLAY_PLACES && a->copies[i].used && !a->copies[i].fading ? &a->copies[i] : nullptr;
}
inline Overlay *slot_for(const std::string &name) { return want(name); }

// Copies on the panel, each in one of its OVERLAY_PLACES. One fading out has let go of its place, as overlay.h's
// remove() has it, the panel fades it on its own.
inline int places_used() {
  int n = 0;
  for (auto &a : overlays)
    for (auto &c : a.copies)
      n += c.used && !c.fading;
  return n;
}

// Copy i of name and its animation, nulls if it isn't shown
inline std::pair<Overlay *, Copy *> copy_for(const std::string &name, int i, const char * = nullptr) {
  Overlay *a = find(name);
  if (a && i >= 0 && i < OVERLAY_PLACES && a->copies[i].used && !a->copies[i].fading)
    return {a, &a->copies[i]};
  return {};
}
// The copy at place p, null if none is there
inline std::pair<Overlay *, Copy *> at_place(uint8_t p) {
  for (auto &a : overlays)
    for (auto &c : a.copies)
      if (!a.name.empty() && c.used && !c.fading && c.place == p)
        return {&a, &c};
  return {};
}
// The place of copy i of name, for a script that looks at it, or one nothing's at
inline uint8_t place_of(const std::string &name, int i) {
  auto [a, c] = copy_for(name, i);
  return c ? c->place : (uint8_t) 0xFE;
}

inline void fade_out(Overlay &a, Copy &c, int fade) {
  c.fading = true;
  c.running = false;
  c.fade_t0 = millis();
  c.fade_ms = fade >= 0 ? fade : a.src.fade_ms;
}

// ---- the messages, as the panel takes them

inline void start_leg(Wave &v, int step) {
  v.step = step;
  v.t0 = millis();
  float k = 1 + rnd((float) -v.jitter, (float) v.jitter) / 100.0f;
  if (step == 0 || step == 2) {
    v.a = v.cur;
    float to = step == 0 ? (float) v.hi : (float) v.lo;
    if (v.random)
      to = step == 0 ? rnd(v.cur, (float) v.hi) : rnd((float) v.lo, v.cur);
    v.b = to;
    float span = v.hi > v.lo ? std::fabs(to - v.cur) / (float) (v.hi - v.lo) : 1;
    v.ms = (uint32_t) ((float) (step == 0 ? v.rise : v.fall) * span * k);
  } else {
    v.ms = (uint32_t) ((float) (step == 1 ? v.hold_hi : v.hold_lo) * k);
  }
}

inline void glide_ch(Copy &c, int ch, float to, int ms, int ease) {
  if (ch == TURN) {  // the short way round
    float d = std::fmod(to - c.base[TURN], 360.0f);
    if (d > 180)
      d -= 360;
    else if (d < -180)
      d += 360;
    to = c.base[TURN] + d;
    c.face = false;
  }
  c.glide[ch] = {true, c.base[ch], to, millis(), (uint32_t) std::max(0, ms), ease};
  if (ch <= Y) {
    c.torus = false;  // the panel's glide can end partly off the screen, and that's all
    c.wander.on = false;
  }
  if (ch <= TURN)
    c.drift[ch] = 0;
  if (!ms) {
    c.base[ch] = to;
    c.glide[ch].on = false;
  }
}

// Message key, laid out as in plink_keys.h, to copy c
inline void apply(Copy &c, uint16_t key, const std::vector<uint8_t> &m) {
  auto u16 = [&](int at) { return (int) plink_get16(&m[at]); };
  auto i16 = [&](int at) { return (int) plink_i16(&m[at]); };
  switch (key) {
    case K_OVERLAY_GLIDE: {
      int bits = m[OVERLAY_GLIDE_CHANNELS], k = 0;
      for (int ch = 0; ch < NCH; ch++)
        if (bits >> ch & 1)
          glide_ch(c, ch, (float) i16(OVERLAY_GLIDE_HEAD + 2 * k++), u16(OVERLAY_GLIDE_MS), m[OVERLAY_GLIDE_EASE]);
      break;
    }
    case K_OVERLAY_DRIFT: {
      int bits = m[OVERLAY_DRIFT_CHANNELS], k = 0;
      for (int ch = X; ch <= TURN; ch++)
        if (bits >> ch & 1) {
          int v = i16(OVERLAY_DRIFT_HEAD + 2 * k++);
          c.drift[ch] = ch == TURN ? (float) v / 10 : (float) v / 100;
          c.glide[ch].on = false;
          if (ch <= Y)
            c.wander.on = false;
          if (ch == TURN)
            c.face = false;
        }
      break;
    }
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
      c.glide[X].on = c.glide[Y].on = false;
      c.drift[X] = c.drift[Y] = 0;
      break;
    }
    case K_OVERLAY_FACE:
      c.face = true;
      c.face_mirror = m[OVERLAY_FACE_FLAGS] & OVERLAY_FACE_MIRROR;
      c.facing = u16(OVERLAY_FACE_FACING);
      break;
    case K_OVERLAY_FLIP:
      c.flip_x = m[OVERLAY_FLIP_FLAGS] & OVERLAY_FLIP_X;
      c.flip_y = m[OVERLAY_FLIP_FLAGS] & OVERLAY_FLIP_Y;
      break;
    case K_OVERLAY_WAVE: {
      int ch = m[OVERLAY_WAVE_CHANNEL], lo = i16(OVERLAY_WAVE_LO), hi = i16(OVERLAY_WAVE_HI);
      if (ch >= NCH)
        break;
      Wave &v = c.wave[ch];
      v = {};
      if (lo == hi && lo == no_change(ch))
        break;  // gone
      v.on = true;
      v.lo = lo, v.hi = hi, v.rise = u16(OVERLAY_WAVE_MS), v.fall = u16(OVERLAY_WAVE_MS + 2);
      v.hold_hi = u16(OVERLAY_WAVE_HOLD), v.hold_lo = u16(OVERLAY_WAVE_HOLD + 2);
      v.jitter = m[OVERLAY_WAVE_JITTER];
      v.random = m[OVERLAY_WAVE_FLAGS] & OVERLAY_WAVE_RANDOM;
      v.slow = m[OVERLAY_WAVE_FLAGS] & 3;
      v.cur = (float) lo;
      start_leg(v, 0);
      break;
    }
    case K_OVERLAY_MAP: {
      int ch = m[OVERLAY_MAP_CHANNEL];
      if (ch >= TURN && ch < NCH)
        c.map[ch] = {true, i16(OVERLAY_MAP_VALUES), i16(OVERLAY_MAP_VALUES + 2), i16(OVERLAY_MAP_VALUES + 4),
                     i16(OVERLAY_MAP_VALUES + 6)};
      break;
    }
    case K_OVERLAY_DEPTH:
      c.depth = m[OVERLAY_DEPTH_DEPTH];
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

// ---- the scripts, stepped as the panel runs them

// What the wave adds to channel ch now
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
// Channel ch of c as it's shown, with its wave
inline float shown(Copy &c, int ch) {
  float v = c.base[ch];
  if (c.wave[ch].on)
    v = ch <= TURN ? v + wave_value(c.wave[ch]) : v * wave_value(c.wave[ch]) / 100;
  return v;
}
inline int box_w(const Overlay &a, const Copy &c) { return c.own && c.cw ? std::min(c.cw, a.src.w) : a.src.w; }
inline int box_h(const Overlay &a, const Copy &c) { return c.own && c.ch ? std::min(c.ch, a.src.h) : a.src.h; }
// The frame c shows, and its rounds so far: its own, or in step with its slot
inline int frame_of(const Overlay &a, const Copy &c) { return c.own ? c.frame : a.frame_at(millis() - a.t0); }
inline int rounds_of(const Overlay &a, const Copy &c) {
  return c.own ? c.rounds : (int) ((millis() - a.t0) / (uint32_t) a.total_ms());
}

inline int32_t get(Overlay &a, Copy &c, int what, const Copy &self) {
  switch (what) {
    case OVERLAY_GET_X:
      return (int32_t) lroundf(shown(c, X));
    case OVERLAY_GET_Y:
      return (int32_t) lroundf(shown(c, Y));
    case OVERLAY_GET_W:
      return box_w(a, c);
    case OVERLAY_GET_H:
      return box_h(a, c);
    case OVERLAY_GET_TURN:
      return ((int32_t) lroundf(shown(c, TURN)) % 360 + 360) % 360;
    case OVERLAY_GET_SIZE:
      return (int32_t) lroundf(shown(c, SIZE));
    case OVERLAY_GET_OPACITY:
      return (int32_t) lroundf(shown(c, OPACITY));
    case OVERLAY_GET_LIGHT:
      return (int32_t) lroundf(shown(c, LIGHT));
    case OVERLAY_GET_FRAME:
      return frame_of(a, c);
    case OVERLAY_GET_ROUNDS:
      return rounds_of(a, c);
    case OVERLAY_GET_HERE:
      return 1;
    case OVERLAY_GET_MS:
      return c.code.empty() ? 0 : (int32_t) (millis() - c.started);
  }
  (void) self;
  return 0;
}
inline void middle(Overlay &a, Copy &c, float &x, float &y) {
  x = shown(c, X) + box_w(a, c) / 2.0f;
  y = shown(c, Y) + box_h(a, c) / 2.0f;
}

inline void set_clip(Copy &c, int first, int last, int w, int h, const Overlay &a) {
  int n = std::max(1, a.src.frames);
  c.first = ((first % n) + n) % n;
  c.last = ((last % n) + n) % n;
  if (c.last < c.first)
    c.last = c.first;
  c.cw = w, c.ch = h;
  c.frame = c.first;
  c.frame_t = 0;
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
    return (int) (int16_t) (uint16_t) (lo | byte() << 8);
  };
  auto reg = [&](int r) -> int32_t & {
    static int32_t none;
    return r < OVERLAY_OWN_REGS ? c.reg[r] : r < OVERLAY_REGS ? shared[r - OVERLAY_OWN_REGS] : (none = 0);
  };
  auto val = [&]() -> int32_t {
    int t = byte();
    if (t == OVERLAY_V16)
      return two();
    if (t == OVERLAY_V32) {
      uint32_t v = 0;
      for (int k = 0; k < 4; k++)
        v |= (uint32_t) byte() << 8 * k;
      return (int32_t) v;
    }
    return reg(t - OVERLAY_VREG);
  };
  auto place = [&]() -> std::pair<Overlay *, Copy *> {
    int p = byte();
    return p == OVERLAY_SELF ? std::pair<Overlay *, Copy *>{&a, &c} : at_place((uint8_t) p);
  };
  auto holds = [](int32_t l, int cmp, int32_t r) {
    switch (cmp) {
      case OVERLAY_IF_LT:
        return l < r;
      case OVERLAY_IF_LE:
        return l <= r;
      case OVERLAY_IF_EQ:
        return l == r;
      case OVERLAY_IF_NE:
        return l != r;
      case OVERLAY_IF_GE:
        return l >= r;
      default:
        return l > r;
    }
  };
  for (int steps = 0; steps < OVERLAY_SCRIPT_STEPS; steps++) {
    if (c.pc >= b.size()) {
      c.running = false;
      return;
    }
    int op = byte();
    switch (op) {
      case OVERLAY_DO_SET:
      case OVERLAY_DO_ADD:
      case OVERLAY_DO_SUB:
      case OVERLAY_DO_MUL:
      case OVERLAY_DO_DIV:
      case OVERLAY_DO_MOD:
      case OVERLAY_DO_MIN:
      case OVERLAY_DO_MAX:
      case OVERLAY_DO_ABS: {
        int32_t &r = reg(byte());
        int32_t v = val();
        uint32_t ur = (uint32_t) r, uv = (uint32_t) v;
        switch (op) {
          case OVERLAY_DO_SET:
            r = v;
            break;
          case OVERLAY_DO_ADD:
            r = (int32_t) (ur + uv);
            break;
          case OVERLAY_DO_SUB:
            r = (int32_t) (ur - uv);
            break;
          case OVERLAY_DO_MUL:
            r = (int32_t) (ur * uv);
            break;
          case OVERLAY_DO_DIV:
            r = v ? (int32_t) ((int64_t) r / v) : 0;
            break;
          case OVERLAY_DO_MOD:
            r = v ? (int32_t) ((int64_t) r % v) : 0;
            break;
          case OVERLAY_DO_MIN:
            r = std::min(r, v);
            break;
          case OVERLAY_DO_MAX:
            r = std::max(r, v);
            break;
          default:
            r = (int32_t) (v < 0 ? 0u - uv : uv);
        }
        break;
      }
      case OVERLAY_DO_RAND: {
        int32_t &r = reg(byte());
        int32_t lo = val(), hi = val();
        if (hi < lo)
          std::swap(lo, hi);
        r = std::uniform_int_distribution<int32_t>(lo, hi)(rng);
        break;
      }
      case OVERLAY_DO_GET: {
        int32_t &r = reg(byte());
        int what = byte();
        auto [pa, pc] = place();
        r = pc ? get(*pa, *pc, what, c) : 0;
        break;
      }
      case OVERLAY_DO_DIST:
      case OVERLAY_DO_ANGLE: {
        int32_t &r = reg(byte());
        auto [a1, c1] = place();
        auto [a2, c2] = place();
        if (!c1 || !c2) {
          r = 0;
          break;
        }
        float x1, y1, x2, y2;
        middle(*a1, *c1, x1, y1);
        middle(*a2, *c2, x2, y2);
        if (op == OVERLAY_DO_DIST)
          r = (int32_t) lroundf(std::hypot(x2 - x1, y2 - y1));
        else
          r = ((int32_t) lroundf(atan2f(y2 - y1, x2 - x1) * 180 / 3.14159265f) + 360) % 360;
        break;
      }
      case OVERLAY_DO_SIN:
      case OVERLAY_DO_COS: {
        int32_t &r = reg(byte());
        int32_t deg = val(), k = val();
        float rad = (float) deg * 3.14159265f / 180;
        r = (int32_t) lroundf((float) k * (op == OVERLAY_DO_SIN ? sinf(rad) : cosf(rad)));
        break;
      }
      case OVERLAY_DO_PICK: {
        int32_t &r = reg(byte());
        int32_t k = val();
        int n = byte();
        size_t at = c.pc;
        c.pc += 2 * (size_t) n;
        int i = std::clamp<int32_t>(k, 0, n - 1);
        r = n ? (int32_t) (int16_t) (uint16_t) (b[at + 2 * i] | b[at + 2 * i + 1] << 8) : 0;
        break;
      }
      case OVERLAY_DO_GOTO: {
        int to = byte();
        c.pc = (size_t) (to | byte() << 8);
        break;
      }
      case OVERLAY_DO_IF: {
        int32_t l = val();
        int cmp = byte();
        int32_t r = val();
        int to = byte();
        to |= byte() << 8;
        if (holds(l, cmp, r))
          c.pc = (size_t) to;
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
        int32_t l = val();
        int cmp = byte();
        int32_t r = val();
        int to = byte();
        to |= byte() << 8;
        if (!holds(l, cmp, r)) {
          c.pc = (size_t) to;  // worked out again at the next motion step
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
        int mode = byte();
        int w = byte();
        w |= byte() << 8;
        int h = byte();
        h |= byte() << 8;
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
        int n = std::max(1, a.src.frames);
        c.frame = ((val() % n) + n) % n;
        c.frame_t = 0;
        break;
      }
      case OVERLAY_DO_RATE:
        c.rate = std::clamp<int32_t>(val(), 0, OVERLAY_RATE_MAX);
        break;
      case OVERLAY_DO_MOVE: {
        int32_t x = val(), y = val(), ms = val();
        int e = byte();
        glide_ch(c, X, (float) x, ms, e);
        glide_ch(c, Y, (float) y, ms, e);
        break;
      }
      case OVERLAY_DO_SEND: {
        uint16_t key = (uint16_t) (K_OVERLAY_FRAMES + byte());
        int n = byte();
        std::vector<uint8_t> m(b.begin() + (long) c.pc, b.begin() + (long) std::min(b.size(), c.pc + n));
        c.pc += (size_t) n;
        int k = byte();
        for (int f = 0; f < k; f++) {
          int at = byte();
          int32_t v = val();
          if (at + 1 < (int) m.size() && at >= 0)
            plink_put16(&m[at], (uint16_t) v);
        }
        apply(c, key, m);
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
        c.ended = true;
        fade_out(a, c, std::max<int32_t>(0, fade));
        return;
      }
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
inline void set_script(Overlay &a, Copy &c, const Program &p) {
  for (auto &t : p.told)
    apply(c, t.key, t.m);
  c.code = p.bytes;
  c.tag = 0;
  if (p.ends() && !(c.tag = ++tags))
    c.tag = ++tags;
  for (auto &q : p.patches) {
    if (q.at >= c.code.size())
      continue;
    if (q.kind == Program::PLACE)
      c.code[q.at] = place_of(q.name, q.copy);
    else if (q.kind == Program::TAG)
      c.code[q.at] = c.tag;
    else if ((size_t) q.at + 1 < c.code.size())
      plink_put16(&c.code[q.at], (uint16_t) (c.fade >= 0 ? c.fade : a.src.fade_ms));
  }
  c.pc = 0;
  c.wait = Copy::NONE;
  c.next = false;
  c.ended = false;
  std::fill(std::begin(c.reg), std::end(c.reg), 0);
  c.started = millis();
  c.running = !c.code.empty();
  run(a, c);  // at once
}

// ---- what overlay.h offers

// The script of start frames in, for a copy with a frame of its own
inline Program started(int start) {
  std::string error;
  return start > 0 ? compile("frame " + std::to_string(start), error) : Program();
}

inline bool place(const std::string &name, int i, int x, int y, Program p, bool own = false, int fade = -1,
                  uint8_t depth = OVERLAY_NEAREST) {
  Overlay *a = want(name);
  if (!a || i < 0 || i >= OVERLAY_PLACES)
    return false;
  Copy &c = a->copies[i];
  if ((!c.used || c.fading) && places_used() >= OVERLAY_PLACES) {
    sim::log('W', "anim", "%s: all %d places taken", name.c_str(), OVERLAY_PLACES);
    return false;
  }
  c = Copy();
  c.used = true;
  c.place = (uint8_t) ((a - overlays) * OVERLAY_PLACES + i);
  c.base[X] = c.px = (float) x;
  c.base[Y] = c.py = (float) y;
  c.own = own || p.frames;
  c.placed = millis();
  c.fade = fade;
  c.fade_in = fade >= 0 ? fade : a->src.fade_ms;
  c.depth = depth;
  set_clip(c, 0, a->src.frames - 1, 0, 0, *a);
  set_script(*a, c, p);
  return true;
}
inline bool place(const std::string &name, int i, int x, int y, bool own, int start, int fade = -1,
                  uint8_t depth = OVERLAY_NEAREST) {
  return place(name, i, x, y, own ? started(start) : Program(), own, fade, depth);
}
inline void remove(const std::string &name, int i, int fade = -1) {
  auto [a, c] = copy_for(name, i);
  if (c)
    fade_out(*a, *c, fade);
}
inline void depth(const std::string &name, int i, uint8_t d) {
  if (Copy *c = copy_for(name, i).second)
    c->depth = d;
}
inline void clear(const std::string &name) {
  if (Overlay *a = find(name)) {
    for (Copy &c : a->copies)
      if (c.used && !c.fading)
        fade_out(*a, c, -1);
    a->cleared = true;
  }
}
// Copy i runs the script in text, the one it ran before gone
inline bool script(const std::string &name, int i, const std::string &text, const Consts &consts = {}) {
  auto [a, c] = copy_for(name, i);
  if (!c)
    return false;
  std::string error;
  Program p = compile(text, error, a->src.clips, consts);
  if (error.empty() && p.frames && !c->own)
    error = "it plays frames, and copy " + std::to_string(i) + " has no frame of its own";
  if (!error.empty()) {
    sim::log('W', "anim", "%s's script, %s", name.c_str(), error.c_str());
    return false;
  }
  set_script(*a, *c, p);
  return true;
}
inline void rounds(const std::string &name, int i, int n) {
  script(name, i, n > 0 ? "wait rounds " + std::to_string(n) + "; end" : "");
}

// The actions that tell a copy one thing, as the messages they send
inline void tell(const std::string &name, int i, uint16_t key, std::vector<uint8_t> m) {
  if (Copy *c = copy_for(name, i).second)
    apply(*c, key, m);
}
inline void glide(const std::string &name, int i, const std::string &what, int to, int ms, const std::string &ease) {
  int ch = channel_of(what);
  if (ch >= 0)
    tell(name, i, K_OVERLAY_GLIDE, msg::glide(ch, {to}, ms, ease_of(ease)));
}
inline void move(const std::string &name, int i, int x, int y, int ms, const std::string &ease) {
  tell(name, i, K_OVERLAY_GLIDE, msg::glide(X, {x, y}, ms, ease_of(ease)));
}
inline void drift(const std::string &name, int i, float vx, float vy) {
  tell(name, i, K_OVERLAY_DRIFT, msg::drift(X, {hundredths(vx), hundredths(vy)}));
}
inline void spin(const std::string &name, int i, float speed) {
  tell(name, i, K_OVERLAY_DRIFT, msg::drift(TURN, {tenths(speed)}));
}
inline void flip(const std::string &name, int i, bool x, bool y) { tell(name, i, K_OVERLAY_FLIP, msg::flip(x, y)); }
inline void area(const std::string &name, int i, int l, int t, int r, int b, const std::string &ex,
                 const std::string &ey) {
  tell(name, i, K_OVERLAY_AREA,
       msg::area(l, t, r, b, std::max(0, text::index(EDGES, ex)), std::max(0, text::index(EDGES, ey))));
}
inline void wander(const std::string &name, int i, float s0, float s1, int heading, int spread, int l0, int l1, int r0,
                   int r1) {
  tell(name, i, K_OVERLAY_WANDER,
       msg::wander((int) lroundf(s0 * 10), (int) lroundf(s1 * 10), heading, spread, l0, l1, r0, r1));
}
inline void face(const std::string &name, int i, int facing, bool mirror, int tilt, int rate, bool level) {
  tell(name, i, K_OVERLAY_FACE, msg::face(facing, mirror, tilt, rate, level));
}
inline void wave(const std::string &name, int i, const std::string &what, int lo, int hi, int rise, int fall,
                 int hold_high, int hold_low, int jitter, bool random, const std::string &slow) {
  int ch = channel_of(what), s = std::max(0, text::index(SLOWS, slow));
  if (ch >= 0)
    tell(name, i, K_OVERLAY_WAVE,
         msg::wave(ch, lo, hi, rise, fall, hold_high, hold_low, jitter, s | (random ? OVERLAY_WAVE_RANDOM : 0)));
}
inline void map(const std::string &name, int i, const std::string &what, int l, int r, int t, int b) {
  int ch = channel_of(what);
  if (ch >= TURN)
    tell(name, i, K_OVERLAY_MAP, msg::map(ch, l, r, t, b));
}

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
  if (!c.own || a.src.frames <= 0)
    return;
  c.frame_t += (float) dt * (float) c.rate / 100;
  for (int guard = 0; guard < 64 && c.rate > 0 && c.frame_t >= (float) a.frame_ms(c.frame); guard++) {
    c.frame_t -= (float) a.frame_ms(c.frame);
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

// Moves every copy on by dt ms, and runs their scripts
inline void step(uint32_t dt) {
  uint32_t now = millis();
  float s = (float) dt / 1000.0f;
  std::vector<std::string> empty;  // animations a script's END left without a copy
  for (Overlay &a : overlays) {
    if (a.name.empty())
      continue;
    for (Copy &c : a.copies) {
      if (!c.used)
        continue;
      if (c.fading && now - c.fade_t0 >= (uint32_t) c.fade_ms) {
        bool ended = c.ended;
        c = Copy();
        if (ended && std::none_of(a.copies.begin(), a.copies.end(), [](const Copy &o) { return o.used; }))
          empty.push_back(a.name);
        continue;
      }
      play(a, c, dt);
      run(a, c);
      float speed = shown(c, SPEED) / 100.0f;
      for (int ch = 0; ch < NCH; ch++) {
        Glide &g = c.glide[ch];
        if (g.on) {
          float t = g.ms ? (float) (now - g.t0) / (float) g.ms : 1;
          c.base[ch] = g.from + (g.to - g.from) * eased(t, g.ease);
          if (t >= 1)
            g.on = false;
        }
      }
      float w = (float) box_w(a, c), h = (float) box_h(a, c);
      float x0 = c.area ? (float) c.ax0 : 0, x1 = c.area ? (float) c.ax1 : 320;
      float y0 = c.area ? (float) c.ay0 : 0, y1 = c.area ? (float) c.ay1 : 240;
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
  for (Overlay &a : overlays) {  // a cleared one lets go of its slot once its copies have faded
    if (a.name.empty() || !(a.cleared || a.state == FAILED))
      continue;
    bool any = false;
    for (Copy &c : a.copies)
      any |= c.used;
    if (!any)
      a = Overlay();
  }
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

// 0xRRGGBB c in the copy's tint and light, but for the colours that glow
inline uint32_t lit(const Overlay &a, const Copy &c, uint32_t p, float light) {
  if (std::find(a.src.glows.begin(), a.src.glows.end(), p) != a.src.glows.end())
    return p;
  auto mix = [](uint32_t from, uint32_t to, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    uint32_t r = 0;
    for (int sh = 0; sh <= 16; sh += 8) {
      float f = (float) (from >> sh & 255), g = (float) (to >> sh & 255);
      r |= (uint32_t) lroundf(f + (g - f) * t) << sh;
    }
    return r;
  };
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

// Draws every copy over rgb, W x H of 0xRRGGBB
inline void draw(uint32_t *rgb, int W, int H) {
  uint32_t now = millis();
  for (Overlay &a : overlays) {
    if (a.name.empty())
      continue;
    for (Copy &c : a.copies) {
      if (!c.used)
        continue;
      float v[NCH];
      for (int ch = 0; ch < NCH; ch++)
        v[ch] = shown(c, ch);
      int bw = box_w(a, c), bh = box_h(a, c);
      float mx = v[X] + bw / 2.0f, my = v[Y] + bh / 2.0f;
      for (int ch = TURN; ch < NCH; ch++) {
        const Map &m = c.map[ch];
        if (!m.on || !c.area)
          continue;
        float fx = std::clamp((mx - (float) c.ax0) / (float) (c.ax1 - c.ax0), 0.0f, 1.0f),
              fy = std::clamp((my - (float) c.ay0) / (float) (c.ay1 - c.ay0), 0.0f, 1.0f);
        float along = (float) m.l + (float) (m.r - m.l) * fx, down = (float) m.t + (float) (m.b - m.t) * fy;
        v[ch] = ch == TURN ? v[ch] + along + down : v[ch] * along / 100 * down / 100;
      }
      float op = std::clamp(v[OPACITY], 0.0f, 100.0f) / 100;
      if (c.fade_in > 0 && now - c.placed < (uint32_t) c.fade_in)
        op *= (float) (now - c.placed) / (float) c.fade_in;
      if (c.fading && c.fade_ms > 0)
        op *= 1 - std::min(1.0f, (float) (now - c.fade_t0) / (float) c.fade_ms);
      if (op <= 0.01f)
        continue;
      const std::vector<uint32_t> &px = a.frame(frame_of(a, c));
      float size = std::max(0.1f, v[SIZE] / 100), turn = v[TURN] * 3.14159265f / 180;
      float ct = cosf(turn), st = sinf(turn), hw = bw / 2.0f, hh = bh / 2.0f;
      bool fx = c.flip_x != c.mirrored, fy = c.flip_y;
      int r = (int) ceilf(std::hypot(hw, hh) * size) + 1;
      // each wrap of the screen, once a drift or wander has it going round it
      int wraps = c.torus && (!c.area || c.ex == OVERLAY_EDGE_SCREEN) ? 1 : 0;
      for (int wrap = -wraps; wrap <= wraps; wrap++)
        for (int y = (int) my - r; y <= (int) my + r; y++) {
          if (y < 0 || y >= H)
            continue;
          for (int x = (int) mx - r; x <= (int) mx + r; x++) {
            int sx0 = x + wrap * W;
            if (sx0 < 0 || sx0 >= W)
              continue;
            float dx = ((float) x + 0.5f - mx) / size, dy = ((float) y + 0.5f - my) / size;
            float u = dx * ct + dy * st + hw, w = -dx * st + dy * ct + hh;
            int ix = (int) floorf(u), iy = (int) floorf(w);
            if (ix < 0 || iy < 0 || ix >= bw || iy >= bh)
              continue;
            if (fx)
              ix = bw - 1 - ix;
            if (fy)
              iy = bh - 1 - iy;
            uint32_t p = px[(size_t) iy * a.src.w + ix];
            if (p >> 24 < 128 || hidden(sx0, y, W, c.depth))
              continue;
            if (wrap)
              wrapped[a.name]++;
            p = lit(a, c, p & 0xFFFFFF, v[LIGHT]);
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
        float l = c.base[X], r = l + (float) box_w(a, c);
        w.n++;
        w.x += (l + r) / 2;
        w.edge += l < 16 || r > 320 - 16;
      }
}

// Everything off, for the next picture
inline void reset() {
  for (Overlay &a : overlays)
    a = Overlay();
  sources.clear();
  wrapped.clear();
  crossed.clear();
  where.clear();
  std::fill(std::begin(shared), std::end(shared), 0);
  tags = 0;
}

}  // namespace flexit::panel::overlay
