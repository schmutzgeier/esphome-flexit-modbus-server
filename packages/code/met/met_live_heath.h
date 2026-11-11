// Animations on the heath: the person on the mountain climbing, running the ridge or skiing, the kayaker paddling
// the bay or the runner on the shore, the dog roaming the hill by the cabin, and now and then a visitor: a reindeer
// out of the trees, a walrus in the bay, a sea eagle's catch on it or two ravens over the mountain by turns, and very
// seldom something rare, see met_live_rare.h. Their frames are met_heath_live.h's. The picture leaves them all out
// (L_HEATH).
#pragma once
#include "met_heath_live.h"
#include "met_live.h"
#include "met_live_rare.h"

namespace flexit::panel::met::live {

// ---- moving a placed copy from here, a message at a time, rather than by a script of its own on the panel

// The waves a copy rocks on are told it as statements: overlay::told(key(n), i, "wave <channel> <low>, <high>, <rise>,
// <fall> [hold <high>, <low>] [jitter <n>] [random] [slow <none|low|high|both>]"). That is the old overlay::wave(key,
// i, what, lo, hi, rise, fall, hold_high, hold_low, jitter, random, slow) with the same numbers, the holds left out
// when both were 0, and the jitter when it was 0. The three it had:
//   the kayak  y -1, 1, 900, 900, jitter 30, both  ->  "wave y -1, 1, 900, 900 jitter 30 slow both"
//   the ravens x -36, 36, 14000, 14000, jitter 20  ->  "wave x -36, 36, 14000, 14000 jitter 20 slow both"
//   the ravens y -7, 7, 9000, 9000, jitter 20      ->  "wave y -7, 7, 9000, 9000 jitter 20 slow both"
inline void move(const std::string &n, int i, int x, int y, int ms, const char *ease = "none") {
  overlay::move(key(n), i, x, y, ms, ease);
}
inline void area(const std::string &n, int i, int l, int t, int r, int b, const char *ex, const char *ey) {
  overlay::area(key(n), i, l, t, r, b, ex, ey);
}
inline void wander(const std::string &n, int i, float s0, float s1, int heading, int spread, int l0, int l1, int r0,
                   int r1) {
  overlay::wander(key(n), i, s0, s1, heading, spread, l0, l1, r0, r1);
}
// it faces the way it goes, mirrored from facing, upright
inline void face(const std::string &n, int i, int facing = 0) { overlay::face(key(n), i, facing, true, 0, 0, true); }
// move() and remove() a while later. remove_later() returns at, for a visit's ms.
inline void move_later(uint32_t at, const char *n, int i, int x, int y, int ms, const char *ease = "none") {
  later(at, [=] { move(n, i, x, y, ms, ease); });
}
inline uint32_t remove_later(uint32_t at, const char *n, int i, int fade = 300) {
  later(at, [=] { remove(n, i, fade); });
  return at;
}

// An amble()'s moves, each to x, y over ms to fn(x, y, ms) in turn. The ms they take. h is a line with at(x).
template<class Line, class F>
uint32_t amble_steps(const Line &h, int x0, int x1, float speed, int mid, int dy, float downhill, int step, F fn) {
  int steps = std::max(1, abs(x1 - x0) / step), px = x0, py = h.at(x0 + mid) - dy;
  uint32_t t = 0;
  for (int k = 1; k <= steps; k++) {
    int x = x0 + (x1 - x0) * k / steps, y = h.at(x + mid) - dy, dx = std::max(1, abs(x - px));
    float v = speed * std::clamp(1 + downhill * (y - py) / dx, 0.6f, 1.5f);
    int ms = (int) (dx * 1000 / v);
    fn(x, y, ms);
    t += ms;
    px = x;
    py = y;
  }
  return t;
}
// Copy i of n from x0 to x1 along the line h, a step every step px or so at speed px/s, and with downhill quicker
// downhill and slower uphill. Its top is dy above the line under its middle, mid px in. The ms it takes.
template<class Line>
uint32_t amble(const char *n, int i, const Line &h, int x0, int x1, float speed, int mid, int dy, float downhill = 0,
               int step = 12) {
  uint32_t t = 0;
  return amble_steps(h, x0, x1, speed, mid, dy, downhill, step, [&](int x, int y, int ms) {
    move_later(t, n, i, x, y, ms);
    t += ms;
  });
}

// A line the characters walk along, every column of it, so they keep to the ground's dips and bumps, but not to a
// single tuft of heather
struct Line {
  uint8_t y[W];
  int at(int x) const { return y[std::clamp(x, 0, W - 1)]; }
};

// Row layer of the land, 0 the mountain's ridge, 1 the land and the shore, 2 the near ground, dy rows lower
inline Line heath_line(const Scene &s, int layer, int dy) {
  const uint8_t *row = s.land->hill[layer];
  Line h;
  for (int x = 0; x < W; x++) {  // the middle of five
    uint8_t v[5];
    for (int k = 0; k < 5; k++)
      v[k] = row[std::clamp(x + k - 2, 0, W - 1)];
    std::nth_element(v, v + 2, v + 5);
    h.y[x] = (uint8_t) std::clamp(v[2] + dy, 0, 255);
  }
  return h;
}
// That line moved so sp stands on it where the art has it. With legs, columns legs0 to legs1 of sp, it stands on
// the highest ground under them, so on a slope its uphill feet are on the ground, not in it.
inline Line under(const Scene &s, int layer, const HeathSprite &sp, int legs0 = -1, int legs1 = -1) {
  Line h = heath_line(s, layer, 0);
  if (legs0 >= 0) {
    Line g = h;
    int mid = sp.frames.w / 2;
    for (int x = 0; x < W; x++)
      for (int c = legs0; c <= legs1; c++)
        h.y[x] = (uint8_t) std::min<int>(h.y[x], g.at(x - mid + c));
  }
  int dy = sp.y + sp.feet - h.at(sp.x + sp.frames.w / 2);
  for (uint8_t &v : h.y)
    v = (uint8_t) std::clamp(v + dy, 0, 255);
  return h;
}

// Its top on line at x
inline int top_at(const Line &line, const HeathSprite &sp, int x) { return line.at(x + sp.frames.w / 2) - sp.feet; }

// Copy 0 of n at x on line, mirrored if flipped, then along it to the other end and back between lo and hi, a
// while there each time
inline void shuttle(const char *n, const HeathSprite &sp, const Line &line, int lo, int hi, float speed, float downhill,
                    bool faces_right, int x, bool first) {
  int to = (int) rnd((float) lo, (float) hi);
  if (abs(to - x) < 15)
    to = x < (lo + hi) / 2 ? hi : lo;
  bool right = to > x;
  place(n, 0, x, top_at(line, sp, x), "", {}, {.left = right != faces_right, .fade = first ? -1 : 0});
  // short steps, so it keeps to the ridge's dips
  uint32_t ms = amble(n, 0, line, x, to, speed, sp.frames.w / 2, sp.feet, downhill, 3);
  later(ms + minutes(0.02f, 0.15f), [=] { shuttle(n, sp, line, lo, hi, speed, downhill, faces_right, to, false); });
}

// ---- the mountain

// The climber goes up the face to the ridge a step at a time, walks along it a while, and starts again
inline void climb(const HeathSprite &sp, const Line &ridge) {
  place("hmount", 0, sp.x, sp.y, "", {}, {.fade = 1500});
  int top = top_at(ridge, sp, sp.x), steps = std::max(1, sp.y - top);
  uint32_t t = 4000;
  for (int i = 1; i <= steps; i++, t += 7000)
    move_later(t, "hmount", 0, sp.x, sp.y - i, 2500);
  int x1 = sp.x + (int) rnd(20, 60);
  later(t, [=] {
    uint32_t walk = amble("hmount", 0, ridge, sp.x, x1, 1.2f, sp.frames.w / 2, sp.feet, 0.5f, 4);
    uint32_t rest = minutes(0.3f, 0.8f);
    remove_later(walk + rest, "hmount", 0, 2000);
    later(walk + rest + minutes(0.3f, 1), [=] { climb(sp, ridge); });
  });
}

inline void mountain(const Scene &s, HeathKind k) {
  HeathSprite sp = heath_sprite(s, k);
  add("hmount", sp.frames);
  Line ridge = heath_line(s, 0, 0);
  if (k == HK_CLIMB)
    climb(sp, ridge);
  else if (k == HK_RUN)  // in the rain it runs the ridge, facing right as drawn
    shuttle("hmount", sp, under(s, 0, sp), 100, 255, 7, 0.4f, true, sp.x, true);
  else  // on skis, quicker downhill
    shuttle("hmount", sp, under(s, 0, sp), 100, 250, 5, 1.5f, true, sp.x, true);
}

// ---- the bay

inline constexpr int KAYAK_WATER = 57;  // just past the bay's water in the kayak's rows, see met_art.h

inline void bay(const Scene &s, HeathKind k) {
  HeathSprite sp = heath_sprite(s, k);
  add("hbay", sp.frames);
  if (k == HK_KAYAK) {  // paddles about the bay, rocking on the waves when there are some
    // Kept to the water: the bay's open from the screen's edge to x 56 in the kayak's rows, and the island is above
    // them. Two rows up or down for the waves and the wander.
    place("hbay", 0, sp.x, sp.y);
    area("hbay", 0, 0, sp.y - 2, KAYAK_WATER, sp.y + sp.frames.h + 2, "bounce", "bounce");
    wander("hbay", 0, 0.8f, 2, 0, 15, 5000, 15000, 2000, 8000);
    face("hbay", 0, 180);
    if (s.heath.bft >= 3)
      overlay::told(key("hbay"), 0, "wave y -1, 1, 900, 900 jitter 30 slow both");
  } else if (k == HK_RUN_SHORE) {  // ashore in waves or a gale, running along it
    shuttle("hbay", sp, under(s, 1, sp), 62, 125, 6, 0, true, sp.x, true);
  } else {
    shuttle("hbay", sp, under(s, 0, sp), 90, 135, 4, 1.5f, true, sp.x, true);
  }
}

// ---- the dog

enum { DOG_SIT, DOG_TROT, DOG_GALLOP, DOG_SNIFF };
inline const char *const DOG_NAMES[] = {"hdogsit", "hdogtrot", "hdoggallop", "hdogsniff"};

inline struct HeathShow {
  HeathSprite dog[4];  // by gait
  Line ground;
  int gait = -1;  // the one it's in
  int dog_x = 0;
  bool dog_right = false;  // facing right, mirrored
  int deer_x = -1;         // where a reindeer is, -1 none
  int turn = 0;            // the visitors take turns, see VISITORS
} HEATH;

// The dog in gait at its x, facing right or left. The gait before goes from the panel, so the dog takes one of
// its animations.
inline void dog_as(int gait, bool right) {
  const HeathSprite &d = HEATH.dog[gait];
  if (HEATH.gait != gait) {
    if (HEATH.gait >= 0) {
      remove(DOG_NAMES[HEATH.gait], 0, 0);
      unload(DOG_NAMES[HEATH.gait]);
    }
    add(DOG_NAMES[gait], d.frames);
  }
  place(DOG_NAMES[gait], 0, HEATH.dog_x, top_at(HEATH.ground, d, HEATH.dog_x), "", {},
        {.left = right, .fade = HEATH.gait < 0 ? -1 : 0});
  HEATH.gait = gait;
  HEATH.dog_right = right;
}

inline void dog_next();

// Along the ground to to in gait, then
inline void dog_go(int gait, int to, float speed, std::function<void()> then) {
  dog_as(gait, to > HEATH.dog_x);
  const HeathSprite &d = HEATH.dog[gait];
  uint32_t ms = amble(DOG_NAMES[gait], 0, HEATH.ground, HEATH.dog_x, to, speed, d.frames.w / 2, d.feet, 0.6f, 3);
  HEATH.dog_x = to;
  later(ms, std::move(then));
}
// Where it is in gait for ms, then
inline void dog_stay(int gait, uint32_t ms, std::function<void()> then) {
  dog_as(gait, HEATH.dog_right);
  later(ms, std::move(then));
}

// What the dog does next: mostly about the hill in front of the cabin, trotting to a new spot, sniffing or sitting
// wagging, now and then a gallop down the slope toward the shore and back, or after a reindeer
inline void dog_next() {
  int home = HEATH.dog[DOG_SIT].x, x = HEATH.dog_x;
  float r = rnd(0, 1);
  if (HEATH.deer_x >= 0 && r < 0.5f) {
    int to = std::clamp(HEATH.deer_x - 14, 150, home + 3);
    dog_go(DOG_GALLOP, to, 24, [home] {
      dog_stay(DOG_SNIFF, minutes(0.04f, 0.1f),
               [home] { dog_go(DOG_TROT, home, 9, [] { dog_stay(DOG_SIT, minutes(0.1f, 0.3f), dog_next); }); });
    });
  } else if (r < 0.15f) {
    dog_go(DOG_GALLOP, (int) rnd(140, 175), 20, [home] {
      dog_stay(DOG_SNIFF, minutes(0.05f, 0.15f),
               [home] { dog_go(DOG_GALLOP, home, 18, [] { dog_stay(DOG_SIT, minutes(0.1f, 0.25f), dog_next); }); });
    });
  } else if (r < 0.55f) {
    int to = (int) rnd((float) (home - 80), (float) (home + 3));
    if (abs(to - x) < 10)
      to = x > home - 40 ? home - 60 : home;
    dog_go(DOG_TROT, to, 9, [] {
      if (chance(0.5f))
        dog_stay(DOG_SNIFF, minutes(0.05f, 0.12f), dog_next);
      else
        dog_next();
    });
  } else if (r < 0.75f) {
    dog_stay(DOG_SNIFF, minutes(0.05f, 0.1f), dog_next);
  } else {
    dog_stay(DOG_SIT, minutes(0.13f, 0.33f), dog_next);
  }
}

inline void the_dog(const Scene &s) {
  for (int g = DOG_SIT; g <= DOG_SNIFF; g++)
    HEATH.dog[g] = heath_sprite(s, (HeathKind) (HK_DOG_SIT + g));
  HEATH.ground = under(s, 2, HEATH.dog[DOG_SIT], 3, 9);  // its legs
  HEATH.dog_x = HEATH.dog[DOG_SIT].x;
  dog_stay(DOG_SIT, minutes(0.05f, 0.15f), dog_next);
}

// ---- the visitors, one at a time

// A reindeer walks out of the trees behind the cabin onto the hill, away from the screen's edge, grazes a while and
// goes back. The ms it takes.
inline uint32_t deer_visit(const HeathSprite &walk, const HeathSprite &graze, const Line &ground) {
  int x0 = walk.x, x1 = (int) rnd(200, 250), mid = walk.frames.w / 2;
  add("hdeerwalk", walk.frames);
  place("hdeerwalk", 0, x0, top_at(ground, walk, x0), "", {}, {.fade = 1500});
  HEATH.deer_x = x0;
  uint32_t out = amble("hdeerwalk", 0, ground, x0, x1, 2, mid, walk.feet, 0, 4), eat = minutes(0.25f, 0.5f);
  later(out, [=] {
    HEATH.deer_x = x1;
    remove("hdeerwalk", 0, 0);
    unload("hdeerwalk");
    add("hdeergraze", graze.frames);
    place("hdeergraze", 0, x1, top_at(ground, graze, x1), "", {}, {.fade = 0});
  });
  later(out + eat, [=] {
    remove("hdeergraze", 0, 0);
    unload("hdeergraze");
    add("hdeerwalk", walk.frames);
    // drawn facing left, it goes back right
    place("hdeerwalk", 0, x1, top_at(ground, walk, x1), "", {}, {.left = true, .fade = 0});
    uint32_t back = amble("hdeerwalk", 0, ground, x1, x0 + 2, 2, mid, walk.feet, 0, 4);
    later(back, [] { HEATH.deer_x = -1; });
    remove_later(back, "hdeerwalk", 0, 1500);
    later(back + 2000, [] { unload("hdeerwalk"); });
  });
  return 2 * out + eat + 3000;
}

// A walrus swims slowly across the back or the front of the bay, coming up and going down. The ms it takes.
inline uint32_t walrus_swim(const HeathSprite &sp) {
  bool back = chance(0.4f), right = chance(0.5f);
  int lo = back ? 14 : 4, hi = back ? 34 : 40, y = (back ? (int) rnd(160, 163) : (int) rnd(190, 200)) - 2;
  int x0 = right ? lo : hi, x1 = right ? hi : lo;
  uint32_t ms = (uint32_t) abs(x1 - x0) * 2500;
  add("hwalrus", sp.frames);
  place("hwalrus", 0, x0, y, "", {}, {.left = right, .fade = 1500, .depth = D_SEA});  // drawn facing left
  move("hwalrus", 0, x1, y, (int) std::min<uint32_t>(ms, 65000));
  remove_later(ms, "hwalrus", 0, 1500);
  later(ms + 2000, [] { unload("hwalrus"); });
  return ms + 3000;
}

// Two ravens come in from off one side over the mountain, circle above its ridge a while and go off the other way,
// along row y. The ms it takes.
inline uint32_t ravens_fly(int y) {
  bool left = chance(0.5f);  // where they come in from
  int x = (int) rnd(90, 200), ms = 15000, stay = (int) minutes(0.3f, 0.6f);
  in_light("ravens");
  place("ravens", 0, left ? -34 : W + 4, y);
  face("ravens", 0);
  overlay::told(key("ravens"), 0, "wave x -36, 36, 14000, 14000 jitter 20 slow both");
  overlay::told(key("ravens"), 0, "wave y -7, 7, 9000, 9000 jitter 20 slow both");
  move("ravens", 0, x, y, ms, "out");
  move_later((uint32_t) (ms + stay), "ravens", 0, left ? W + 40 : -70, y - 12, ms, "in");
  uint32_t all = remove_later((uint32_t) (2 * ms + stay), "ravens", 0);
  later(all + 500, [] { unload("ravens"); });
  return all + 1000;
}

// The visitors by turns: the reindeer, the walrus, the sea eagle's catch and the ravens, each one that can come, and
// now and then something rare in a turn instead, see heath_rare(). The act's cost leaves VISIT for the one that is
// here, so a visit waits while another is, and the cast's own roll for rare encounters leaves the heath to these.
inline constexpr uint32_t VISIT_EVERY = minutes(5);
inline void visitors(const Scene &s) {
  bool walrus = s.at.fog <= 45 && s.heath.bft <= 6, ridge_free = heath_mountain(s) == HK_NONE;
  bool ravens = s.at.sun_el > -6 && s.at.fog <= 45 && !s.at.thunder && s.at.gust < 20;
  HeathSprite walk = heath_sprite(s, HK_DEER_WALK), graze = heath_sprite(s, HK_DEER_GRAZE),
              wal = heath_sprite(s, HK_WALRUS);
  Line ground = under(s, 2, walk);
  int top = H;  // the mountain's top, under the forecast
  for (int x = 60; x < 260; x++)
    top = std::min<int>(top, s.land->hill[0][x]);
  int raven_y = std::max(s.overlay ? OV_H + 6 : 10, top - 26);
  sometimes(VISIT_EVERY, minutes(0.5f, 3), [=] {
    if (!visitor_room())
      return minutes(1);
    R.guest = VISIT;
    if (uint32_t ms = heath_rare(VISIT_EVERY, ridge_free))
      return ms;
    for (int k = 0; k < 4; k++) {
      int v = HEATH.turn++ % 4;
      // the eagle's catch is an encounter and comes by come() itself, see encounter()
      uint32_t ms = v == 2 ? heath_eagle()
                           : come(
                                 [&] {
                                   return v == 0   ? deer_visit(walk, graze, ground)
                                          : v == 1 ? (walrus ? walrus_swim(wal) : 0)
                                                   : (ravens ? ravens_fly(raven_y) : 0);
                                 },
                                 ON_PANEL);
      if (v != 2 && ms)  // the reindeer swaps its sprite for another, so the visit holds the room till it's over
        R.busy = millis() + ms;
      if (ms)
        return ms;
    }
    return minutes(1);
  });
}

// ---- the show

inline void life_on_heath(const Scene &s) {
  HEATH = HeathShow{};
  HeathKind m = heath_mountain(s), b = heath_bay(s);
  if (m != HK_NONE)
    mountain(s, m);
  if (b != HK_NONE)
    bay(s, b);
  if (s.heath.nfig >= 3)
    the_dog(s);
  visitors(s);
}

// What it's made of, so it starts again when that changes: who's out, their colours' light, the headlamp, snow,
// the waves and whether the walrus can come
inline uint32_t heath_cast(const Scene &s) {
  return (uint32_t) heath_mountain(s) | (uint32_t) heath_bay(s) << 4 | (uint32_t) s.heath.lamp << 8 |
         (uint32_t) (s.at.dark + (s.at.light < 0.5f) + (s.at.light <= 0)) << 9 | (uint32_t) s.at.white << 11 |
         (uint32_t) (s.heath.bft >= 3) << 12 | (uint32_t) (s.at.fog <= 45 && s.heath.bft <= 6) << 13 |
         (uint32_t) (s.at.sun_el > -6 && s.at.fog <= 45 && !s.at.thunder && s.at.gust < 20) << 14;
}

}  // namespace flexit::panel::met::live
