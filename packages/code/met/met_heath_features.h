// The heath's visitors as features of their own, for "Panel Weather Feature": the reindeer, the walrus, the ravens,
// the sea eagle's catch, and the sheep grazing on the hill. Each fits only the heath, and costs the panel nothing
// beyond the room every cast leaves for a visitor (VISIT): its visits take that room as the heath act's visitors do,
// so never two are here at once. While one of these acts is on, the heath act's own visitors, which take turns, stay
// away (heath_feature_on()). They're never drawn by lot (their lots are 0): in auto the visitors take their turns
// inside "Life on the heath", and a second source of the same visits would only crowd that cast. They're on when
// asked for, whenever they fit, also while everyone's in. The sheep stay for as long as the act does, in the room of
// a visitor, and the picture leaves out its own (L_HEATH_SHEEP).
#pragma once
#include <cstring>
#include "met_live_heath.h"

namespace flexit::panel::met::live {

inline constexpr const char *HEATH_FEATURES[] = {"Reindeer on the heath", "The walrus", "Ravens over the heath",
                                                 "A sea eagle's catch", "Sheep grazing"};
inline bool heath_feature_named(const char *name) {
  for (const char *n : HEATH_FEATURES)
    if (!strcmp(name, n))
      return true;
  return false;
}
// Whether one of them is on, so the heath act's own visitors leave the room to it
inline bool heath_feature_on() {
  for (const On &o : R.cast)
    if (heath_feature_named(o.name))
      return true;
  return false;
}

// What each needs of the picture s, the same as the heath's visitors in visitors()
inline bool heath_walrus_ok(const Scene &s) { return s.at.fog <= 45 && s.heath.bft <= 6; }
inline bool heath_ravens_ok(const Scene &s) {
  return s.at.sun_el > -6 && s.at.fog <= 45 && !s.at.thunder && s.at.gust < 20;
}
inline bool heath_eagle_ok(const Scene &s) {
  return s.at.sun_el > 0 && s.at.light > 0.5f && s.at.wind < 10 && !wet(s.at);
}
// The sheep are out by day and not in snow
inline bool heath_sheep_ok(const Scene &s) { return !s.at.white && s.at.light > 0.5f && !s.at.dark; }
inline float heath_no_lots(const Scene &) { return 0; }

// A visit by fn about every mean ms once the room is free, first after first ms. fn returns the ms it takes, 0 if
// it can't now. Not via_come: fn takes the room itself, as an encounter does.
inline void heath_visits(uint32_t mean, uint32_t first, std::function<uint32_t()> fn, bool via_come = true) {
  sometimes(mean, first, [=] {
    if (!visitor_room())
      return (uint32_t) minutes(0.5f);
    R.guest = VISIT;
    uint32_t ms = via_come ? come(fn, ON_PANEL) : fn();
    if (ms && via_come)  // the reindeer swaps its sprite for another, so the visit holds the room till it's over
      R.busy = millis() + ms;
    return ms ? ms : (uint32_t) minutes(1);
  });
}

inline void heath_deer_feature(const Scene &s) {
  HeathSprite walk = heath_sprite(s, HK_DEER_WALK), graze = heath_sprite(s, HK_DEER_GRAZE);
  Line ground = under(s, 2, walk);
  heath_visits(minutes(1), minutes(0.05f, 0.2f), [=] { return deer_visit(walk, graze, ground); });
}
inline void heath_walrus_feature(const Scene &s) {
  HeathSprite wal = heath_sprite(s, HK_WALRUS);
  heath_visits(minutes(0.5f), minutes(0.05f, 0.2f), [=] { return walrus_swim(wal); });
}
inline void heath_ravens_feature(const Scene &s) {
  int top = H;  // the mountain's top, under the forecast
  for (int x = 60; x < 260; x++)
    top = std::min<int>(top, s.land->hill[0][x]);
  int y = std::max(s.overlay ? OV_H + 6 : 10, top - 26);
  heath_visits(minutes(0.5f), minutes(0.05f, 0.2f), [=] { return ravens_fly(y); });
}
inline void heath_eagle_feature(const Scene &) {
  heath_visits(minutes(0.5f), minutes(0.05f, 0.2f), [] { return heath_eagle(); }, false);
}

// Three sheep on the hill's face, one copy of the sprite grazing about its patch. The picture's own sheep are out
// as long as the act is on. A visitor's room holds them, renewed every 20 s while the act lasts.
inline constexpr int SHEEP_X = 138, SHEEP_Y = 163;
inline void heath_sheep_feature(const Scene &) {
  sometimes(20000, 500, [] {
    if (R.comers.count("sheep")) {  // here: for another minute
      R.comers["sheep"] = millis() + 60000;
      return 0u;
    }
    if (!visitor_room())
      return 0u;
    R.guest = VISIT;
    come(
        [] {
          in_light("sheep");
          place("sheep", 0, SHEEP_X, SHEEP_Y,
                "area x - 4, y - 2, x + 33, y + 4 bounce; wander 0.3, 0.3, 0, 180, 3000, 3000, 6000, 12000",
                {{"x", SHEEP_X}, {"y", SHEEP_Y}}, {.fade = 2000, .depth = D_MID_ON});
          return 60000u;
        },
        0);
    return 0u;
  });
}

}  // namespace flexit::panel::met::live
