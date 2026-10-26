// The heath's characters as sprites for its show, met_live_heath.h: the one on the mountain, the one by the bay,
// the dog in its gaits, the reindeer and the walrus, made from met_art.h's figures and met_heath.cpp's bitmaps.
// Plain C++ without ESPHome, see met_scene.h.
#pragma once
#include "met_heath.h"
#include "met_sprites.h"

namespace flexit::panel::met::live {

enum HeathKind : uint8_t {
  HK_CLIMB,  // the mountain: climbing, running the ridge in the rain, skiing in snow
  HK_RUN,
  HK_SKI,
  HK_KAYAK,  // the bay: paddling, running the shore in waves or a gale, skiing in snow
  HK_RUN_SHORE,
  HK_SKI_SHORE,
  HK_DOG_SIT,  // the dog, wagging, trotting, at a gallop, sniffing
  HK_DOG_TROT,
  HK_DOG_GALLOP,
  HK_DOG_SNIFF,
  HK_DEER_WALK,  // the reindeer from the rock carvings
  HK_DEER_GRAZE,
  HK_WALRUS,  // surfacing and diving again
  HK_NONE,
};

// Who's out on the mountain and by the bay, HK_NONE if they're in
HeathKind heath_mountain(const Scene &s);
HeathKind heath_bay(const Scene &s);

// A character's frames in the scene's colours, and where its top left goes so it stands where the art has it.
// feet: rows from its top to just under its feet, on the line it walks along.
struct HeathSprite {
  Frames frames;
  int x, y, feet;
};
HeathSprite heath_sprite(const Scene &s, HeathKind k);

}  // namespace flexit::panel::met::live
