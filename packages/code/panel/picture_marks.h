// A picture's marks (marks.h): the soft ones drawn into it, the sharp ones shown over it as a still overlay in front
// of the animations. on() hooks a picture up, set() gives it a new layer.
#pragma once
#include <functional>
#include <map>
#include <memory>
#include <string>
#include "marks.h"
#include "overlay.h"
#include "picture.h"

namespace flexit::panel::marks {

struct Marks {
  std::string name;   // the picture's, and its overlay's with "_marks"
  std::string label;  // its overlay's, for Panel Animation Status
  Layer layer;
  bool into = false;                       // the layer draws into the picture
  std::function<uint32_t(int, int)> base;  // the picture without them, at the screen's size
  std::unique_ptr<Soft> soft;              // while the picture is made
  struct Part {                            // an overlay: the layer's sharp ops, or from Layer::apart on the rest
    std::unique_ptr<Sharp> sharp;          // while it's made
    int ox = 0, oy = 0, ow = 0, oh = 0;    // its place and size, 0 wide for none
  } part[2];
  bool on_panel = false;  // the picture is
};
inline std::map<std::string, Marks> all;  // by picture

inline std::string overlay_of(const std::string &name, int part = 0) { return name + (part ? "_marks2" : "_marks"); }

inline uint32_t soft_pixel(Marks &m, int x, int y) {
  if (x < 0 || y < 0 || x >= W || y >= H)
    return 0;
  if (!m.into)
    return m.base(x, y);
  if (!m.soft)
    m.soft.reset(new (std::nothrow) Soft());
  if (!m.soft)
    return m.base(x, y);
  int y0 = y / BAND * BAND;
  if (m.soft->y0 != y0)
    draw_soft(m.layer, m.base, *m.soft, y0);
  return m.soft->rgb[(y - y0) * W + x];
}

inline uint32_t sharp_pixel(Marks &m, int k, int x, int y) {
  Marks::Part &q = m.part[k];
  int sx = x + q.ox, sy = y + q.oy;
  if (!q.sharp)
    q.sharp.reset(new (std::nothrow) Sharp());
  if (!q.sharp || sx >= W || sy >= H)
    return 0;
  int y0 = q.oy + y / SHARP_BAND * SHARP_BAND;  // from its top, so a pixel at half size has its 2 x 2 in one band
  if (q.sharp->y0 != y0)
    draw_sharp(m.layer, *q.sharp, y0, m.layer.apart > 0 ? k : -1);
  uint32_t p = q.sharp->px[(sy - y0) * W + sx], a = p >> 24;
  if (x == q.ow - 1 && y == q.oh - 1)
    q.sharp.reset();  // the last pixel of a pass, the next starts over
  if (!a)
    return 0;
  uint32_t out = a << 24;
  for (int s = 0; s < 24; s += 8)
    out |= std::min<uint32_t>(255, ((p >> s & 255) * 255 + a / 2) / a) << s;
  return out;
}

// Picture name's marks are drawn into it and shown over it, from set() on, label in Panel Animation Status. After
// picture::add().
inline void on(const std::string &name, const std::string &label = "") {
  auto it = picture::sources.find(name);
  if (it == picture::sources.end())
    return;
  Marks &m = all[name];
  m.name = name;
  m.label = label.empty() ? name + " text" : label;
  picture::Source &s = it->second;
  m.base = s.pixel;
  s.pixel = [&m](int x, int y) { return soft_pixel(m, x, y); };
  auto done = s.done;
  s.done = [&m, done]() {
    m.soft.reset();
    if (done)
      done();
  };
  auto panel = s.on_panel;
  s.on_panel = [&m, panel](bool on) {
    m.on_panel = on;
    for (int i = 0; i < 2; i++) {
      std::string k = overlay_of(m.name, i);
      if (on && m.part[i].ow)
        overlay::show(k);
      else if (!on && overlay::find(k)) {
        overlay::clear(k);
        m.part[i].sharp.reset();
      }
    }
    if (panel)
      panel(on);
  };
}

// Picture name's marks are now l, unless they're the same. What it draws into the picture comes with the picture
// made again, the overlay with it. Without that the overlay changes at once.
inline void set(const std::string &name, Layer l) {
  auto it = all.find(name);
  if (it == all.end())
    return;
  Marks &m = it->second;
  drop(l.boxes);  // they were for laying it out
  if (l == m.layer)
    return;
  for (int i = 0; i < 2; i++)
    if (overlay::find(overlay_of(name, i)))
      overlay::clear(overlay_of(name, i));
  bool was_into = m.into;
  m.layer = std::move(l);
  m.into = into_picture(m.layer);
  m.soft.reset();
  bool shown = false;
  for (int i = 0; i < 2; i++) {
    Marks::Part &q = m.part[i];
    q.sharp.reset();
    q.ow = 0;
    // the overlay covers what its sharp ops do
    Box b{(float) W, (float) H, 0, 0};
    for (const Op &o : m.layer.ops)
      if (!o.soft && m.layer.part_of(o) == i)
        b = {std::min(b.x0, o.box.x0), std::min(b.y0, o.box.y0), std::max(b.x1, o.box.x1), std::max(b.y1, o.box.y1)};
    if (b.x0 >= b.x1)
      continue;
    q.ox = std::max(0, (int) floorf(b.x0));
    q.oy = std::max(0, (int) floorf(b.y0));
    q.ow = std::min(W - q.ox, (int) ceilf(b.x1) - q.ox + 1);
    q.oh = std::min(H - q.oy, (int) ceilf(b.y1) - q.oy + 1);
    overlay::Source s{q.ow, q.oh, 1, 500,
                      q.ox, q.oy, 1, [&m, i](int, int x, int y) { return sharp_pixel(m, i, x, y); }};
    s.ram = sizeof(Sharp);
    s.colours = colours_of(m.layer);
    s.fade_ms = 300;
    s.label = m.label;
    overlay::add(overlay_of(name, i), std::move(s));
    shown = true;
  }
  if (was_into || m.into)
    picture::changed(name);
  else if (m.on_panel && shown)
    for (int i = 0; i < 2; i++)
      if (m.part[i].ow)
        overlay::show(overlay_of(name, i));
}

}  // namespace flexit::panel::marks
