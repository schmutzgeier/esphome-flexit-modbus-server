// Code for packages/quiet_hours.yaml
#pragma once
#include "controls.h"

namespace flexit::quiet_hours {

// Whether minute of the day `now` is in [start, end). A window past midnight has start > end. start == end is
// never.
inline bool in_window(int now, int start, int end) {
  if (start <= end)
    return now >= start && now < end;
  return now >= start || now < end;
}

// every 30 s
template<typename Time, typename Start, typename End> void tick(Time *clock, Start *start, End *end, bool enabled) {
  bool quiet = false;
  auto now = clock->now();
  // Without the time, boosts are allowed: a lost clock shouldn't stop the ventilation from reacting
  if (enabled && now.is_valid() && start->has_state() && end->has_state())
    quiet = in_window(now.hour * 60 + now.minute, start->hour * 60 + start->minute, end->hour * 60 + end->minute);
  if (quiet != controls::quiet) {
    ESP_LOGI("quiet", quiet ? "Quiet hours" : "Quiet hours over");
  }
  controls::quiet = quiet;
}

}  // namespace flexit::quiet_hours
