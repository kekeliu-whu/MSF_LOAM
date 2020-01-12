//
// Created by whu on 1/11/20.
//

#ifndef MSF_LOAM_VELODYNE_TIME_H
#define MSF_LOAM_VELODYNE_TIME_H

#include <chrono>

struct UniversalTimeScaleClock {
  using rep = int64_t;
  using period = std::nano;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
  static constexpr int f1 = period::den;
  static constexpr int f2 = std::nano::den / f1;
};

using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

inline Duration FromSeconds(double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

inline double ToSeconds(Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

inline Time FromUniversal(int64_t ticks) { return Time(Duration(ticks)); }

inline int64_t ToUniversal(Time time) {
  return time.time_since_epoch().count();
}

#endif  // MSF_LOAM_VELODYNE_TIME_H
