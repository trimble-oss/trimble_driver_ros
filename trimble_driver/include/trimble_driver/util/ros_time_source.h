/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include "trimble_driver/util/wise_enum/wise_enum.h"

namespace util {

enum class RosTimeSource { NOW, GPS, GPS_TIME_OF_WEEK };
static inline RosTimeSource toRosTimeSource(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), [](char c) { return std::tolower(c); });
  if (str == "now") {
    return RosTimeSource::NOW;
  } else if (str == "gps") {
    return RosTimeSource::GPS;
  } else if (str == "gps_time_of_week") {
    return RosTimeSource::GPS_TIME_OF_WEEK;
  }
  return RosTimeSource::NOW;
}

}  // namespace util
