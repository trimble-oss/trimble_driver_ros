/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/position_fix.h"

namespace trmb::gsof {

PositionFix toPositionFix(uint8_t position_fix) {
  if (position_fix <= static_cast<uint8_t>(PositionFix::k_slas_dgnss)) {
    return static_cast<PositionFix>(position_fix);
  }

  return PositionFix::k_unknown;
}

}  // namespace trmb::gsof
