/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver/gsof/omnistar.h"

namespace trmb::gsof::omnistar {

HpXpEngine toHpXpEngine(std::uint8_t flag) {
  if (flag <= 4) {
    return static_cast<HpXpEngine>(flag);
  }
  return HpXpEngine::k_unknown;
}

HpXpLibraryMode toHpXpLibraryMode(std::uint8_t flag) {
  return flag ? HpXpLibraryMode::k_active : HpXpLibraryMode::k_not_active;
}

VbsLibraryMode toVbsLibraryMode(std::uint8_t flag) {
  return flag ? VbsLibraryMode::k_active : VbsLibraryMode::k_not_active;
}

BeamMode toBeamMode(std::uint8_t flag) {
  if (flag <= 7) {
    return static_cast<BeamMode>(flag);
  }
  return BeamMode::k_off;
}

MotionState toMotionState(std::uint8_t flag) {
  if (flag <= 2) {
    return static_cast<MotionState>(flag);
  }
  return MotionState::k_unknown;
}

NmeaEncryptionState toNmeaEncryptionState(std::uint8_t flag) {
  return flag ? NmeaEncryptionState::k_on : NmeaEncryptionState::k_off;
}

}  // namespace trmb::gsof::omnistar
