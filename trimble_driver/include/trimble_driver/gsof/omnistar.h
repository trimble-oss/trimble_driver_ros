/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>

/**
 * This file contains enumerations for interpreting status codes pertaining to OmniSTAR products.
 */

namespace trmb::gsof::omnistar {

enum class HpXpEngine { k_xp = 0, k_hp = 1, k_g2 = 2, k_hp_g2 = 3, k_hp_xp = 4, k_unknown = 255 };

HpXpEngine toHpXpEngine(std::uint8_t flag);

enum class HpXpLibraryMode { k_not_active = 0, k_active = 1 };

HpXpLibraryMode toHpXpLibraryMode(std::uint8_t flag);

enum class VbsLibraryMode { k_not_active = 0, k_active = 1 };

VbsLibraryMode toVbsLibraryMode(std::uint8_t flag);

enum class BeamMode {
  k_off             = 0,
  k_fft_init        = 1,
  k_fft_running     = 2,
  k_search_init     = 3,
  k_search_running  = 4,
  k_track_init      = 5,
  k_track_searching = 6,
  k_tracking        = 7
};

BeamMode toBeamMode(std::uint8_t flag);

enum class MotionState { k_dynamic = 0, k_static = 1, k_omni_star_is_not_ready = 2, k_unknown = 0xFF };

MotionState toMotionState(std::uint8_t flag);

enum class NmeaEncryptionState { k_off = 0, k_on = 1 };

NmeaEncryptionState toNmeaEncryptionState(std::uint8_t flag);

}  // namespace trmb::gsof::omnistar
