/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdint>

namespace trmb::gsof {

enum class PositionFix : uint8_t {
  k_none                 = 0,   // No fix or Old Position Fix.
  k_autonomous           = 1,   // Full Measurement Autonomous Stinger Fix
  k_autonomous_prop      = 2,   // Propagated Autonomous Stinger Fix
  k_sbas                 = 3,   // Full Differential SBAS/WAAS Stinger Fix
  k_sbas_prop            = 4,   // Propagated SBAS/WAAS Stinger Fix
  k_dgnss                = 5,   // Full Differential Stinger Fix
  k_dgnss_prop           = 6,   // Propagated Differential Stinger Fix
  k_rtk_float_sync       = 7,   // Float RTK Position Fix at Full Epoch
  k_rtk_float_prop       = 8,   // Propagated Float RTK Position Fix
  k_rtk_fixed_sync       = 9,   // Full Fixed-ambiguity RTK Position Fix
  k_rtk_fixed_prop       = 10,  // Propagated Fixed-ambiguity RTK Fix
  k_omni_hp              = 11,  // Omnistar HP Differential Fix
  k_omni_xp              = 12,  // Omnistar XP Differential Fix
  k_dithered_rtk         = 13,  // RTK Location (Dithered RTK)
  k_omni_vbs             = 14,  // Omnistar VBS Differential
  k_beacon_dgnss         = 15,  // Beacon Differential Solution.
  k_omni_hp_xp           = 16,  // OmniSTAR HPXP solution type.
  k_omni_hp_g_2          = 17,  // OmniSTAR HPG2 solution type.
  k_omni_g_2             = 18,  // OmniSTAR G2 solution type.
  k_rtx_sync             = 19,  // Synchronous RTx solution
  k_rtx_prop             = 20,  // LowLatency RTx solution
  k_omni_ms              = 21,  // OmniSTAR Multiple source solution type
  k_omni_l_1_only        = 22,  // OmniSTAR L1 only solution type
  k_ins_autonomous       = 23,  // INS fix based on Stinger Autonomous fix.
  k_ins_sbas             = 24,  // INS fix based on SBAS-Stinger fix.
  k_ins_dgnss            = 25,  // INS fix based on DGNSS (Code Phase Differential-Stinger fix or Omnistar-VBS
  k_ins_rtx_code         = 26,  // INS fix based on RTX code-phase corrections.
  k_ins_rtx_carrier      = 27,  // 27 INS fix based on RTX carrier-phase corrections.
  k_ins_omni_precise     = 28,  // INS fix based on an Omnistar precision fix (HP XP G2)
  k_ins_rtk              = 29,  // INS fix based on RTK fixed or float.
  k_ins_dead_reckoning   = 30,  // INS fix based on Dead Reckoning.
  k_rtx_code             = 31,  // Fix based on RTX codePhase corrections.
  k_rtx_fast_sync        = 32,  // Fix based on RAIN or SRS in sync mode
  k_rtx_fast_prop        = 33,  // Fix based on RAIN or SRS in low latency mode.
  k_omni_g_3             = 34,
  k_omni_g_4             = 35,
  k_xfill_x              = 36,  // xFillX fix type
  k_rtx_lite_prop        = 37,  // LowLatency RTxLite solution
  k_rtx_lite_sync        = 38,  // Synchronous RTxLite solution
  k_rtx_lite_l_1_prop    = 39,  // LowLatency RTxLiteL1 solution
  k_rtx_lite_l_1_sync    = 40,  // Synchronous RTxLiteL1 solution
  k_rtx_field_point_prop = 41,  // LowLatency RTxFieldPoint solution
  k_rtx_field_point_sync = 42,  // Synchronous RTxFieldPoint solution
  k_omni_g_2_plus        = 43,  // OmniSTAR G2+ solution type
  k_omni_g_4_plus        = 44,  // OmniSTAR G4+ solution type
  k_titan_autonomous     = 45,
  k_titan_sbas           = 46,
  k_titan_dgnss          = 47,
  k_slas_dgnss           = 48,  // L1S SLAS based solution
  k_unknown              = 255
};

PositionFix toPositionFix(uint8_t position_fix);

}  // namespace trmb::gsof
