/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

// Contains definitions for LLa<T>, Ned<T>, Rph<T>, Xyz<T>
#include "trimble_driver/gsof/omnistar.h"
#include "trimble_driver/gsof/position_fix.h"
#include "trimble_driver/math.h"

namespace trmb::gsof {

using Id                                             = std::uint8_t;
static constexpr Id GSOF_ID_1_POS_TIME               = 1U;
static constexpr Id GSOF_ID_2_LLH                    = 2U;
static constexpr Id GSOF_ID_3_ECEF                   = 3U;
static constexpr Id GSOF_ID_6_ECEF_DELTA             = 6U;
static constexpr Id GSOF_ID_7_TPLANE_ENU             = 7U;
static constexpr Id GSOF_ID_8_VELOCITY               = 8U;
static constexpr Id GSOF_ID_9_DOP                    = 9U;
static constexpr Id GSOF_ID_10_CLOCK_INFO            = 10U;
static constexpr Id GSOF_ID_11_POS_VCV_INFO          = 11U;
static constexpr Id GSOF_ID_12_POS_SIGMA             = 12U;
static constexpr Id GSOF_ID_15_REC_SERIAL_NUM        = 15U;
static constexpr Id GSOF_ID_16_CURR_TIME             = 16U;
static constexpr Id GSOF_ID_27_ATTITUDE              = 27U;
static constexpr Id GSOF_ID_33_ALL_SV_BRIEF          = 33U;
static constexpr Id GSOF_ID_34_ALL_SV_DETAIL         = 34U;
static constexpr Id GSOF_ID_35_RECEIVED_BASE_INFO    = 35U;
static constexpr Id GSOF_ID_37_BATTERY_MEM_INFO      = 37U;
static constexpr Id GSOF_ID_38_POSITION_TYPE_INFO    = 38U;
static constexpr Id GSOF_ID_40_LBAND_STATUS          = 40U;
static constexpr Id GSOF_ID_41_BASE_POSITION_QUALITY = 41U;
static constexpr Id GSOF_ID_49_INS_FULL_NAV          = 49U;
static constexpr Id GSOF_ID_50_INS_RMS               = 50U;
static constexpr Id GSOF_ID_52_DMI_RAW_DATA          = 52U;
static constexpr Id GSOF_ID_63_INS_FULL_NAV_KRYPTON  = 63U;
static constexpr Id GSOF_ID_64_INS_RMS_KRYPTON       = 64U;

using Mask = std::uint8_t;
static constexpr Mask mask0{0b0000'0001};
static constexpr Mask mask1{0b0000'0010};
static constexpr Mask mask2{0b0000'0100};
static constexpr Mask mask3{0b0000'1000};
static constexpr Mask mask4{0b0001'0000};
static constexpr Mask mask5{0b0010'0000};
static constexpr Mask mask6{0b0100'0000};
static constexpr Mask mask7{0b1000'0000};

/**
 * A lot of the internal classes for message parsing have the void switchEndianess() member function. While it seems
 * like it should be implementing an interface, this would create dynamic objects with a vtable which is not
 * compatible with memcpy for type punning.
 */

#pragma pack(push, 1)
struct Header {
  Id type;
  uint8_t length;

  void switchEndianess() {
    // no-op because all the fields are single byte
  }
};

struct GpsTime {
  uint16_t week;
  uint32_t time_msec;

  void switchEndianess() {
    byteswapInPlace(&week);
    byteswapInPlace(&time_msec);
  }
};

struct Status {
  uint8_t imu_alignment;
  uint8_t gnss;

  void switchEndianess() {
    // no-op because all the fields are single byte
  }

  enum class ImuAlignmentStatus { GPS_ONLY, COARSE_LEVELING, DEGRADED, ALIGNED, FULL_NAV, UNKNOWN };

  [[nodiscard]] ImuAlignmentStatus getImuAlignmentStatus() const {
    switch (imu_alignment) {
      case 0:
        return ImuAlignmentStatus::GPS_ONLY;
      case 1:
        return ImuAlignmentStatus::COARSE_LEVELING;
      case 2:
        return ImuAlignmentStatus::DEGRADED;
      case 3:
        return ImuAlignmentStatus::ALIGNED;
      case 4:
        return ImuAlignmentStatus::FULL_NAV;
      default:
        return ImuAlignmentStatus::UNKNOWN;
    }
  }

  enum class GnssStatus {
    FIX_NOT_AVAILABLE,
    GNSS_SPS_MODE,
    DIFFERENTIAL_GPS_SPS,
    GPS_PPS_MODE,
    FIXED_RTK_MODE,
    FLOAT_RTK,
    DIRECT_GEOREFERENCING_MODE,
    UNKNOWN
  };

  [[nodiscard]] GnssStatus getGnssStatus() const {
    switch (gnss) {
      case 0:
        return GnssStatus::FIX_NOT_AVAILABLE;
      case 1:
        return GnssStatus::GNSS_SPS_MODE;
      case 2:
        return GnssStatus::DIFFERENTIAL_GPS_SPS;
      case 3:
        return GnssStatus::GPS_PPS_MODE;
      case 4:
        return GnssStatus::FIXED_RTK_MODE;
      case 5:
        return GnssStatus::FLOAT_RTK;
      case 6:
        return GnssStatus::DIRECT_GEOREFERENCING_MODE;
      default:
        return GnssStatus::UNKNOWN;
    }
  }
};

struct Mode {
  uint8_t calc_mode;

  void switchEndianess() {
    // no-op because all the fields are single byte
  }

  enum class CalcMode { NONE, AUTONOMOUS, RTK_FLOAT, RTK_FIX, DGPS, UNKNOWN };

  [[nodiscard]] CalcMode getCalcMode() const {
    switch (calc_mode) {
      case 0:
        return CalcMode::NONE;
      case 1:
        return CalcMode::AUTONOMOUS;
      case 2:
        return CalcMode::RTK_FLOAT;
      case 3:
        return CalcMode::RTK_FIX;
      case 4:
        return CalcMode::DGPS;
      default:
        return CalcMode::UNKNOWN;
    }
  }
};

struct VnavStatus {
  uint8_t imu_alignment;
  uint8_t gps_quality;

  void switchEndianess() {
    // no-op because all the fields are single byte
  }

  enum class ImuAlignmentStatus { GPS_ONLY, COARSE_LEVELING, DEGRADED, ALIGNED, FULL_NAV, UNKNOWN };

  [[nodiscard]] ImuAlignmentStatus getImuAlignmentStatus() const {
    switch (imu_alignment) {
      case 0:
        return ImuAlignmentStatus::GPS_ONLY;
      case 1:
        return ImuAlignmentStatus::COARSE_LEVELING;
      case 2:
        return ImuAlignmentStatus::DEGRADED;
      case 3:
        return ImuAlignmentStatus::ALIGNED;
      case 4:
        return ImuAlignmentStatus::FULL_NAV;
      default:
        return ImuAlignmentStatus::UNKNOWN;
    }
  }

  enum class GpsQuality {
    FIX_NOT_AVAILABLE,
    GPS_SPS_MODE,
    DIFFERENTIAL_GPS_SPS,
    GPS_PPS_MODE,
    FIXED_RTK_MODE,
    FLOAT_RTK,
    ESTIMATED_MODE,
    MANUAL_INPUT_MODE,
    SIMULATOR_MODE,
    UNKNOWN
  };

  [[nodiscard]] GpsQuality getGpsQuality() const {
    switch (gps_quality) {
      case 0:
        return GpsQuality::FIX_NOT_AVAILABLE;
      case 1:
        return GpsQuality::GPS_SPS_MODE;
      case 2:
        return GpsQuality::DIFFERENTIAL_GPS_SPS;
      case 3:
        return GpsQuality::GPS_PPS_MODE;
      case 4:
        return GpsQuality::FIXED_RTK_MODE;
      case 5:
        return GpsQuality::FLOAT_RTK;
      case 6:
        return GpsQuality::ESTIMATED_MODE;
      case 7:
        return GpsQuality::MANUAL_INPUT_MODE;
      case 8:
        return GpsQuality::SIMULATOR_MODE;
      default:
        return GpsQuality::UNKNOWN;
    }
  }
};

enum class SatelliteType {
  GPS      = 0,
  SBAS     = 1,
  GLONASS  = 2,
  GALILEO  = 3,
  QZSS     = 4,
  BEIDOU   = 5,
  IRNSS    = 6,
  OMNISTAR = 10,
  UNKNOWN
};

SatelliteType toSatelliteType(int sv_system);

struct SVBriefInfo {
  uint8_t prn;
  uint8_t sv_system;
  uint8_t sv_flags1;
  uint8_t sv_flags2;

  void switchEndianess() {
    // no-op because all the fields are single byte
  }

  [[nodiscard]] SatelliteType getSVSystemMode() const { return toSatelliteType(sv_system); }

  /* SV FLAGS1 indicate conditions relating to satellites.
   * bit 0 set: Above horizon
   * bit 1 set: Currently assigned to a channel (trying to track)
   * bit 2 set: Currently tracked on L1/G1 frequency
   * bit 3-7: RESERVED
   * SV FLAGS2 indicate conditions relating to satellites.
   * bits 0-7: RESERVED
   */
  [[nodiscard]] bool isAboveHor() const { return sv_flags1 & mask0; }
  [[nodiscard]] bool isAssignedToChannel() const { return sv_flags1 & mask1; }
  [[nodiscard]] bool isTracked() const { return sv_flags1 & mask2; }
};

struct SVDetailedInfo {
  uint8_t prn;
  uint8_t sv_system;
  uint8_t sv_flags1;
  uint8_t sv_flags2;
  uint8_t elevation;
  uint16_t azimuth;
  uint8_t snr_L1;
  uint8_t snr_L2;
  uint8_t snr_L5;

  void switchEndianess() { byteswapInPlace(&azimuth); }

  [[nodiscard]] SatelliteType getSvType() const { return toSatelliteType(sv_system); }

  /* SV FLAGS1 is a bitmap field having the following values:
   *    bit 0 Set: Above horizon
   *    bit 1 Set: Currently assigned to a channel (trying to track)
   *    bit 2 Set: Currently tracked on single frequency band
   *    bit 3 Set: Currently tracked on dual frequency band
   *    bit 4 Set: Reported at base on L1/G1 frequency
   *    bit 5 Set: Reported at base on L2/G2 frequency
   *    bit 6 Set: Used in current position
   *    bit 7 Set: Used in the current RTK solution.
   *
   * SV FLAGS2 is a bitmap variable having the following values:
   *    bit 0 Set: Tracking P-Code on L1/G1
   *    bit 1 Set: Tracking P-Code on L2
   *  IF GPS SV
   *    bit 2 Set: Tracking CS on L2
   *    bit 3 Set: Tracking L5 Signal
   *    bit 4 Set: Tracking L1C
   *    Bits 5-7 are reserved
   * If GLONASS SV
   *    bit 2 Set: Glonass SV is "M" SV
   *    bit 3 Set: Glonass SV is "K" SV
   *    Bits 4-7 are reserved
   * ELSE
   *    Bits 2-7 are reserved
   */
  [[nodiscard]] bool isAboveHor() const { return sv_flags1 & mask0; }
  [[nodiscard]] bool isAssignedToChannel() const { return sv_flags1 & mask1; }
  [[nodiscard]] bool isTracked() const { return sv_flags1 & mask2; }
  [[nodiscard]] bool isCurrTrackedDual() const { return sv_flags1 & mask3; }
  [[nodiscard]] bool isL1G1Freq() const { return sv_flags1 & mask4; }
  [[nodiscard]] bool isL1G2Freq() const { return sv_flags1 & mask5; }
  [[nodiscard]] bool isUsedAtCurrentPos() const { return sv_flags1 & mask6; }
  [[nodiscard]] bool isUsedInRtkSolution() const { return sv_flags1 & mask7; }

  [[nodiscard]] bool isTrackPCodeL1G1() const { return sv_flags2 & mask0; }
  [[nodiscard]] bool isTrackPCodeL2() const { return sv_flags2 & mask1; }

  [[nodiscard]] bool isTrackingCsOnL2() const { return (getSvType() == SatelliteType::GPS) && sv_flags2 & mask2; }

  [[nodiscard]] bool isTrackingL5Signal() const { return (getSvType() == SatelliteType::GPS) && sv_flags2 & mask3; }

  [[nodiscard]] bool isTrackingL1C() const { return (getSvType() == SatelliteType::GPS) && sv_flags2 & mask4; }

  /**
   * GLONASS-M is the second generation of GLONASS satellites
   * @return
   */
  [[nodiscard]] bool isGlonassMSv() const { return (getSvType() == SatelliteType::GLONASS) && sv_flags2 & mask2; }

  /**
   * GLONASS-K is the third generation of GLONASS satellites
   * @return
   */
  [[nodiscard]] bool isGlonassKSv() const { return (getSvType() == SatelliteType::GLONASS) && sv_flags2 & mask3; }
};

/* GSOF #1: Position Time Info
 *
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (long) GPS TIME (ms)
 * 2 (int) GPS WEEK NUMBER
 * 1 (byte) NUMBER OF SVS USED
 * 1 (byte) POSITION FLAGS 1
 * bit 0 SET: New Position
 * bit 1 SET: Clock fix calculated this position
 * bit 2 SET: Horizontal coordinates calculated this position
 * bit 3 SET: Height calculated this position
 * bit 4 reserved: Always SET (was “Weighted position”)
 * bit 5 SET: Least squares position
 * bit 6 reserved: Always CLEAR (was “Iono-free position”)
 * bit 7 SET: Position uses Filtered L1 pseudoranges
 *
 * 1 (byte) POSITION FLAGS 2
 * bit 0 SET: Position is a differential solution.
 *            RESET: Position is autonomous or WAAS solution.
 * bit 1 SET: Differential position is phase including RTK (float, fixed or location), RTX, xFill, HP or XP Omnistar
 *            (VBS is not derived from phase).
 *            RESET: Differential position is code.
 * bit 2 SET: Differential position is fixed integer phase position (RTK-fixed). Uncorrected position is WAAS (if
 *            bit 0 is 0).
 *            RESET: Differential position is RTK-float, RTK-location or code phase (DGPS), Uncorrected position is
 *            Autonomous (if bit 0 is 0).
 * bit 3 SET: OmniSTAR differential solution (including HP, XP, and VBS.)
 *            RESET: Not OmniSTAR solution
 * bit 4 SET: Position determined with STATIC as a constraint
 * bit 5 SET: Position is Network RTK solution
 * bit 6 SET: RTK-Location
 * bit 7 SET: Beacon DGPS
 *
 * 1 (byte) INITIALIZATION NUMBER
 */
struct PositionTimeInfo {
  Header header;
  // We do not use GpsTime here because for some reason, this is the only record that has the gps week after the week
  // time
  uint32_t gps_time_ms;
  uint16_t gps_week;
  uint8_t number_space_vehicles_used;
  uint8_t position_flags_1;
  uint8_t position_flags_2;
  uint8_t init_num;

  void switchEndianess() {
    byteswapInPlace(&gps_time_ms);
    byteswapInPlace(&gps_week);
  }

  [[nodiscard]] bool isNewPos() const { return position_flags_1 & mask0; }
  [[nodiscard]] bool isClockFix() const { return position_flags_1 & mask1; }
  [[nodiscard]] bool isHCoordinatesComputedHere() const { return position_flags_1 & mask2; }
  [[nodiscard]] bool isHeightComputedHere() const { return position_flags_1 & mask3; }
  [[nodiscard]] bool isLeastSquares() const { return position_flags_1 & mask5; }
  [[nodiscard]] bool isL1PseudoRangeUsed() const { return position_flags_1 & mask7; }

  [[nodiscard]] bool isDiffSoln() const { return position_flags_2 & mask0; }
  [[nodiscard]] bool isDiffPosInPhase() const { return position_flags_2 & mask1; }
  [[nodiscard]] bool isDiffPosFixedInt() const { return position_flags_2 & mask2; }
  [[nodiscard]] bool isOmnistarSoln() const { return position_flags_2 & mask3; }
  [[nodiscard]] bool isStaticConstr() const { return position_flags_2 & mask4; }
  [[nodiscard]] bool isNetworkRtkSoln() const { return position_flags_2 & mask5; }
  [[nodiscard]] bool isRtkLocation() const { return position_flags_2 & mask6; }
  [[nodiscard]] bool isBeaconDGPS() const { return position_flags_2 & mask7; }
};

static_assert(sizeof(PositionTimeInfo) == 12);

/* GSOF #2: Lat, Long, Height
 *
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 8 (double) LATITUDE (WGS84 LAT in rad)
 * 8 (double) LONGITUDE (WGS84 LONG in rad)
 * 8 (double) HEIGHT (WGS84 HEIGHT in m)
 */
struct LatLongHeight {
  Header header;
  Llad lla;
  void switchEndianess() { lla.switchEndianess(); }
};

static_assert(sizeof(LatLongHeight) == 26);

/* GSOF #3: ECEF Position
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 8 (double) X
 * 8 (double) Y
 * 8 (double) Z
 */

struct EcefPosition {
  Header header;
  Xyzd pos;
  void switchEndianess() { pos.switchEndianess(); }
};

static_assert(sizeof(EcefPosition) == 26);

/* GSOF #6: ECEF Delta
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 8 (double) DELTA X
 * 8 (double) DELTA Y
 * 8 (double) DELTA Z
 */

struct EcefDelta {
  Header header;
  Xyzd delta;
  void switchEndianess() { delta.switchEndianess(); }
};

static_assert(sizeof(EcefDelta) == 26);

/* GSOF #7: Tangent Plane Delta
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 8 (double) DELTA EAST
 * 8 (double) DELTA NORTH
 * 8 (double) DELTA UP
 */

struct TangentPlaneDelta {
  Header header;
  Enud enu;

  void switchEndianess() { enu.switchEndianess(); }
};

static_assert(sizeof(TangentPlaneDelta) == 26);

/* GSOF #8: Velocity Data
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 1 (byte) VELOCITY FLAGS
 * bit 0 SET: Velocity data valid. RESET: Velocity data not valid
 * bit 1 SET: Velocity computed from consecutive measurements. RESET: Velocity computed from Doppler
 * bit 2 SET: Heading data valid. RESET: Heading data not valid. This is added in firmware version 5.50 0.50
 * bits 3-7: RESERVED
 *
 * 4 (float) VELOCITY is the horizontal velocity in meters per second
 * 4 (float) HEADING is the WGS84 referenced true north heading in radians
 * 4 (float) VERTICAL VELOCITY is the velocity in the vertical direction in meters per second.
 * 4 (float) LOCAL HEADING is the local coordinate referenced coordinate system north heading in radians. It is only
 *           present in the message if a planar local coordinate system is loaded.
 */

struct Velocity {
  Header header;
  uint8_t velocity_flags;
  float velocity;
  float heading;
  float vertical_velocity;
  std::optional<float> local_heading = std::nullopt;

  void switchEndianess() {
    byteswapInPlace(&velocity);
    byteswapInPlace(&heading);
    byteswapInPlace(&vertical_velocity);

    if (local_heading.has_value()) {
      byteswapInPlace(&(*local_heading));
    }
  }

  [[nodiscard]] bool isVelDataValid() const { return velocity_flags & mask0; }

  enum class VelocitySource { k_consecutive_measurements = 0, k_doppler = 1 };

  [[nodiscard]] VelocitySource getVelocitySource() const {
    if ((velocity_flags & mask1) == 0) {
      return VelocitySource::k_consecutive_measurements;
    } else {
      return VelocitySource::k_doppler;
    }
  }

  [[nodiscard]] bool isHeadingDataValid() const { return velocity_flags & mask2; }
};

/* GSOF #9: PDOP Info (Dilution of Precision)
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (float) PDOP is the positional dilution of precision
 * 4 (float) HDOP is the horizontal dilution of precision
 * 4 (float) VDOP is the vertical dilution of precision
 * 4 (float) TDOP is the time dilution of precision
 * Notes:
 * When an RTK system is placed in the Static (measuring) mode, these values become Relative DOP values, and as such
 * tend to diminish with elapsed time spend static.
 */
struct PdopInfo {
  Header header;
  float position_dop;
  float horiziontal_dop;
  float vertical_dop;
  float time_dop;

  void switchEndianess() {
    byteswapInPlace(&position_dop);
    byteswapInPlace(&horiziontal_dop);
    byteswapInPlace(&vertical_dop);
    byteswapInPlace(&time_dop);
  }
};

static_assert(sizeof(PdopInfo) == 18);

/* GSOF #10: Clock Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 1 (byte) CLOCK FLAGS
 * bit 0 SET: Clock offset is valid
 * bit 1 SET: Frequency offset is valid
 * bit 2 SET: Receiver is in anywhere fix mode
 * bit 3-7: RESERVED
 *
 * 8 (double) CLOCK OFFSET (ms)
 * 8 (double) FREQUENCY OFFSET (ppm)
 */

struct ClockInfo {
  Header header;
  uint8_t clock_flags;
  double clock_offset;
  double freq_offset;

  void switchEndianess() {
    byteswapInPlace(&clock_offset);
    byteswapInPlace(&freq_offset);
  }

  [[nodiscard]] bool isClockOffsetValid() const { return clock_flags & mask0; }
  [[nodiscard]] bool isFreqOffsetValid() const { return clock_flags & mask1; }
  [[nodiscard]] bool isReceiverInAnywhereFixMode() const { return clock_flags & mask2; }
};

static_assert(sizeof(ClockInfo) == 19);

/* GSOF #11: Position VCV Info (variance-covariance matrix)
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (float) POSITION RMS is the square root of (the sum of the squares of the range residuals divided by the number of
 *           degrees of freedom in the solution).
 *
 * VCVxx .. VCVzz is the variance-covariance matrix. This contains the positional components of the inverted normal
 * matrix of the position solution in a ECEF WGS84 reference. Special notes: For INS mode, the user point covariances
 * are output in GSOF 11. For TitanAINS with Survey-Pole dynamic model, developers should use GSOF 58 and this GSOF 11
 * should not be used.
 * VCV matrix is symmetric. Thus, VCV yx = VCV xy, VCV zx = VCV xz, VCV zy = VCV yz
 *
 * 4 (float) VCV xx
 * 4 (float) VCV xy
 * 4 (float) VCV xz
 * 4 (float) VCV yy
 * 4 (float) VCV yz
 * 4 (float) VCV zz
 * 4 (float) UNIT VARIANCE is the unit variance of the position solution.
 * 2 (short) NUMBER OF EPOCHS indicates the number of measurements used to compute the position. It may be greater than
 *           1 for positions subjected to a STATIC constraint.
 */

struct PositionVcvInfo {
  Header header;
  float position_rms;
  float xx;
  float xy;
  float xz;
  float yy;
  float yz;
  float zz;
  float unit_var;
  uint16_t num_epochs;

  void switchEndianess() {
    byteswapInPlace(&position_rms);
    byteswapInPlace(&xx);
    byteswapInPlace(&xy);
    byteswapInPlace(&xz);
    byteswapInPlace(&yy);
    byteswapInPlace(&yz);
    byteswapInPlace(&zz);
    byteswapInPlace(&unit_var);
    byteswapInPlace(&num_epochs);
  }
};

static_assert(sizeof(PositionVcvInfo) == 36);

/* GSOF #12: POSITION SIGMA INFO
 * 1 (byte) OUTPUT RECORD TYPE = 12
 * 1 (byte) RECORD LENGTH
 * 4 (float) POSITION RMS
 * 4 (float) SIGMA EAST [m]
 * 4 (float) SIGMA NORTH [m]
 * 4 (float) COVAR. EAST-NORTH
 * 4 (float) SIGMA UP [m]
 * 4 (float) SEMI MAJOR AXIS [m]
 * 4 (float) SEMI-MINOR AXIS [m]
 * 4 (float) ORIENTATION of the semi-major axis is in degrees from clockwise from True North [deg]
 *           Special notes: For INS mode, the user point SIGMAs and error ellipse info are output in this GSOF 12.
 *           For TitanAINS with Survey-Pole dynamic model, developers should use GSOF 58 and this GSOF 12 should not be
 *           used.
 * 4 (float) UNIT VARIANCE is valid only for over determined solutions. It should tend towards 1.0. A value less than
 *           1.0 indicates that the apriori variances were too pessimistic.
 * 2 (short) NUMBER EPOCHS indicates the number of measurements used to compute the position. It may be greater than 1
 *           for positions subjected to a STATIC constraint.
 */
struct PositionSigmaInfo {
  Header header;
  float position_rms;
  float sigma_east;
  float sigma_north;
  float covariance_east_north;
  float sigma_up;
  float semi_major_axis;
  float semi_minor_axis;
  float orientation;
  float unit_variance;
  uint16_t number_epochs;

  void switchEndianess() {
    byteswapInPlace(&position_rms);
    byteswapInPlace(&sigma_east);
    byteswapInPlace(&sigma_north);
    byteswapInPlace(&covariance_east_north);
    byteswapInPlace(&sigma_up);
    byteswapInPlace(&semi_major_axis);
    byteswapInPlace(&semi_minor_axis);
    byteswapInPlace(&orientation);
    byteswapInPlace(&unit_variance);
    byteswapInPlace(&number_epochs);
  }
};

static_assert(sizeof(PositionSigmaInfo) == 40);

/* GSOF #15: Receiver Serial Number
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (long) SERIAL NUMBER
 */
struct ReceiverSerialNumber {
  Header header;
  int32_t number;

  void switchEndianess() { byteswapInPlace(&number); }
};

static_assert(sizeof(ReceiverSerialNumber) == 6);

/* GSOF #16: Current Time
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (long) GPS MILLISEC OF WEEK
 * 2 (short) GPS WEEK NUMBER
 * 2 (short) UTC OFFSET
 * 1 (byte) FLAGS
 * bit 0 SET: Time information (week and milliseconds of week) valid
 * bit 1 SET: UTC Offset is valid
 */

struct CurrentTime {
  Header header;
  uint32_t gps_ms_week;
  uint16_t gps_week;
  uint16_t utc_offset;
  uint8_t flags;

  void switchEndianess() {
    byteswapInPlace(&gps_ms_week);
    byteswapInPlace(&gps_week);
    byteswapInPlace(&utc_offset);
  }

  [[nodiscard]] bool isTimeInfoValid() const { return flags & mask0; }
  [[nodiscard]] bool isUtcOffsetValid() const { return flags & mask1; }
};

static_assert(sizeof(CurrentTime) == 11);

/* GSOF #27: Attitude Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (unsigned long) GPS TIME is time of position in milliseconds of GPS week.
 * 1 (byte) FLAGS
 *    Bit 0: Calibrated
 *    Bit 1: Pitch Valid
 *    Bit 2: Yaw Valid
 *    Bit 3: Roll Valid
 *    Bit 4: Scalar Valid
 *    For non-COBRA system: Bit 5 – Bit 7 Reserved
 *    For COBRA system:
 *    Bit 5 - Diagnostic Valid
 *    Bit 6 - Slave Static
 *    Bit 7 - Error Stats valid
 *
 * 1 (byte) NUMBER OF SVS
 * 1 (byte) CALCULATION MODE
 * 1 (byte) RESERVED
 * 8 (double) PITCH [rad]
 * 8 (double) YAW [rad]
 * 8 (double) ROLL [rad]
 * 8 (double) MASTER-SLAVE RANGE [m]
 * 2 (word) PDOP
 * NOT IMPLEMENTED in firmware versions prior to GNSS v4.20
 * 4 (float) PITCH VARIANCE [rad^2]
 * 4 (float) YAW VARIANCE [rad^2]
 * 4 (float) ROLL VARIANCE [rad^2]
 * 4 (float) PITCH-YAW COVARIANCE [rad^2]
 * 4 (float) PITCH-ROLL COVARIANCE [rad^2]
 * 4 (float) YAW-ROLL COVARIANCE [rad^2]
 * 4 (float) MASTER-SLAVE RANGE VARIANCE [m^2]
 */

struct AttitudeInfo {
  Header header;
  uint32_t gps_time;
  uint8_t flags;
  uint8_t num_svs;
  Mode calc_mode;
  uint8_t reserved;
  Pyrd pyr;
  double master_slave_range;
  uint16_t pdop;

  struct Variance {
    float pitch;
    float yaw;
    float roll;
    float pitch_yaw;
    float pitch_roll;
    float yaw_roll;
    float master_slave_range;

    void switchEndianess() {
      byteswapInPlace(&pitch);
      byteswapInPlace(&yaw);
      byteswapInPlace(&roll);
      byteswapInPlace(&pitch_yaw);
      byteswapInPlace(&pitch_roll);
      byteswapInPlace(&yaw_roll);
      byteswapInPlace(&master_slave_range);
    }
  };

  std::optional<Variance> variance = std::nullopt;

  void switchEndianess() {
    byteswapInPlace(&gps_time);
    pyr.switchEndianess();
    byteswapInPlace(&master_slave_range);
    byteswapInPlace(&pdop);
    if (variance.has_value()) {
      variance->switchEndianess();
    }
  }

  [[nodiscard]] bool isCalibrated() const { return flags & mask0; }
  [[nodiscard]] bool isPitchValid() const { return flags & mask1; }
  [[nodiscard]] bool isYawValid() const { return flags & mask2; }
  [[nodiscard]] bool isRollValid() const { return flags & mask3; }
  [[nodiscard]] bool isScalarValid() const { return flags & mask4; }
  // COBRA system only
  [[nodiscard]] bool isDiagValid() const { return flags & mask5; }
  [[nodiscard]] bool isSlaveStatic() const { return flags & mask6; }
  [[nodiscard]] bool isErrValid() const { return flags & mask7; }

  [[nodiscard]] float getTruePdop() const {
    constexpr float k_pdop_scale_factor = 0.1f;
    return pdop * k_pdop_scale_factor;
  }
};

/* GSOF #33: All SV Brief Info (External only)
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 1 (byte) NUMBER OF SVS
 * repeated for number of SVs
 * 1 (byte) PRN
 * 1 (byte) SV System
 * 1 (byte) SV FLAGS1
 * 1 (byte) SV FLAGS2
 */
struct AllSvBrief {
  Header header;
  uint8_t num_svs;
  std::vector<SVBriefInfo> sv_info;

  void switchEndianess() {
    for (auto &info : sv_info) {
      info.switchEndianess();
    }
  }
};

/* GSOF #34: All SV Detailed Info (External only, limited to 24 SVs)
 * 1 (byte) OUTPUT RECORD TYPE = 34
 * 1 (byte) RECORD LENGTH
 * 1 (byte) NUMBER OF SVS
 * repeated for number of SVs
 * 1 (byte) PRN
 * 1 (byte) SV SYSTEM
 * 1 (byte) SV FLAGS1
 * 1 (byte) SV FLAGS2
 * 1 (signed byte) ELEVATION
 * 2 (short) AZIMUTH
 * 1 (byte) SNR L1*4
 * 1 (byte) SNR L2*4
 * 1 (byte) SNR L5*4
 */
struct AllSvDetailed {
  Header header;
  uint8_t num_svs;
  std::vector<SVDetailedInfo> sv_info;
  void switchEndianess() {
    for (auto &info : sv_info) {
      info.switchEndianess();
    }
  }
};

/* GSOF #35: Received Base Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 1 (Byte) FLAGS and VERSION OF MESSAGE
 * 8 (chars) BASE NAME
 * 2 (bytes) BASE ID
 * 8 (double) BASE LATITUDE
 * 8 (double) BASE LONGITUDE
 * 8 (double) BASE HEIGHT
 */
struct ReceivedBaseInfo {
  Header header;
  uint8_t flags;
  std::array<char, 8> name;
  uint16_t id;
  Llad base_lla;

  void switchEndianess() {
    byteswapInPlace(&id);
    base_lla.switchEndianess();
  }

  /* FLAGS specifies a few attributes about the BASE. Defined values:
   * Bits 0 - 2: specify a "version number" for this message
   * Bit 3: if SET specifies that the base info given is valid
   * Bits 4 - 7: RESERVED
   */

  [[nodiscard]] uint8_t getVersionNumber() const { return flags & 0b0000'0111; }
  [[nodiscard]] bool isBaseInfoValid() const { return flags & mask3; }
};

/* GSOF #37: Battery Memory Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 2 (short) BATTERY CAPACITY
 * 8 (double) REMAINING DATA LOGGING TIME
 */
struct BatteryMemoryInfo {
  Header header;
  uint16_t battery_cap;
  double remaining_time;
  void switchEndianess() {
    byteswapInPlace(&battery_cap);
    byteswapInPlace(&remaining_time);
  }
};

enum class SolutionIntegrity {
  k_not_checking,
  k_checking_initialization,
  k_initialization_passed,
  k_initialization_failed,
  k_unknown
};

SolutionIntegrity toSolutionIntegrity(uint8_t value);

enum class RtkCondition {
  k_new_position_computed,
  k_unable_to_obtain_synced_pair_from_both_stations,
  k_inssufficient_double_difference_measurements,
  k_reference_position_unavailable,
  k_failed_integer_verification_with_fixed_solution,
  k_solution_rms_over_limit,  // or pole is wobbling if static
  k_pdop_or_rdop_exceeds_mask,
  k_unknown
};

RtkCondition toRtkCondition(uint8_t rtk_condition);

enum class RtcmStatus {
  k_not_available_or_unknown,
  k_collecting_messages,  // And not received complete cycle yet
  k_collection_complete,  // But data insufficient for RTK network solution
  k_working               // RTCM v3 network RTK message collection completed and VRS observations epochs
  // generated from V3 network messages. I.e. V3 network is up and running and in good
  // shape.
};

RtcmStatus toRtcmStatus(uint8_t network_flags);

/* GSOF#38: Position Type Information
 * 1 (byte) OUTPUT RECORD TYPE = 38
 * 1 (byte) RECORD LENGTH
 * 4 (float) ERROR SCALE  is the output precisions are scaled by a factor of 1.5 which is an
 *                        empirical constant used to produce approximately a 99% confidence level or
 *                        3-sigma. This field is believed to be the same as 'ERRORSCALE' in
 *                        31h_RETPOS3 which is described as "an “a posteriori” measurement sigma
 *                        which provides error estimation of the position accuracy. This value
 *                        must be multiplied by the DOP factors to obtain error estimate in
 *                        meters". See Precision Calculation
 * (new in fw 4.40)
 *    1 (byte) SOLUTION FLAGS is a bitmap as follows:
 *                            bit 0: Solution is Wide Area / Network / VRS (same as 31h_RETPOS3 field 'POSITION
 *                                   TYPE' value 7.
 *                            bit 1: RTK fix solution (clear: RTK float solution) (same as 31h_RETPOS3 field
 *                                   'CALCULATION FLAGS' bit 6).
 *                            bit 2: Initialization Integrity Check bit (same as 31h_RETPOS3 field 'CALCULATION
 *                                   FLAGS' bit 2).
 *                            bit 3: Initialization Integrity Check bit (same as 31h_RETPOS3 field 'CALCULATION
 *                                   FLAGS' bit 3).
 *                            bits 4-7: (reserved)
 *    1 (byte) RTK CONDITION  is the same as bits 0..3 of 31h_RETPOS3 'RTK POSITION FLAGS'
 *                            Bits 0-3 values:
 *                            0: New Position computed
 *                            1: Unable to obtain a synced pair from both stations
 *                            2: Insufficient double difference measurements
 *                            3: Reference position unavailable
 *                            4: Failed integer verification with fixed solution
 *                            5: Solution residual RMS exceeds predefined limit (Roving) or Pole is wobbling (Static)
 *                            6: PDOP or RDOP exceeds (absolute positioning) PDOP mask
 *    4 (float) CORRECTION AGE  is the time in since the last differential measurement update in seconds. Same as in
 *                              31h_RETPOS3, GSOF_17, etc.
 *    1 (byte) NETWORK FLAGS  is the same as the 'NETWORK FLAGS' field in 31h RETPOS3.
 *                            Bit 0: New physical base station available. Bit reset after next command GETBASE, 34h.
 *                            Bit 2,1:
 *                            0,0: RTCM v3 Network messages not available or unknown (RTCM3Net not operational).
 *                            0,1: Collecting RTCM v3 Network messages from beginning and have not received a complete
 *                                 cycle yet.
 *                            1,0: Completed a full cycle collection, but found the network message data insufficient to
 *                                 generate RTK network solutions.
 *                            1,1: RTCM v3 network RTK message collection completed and VRS observations epochs
 *                                 generated from V3 network messages. I.e. V3 network is up and running and in good
 *                                 shape.
 *                            Bit 3: GeoFence option is enabled and unit is outside Geofence area. See also GeoFence,
 *                                   4Bh RETOPT
 *                            Bit 4: RTK Range limiting is enabled and unit is too far from the base (Range limit
 *                                   exceeded).
 *                            Bit 5: RTK xFill position flag. 1 = xFill position, 0 = Not xFill position.
 *                            Bit 6: RTX position flag. 1 = RTX position, 0 = Not RTX position.
 *                            Bit 7: RTX/xFill link is down. 1 = link is down, 0 = don't care
 *    1 (byte) NETWORK FLAGS2 contains extra positioning and status flags.
 *                            Bit 0: xFill is ready to propagate RTK positions (or is already running).
 *                            Bit 1: RTX solution is RAIN (RTX_Fast).
 *                            Bit 2: xFill-RTX offset (from RTK) is known to an acceptable accuracy to propagate RTK.
 *                            Bit 3: If set to 1,indicates that CMRxe is being received.
 *                            Bit 4: If set, indicates RTX is in "wet" area.
 * (new in fw 4.82)
 *    1 (byte) FRAME FLAG
 *    2 (int16) ITRF EPOCH
 *    1 (byte) TECTONIC PLATE   ITRF EPOCH, and TECTONIC PLATE define ITRF realisation of coordinates. See
 *                              Tectonic_Plates
 *    4 (int32) RTX_RAM SUB MINUTES LEFT is the number of minutes remaining for the hourly RTX RAM subscription. 0 means
 *                                       this hourly subscription feature is not used, and 0xFFFFFFFF means all the
 *                                       minutes has been used up or expired.
 * (new in fw 4.90)
 *    1 (byte) POLE WOBBLE STATUS FLAG  only being used when rover is in static mode. In kinematic mode, it is always
 *                                      false. In static mode, when pole wobbled, this flag is set to true by RTK. It
 *                                      is only reset when SC set the rx to kinematic again.
 *    4 (float) POLE WOBBLE DISTANCE in meters.
 * (new in fw 4.94)
 *    1 (byte) POSITION FIX TYPE  is the receiver's internal designation for the type of position solution this is. This
 *                                refers to "enum pm_fix_type_e" in receiver source code (pm/pm_iface.h). See Position
 *                                Fix Type
 * (record may grow in future)
 */
struct PositionTypeInformation {
  Header header;
  float error_scale;
  uint8_t solution_flags;
  uint8_t rtk_condition;
  float correction_age;
  uint8_t network_flags;
  uint8_t network_flags2;
  uint8_t frame_flag;
  uint16_t itrf_epoch;
  uint8_t tectonic_plate;
  int32_t rtx_ram_sub_minutes_left;
  uint8_t pole_wobble_status_flag;
  float pole_wobble_distance;
  uint8_t position_fix_type;
  std::vector<std::byte> unparsed_bytes;  // Because this message may grow with time, we add the possibility of
  // capturing unparsed bytes for forwards compatibility

  void switchEndianess() {
    byteswapInPlace(&error_scale);
    byteswapInPlace(&correction_age);
    byteswapInPlace(&itrf_epoch);
    byteswapInPlace(&rtx_ram_sub_minutes_left);
    byteswapInPlace(&pole_wobble_distance);
  }

  [[nodiscard]] bool isSolutionWideArea() const { return solution_flags & mask0; }
  [[nodiscard]] bool isRtkFixSolution() const { return solution_flags & mask1; }

  [[nodiscard]] SolutionIntegrity getSolutionIntegrity() const { return toSolutionIntegrity(solution_flags); }

  [[nodiscard]] RtkCondition getRtkCondition() const { return toRtkCondition(rtk_condition); }

  [[nodiscard]] bool isNewPhysicalBaseStationAvailable() const { return network_flags & mask0; }

  [[nodiscard]] RtcmStatus getRtcmStatus() const { return toRtcmStatus(network_flags); }

  /**
   *  GeoFence option is enabled and unit is outside Geofence area. See also GeoFence, 4Bh RETOPT
   */
  [[nodiscard]] bool isGeofenceEnabledAndTriggered() const { return network_flags & mask3; }
  [[nodiscard]] bool isRtkRangeLimitExceeded() const { return network_flags & mask4; }
  [[nodiscard]] bool isXFillPosition() const { return network_flags & mask5; }
  [[nodiscard]] bool isRtkPosition() const { return network_flags & mask6; }

  // Note, the negative statement here will probably be mildly confusing since all the other boolean checks are
  // positive assertions. However, this is how the message is documented, and changing this might make things worse.
  [[nodiscard]] bool isRtxOrXFillLinkDown() const { return network_flags & mask7; }

  [[nodiscard]] bool isXFillReady() const { return network_flags2 & mask0; }
  [[nodiscard]] bool isRtxSolutionRain() const { return network_flags2 & mask1; }
  [[nodiscard]] bool isXFillRtxOffsetGood() const { return network_flags2 & mask2; }
  [[nodiscard]] bool isCmrxeReceived() const { return network_flags2 & mask3; }
  [[nodiscard]] bool isRtxInWetArea() const { return network_flags2 & mask4; }

  [[nodiscard]] PositionFix getPositionFixType() const { return toPositionFix(position_fix_type); }
};

/* GSOF #40: LBAND Status Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 5 (bytes) SATELLITE NAME
 * 4 (bytes)/float NOMINAL/REQUESTED SATELLITE FREQUENCY in MHz
 * 2 (bytes)/short SATELLITE BIT RATE [Hz]
 * 4 (bytes)/float C/No [dBHz]
 * 1 (byte) HP/XP SUBSCRIBED ENGINE
 * 1 (byte) HP/XP LIBRARY MODE
 * 1 (byte) VBS LIBRARY MODE
 * 1 (byte) BEAM MODE
 * 1 (byte) OMNISTAR MOTION
 * 4 (bytes)/float 3-SIGMA HORIZONTAL PRECISION THRESHOLD
 * 4 (bytes)/float 3-SIGMA VERTICAL PRECISION THRESHOLD
 * 1 (byte) NMEA ENCRYPTION STATE
 */

struct LbandStatusInfo {
  Header header;
  std::array<char, 5> satellite_name;
  float nominal_sat_freq;
  uint16_t sat_bit_rate;
  float c_no;
  uint8_t hpxp_sub_engine;  // HP/XP Library is software supplied by OmniSTAR
  uint8_t hpxp_library_mode;
  uint8_t vbs_library_mode;
  uint8_t beam_mode;
  uint8_t omnistar_motion;
  float sigma_hor_threshold;
  float sigma_ver_threshold;
  uint8_t nmea_enc_state;
  float i_q_ratio;
  float estimated_bit_error_rate;
  uint32_t total_messages;
  uint32_t total_unique_words_with_errors;
  uint32_t total_bad_unique_word_bits;
  uint32_t total_num_viterbi_symbols;
  uint32_t num_corrected_viterbi_symbols;
  uint32_t num_bad_messages;
  uint8_t meas_frequency_valid_flag;
  double measured_frequency;  // [Hz]

  void switchEndianess() {
    byteswapInPlace(&nominal_sat_freq);
    byteswapInPlace(&sat_bit_rate);
    byteswapInPlace(&c_no);
    byteswapInPlace(&sigma_hor_threshold);
    byteswapInPlace(&sigma_ver_threshold);
    byteswapInPlace(&i_q_ratio);
    byteswapInPlace(&estimated_bit_error_rate);
    byteswapInPlace(&total_messages);
    byteswapInPlace(&total_unique_words_with_errors);
    byteswapInPlace(&total_bad_unique_word_bits);
    byteswapInPlace(&total_num_viterbi_symbols);
    byteswapInPlace(&num_corrected_viterbi_symbols);
    byteswapInPlace(&num_bad_messages);
    byteswapInPlace(&measured_frequency);
  }

  [[nodiscard]] omnistar::HpXpEngine getHpXpEngine() const { return omnistar::toHpXpEngine(hpxp_sub_engine); }
  [[nodiscard]] omnistar::HpXpLibraryMode getHpXpLibraryMode() const {
    return omnistar::toHpXpLibraryMode(hpxp_library_mode);
  }

  [[nodiscard]] omnistar::VbsLibraryMode getVbsLibraryMode() const {
    return omnistar::toVbsLibraryMode(vbs_library_mode);
  }

  [[nodiscard]] omnistar::BeamMode getBeamMode() const { return omnistar::toBeamMode(beam_mode); }

  [[nodiscard]] omnistar::MotionState getMotionState() const { return omnistar::toMotionState(omnistar_motion); }

  [[nodiscard]] omnistar::NmeaEncryptionState getNmeaEncryptionState() const {
    return omnistar::toNmeaEncryptionState(nmea_enc_state);
  }

  [[nodiscard]] bool isMeasuredFrequencyValid() const { return meas_frequency_valid_flag > 0; }
};

enum class BaseQuality {
  k_not_available_or_invalid          = 0,  // Fix not available or invalid
  k_autonomous                        = 1,  // Autonomous
  k_differential_sbas_or_omnistar_vbs = 2,  // Differential, SBAS or OmniSTAR VBS
  k_rtk_fixed                         = 3,  // RTK Fixed
  k_omnistar_xp_hp_or_rtk_float       = 4,  // OmniSTAR XP, OmniSTAR HP, RTK Float or RTK Location
};

BaseQuality toBaseQuality(uint8_t quality);

/* GSOF #41: Base Position and Quality Indicator
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 4 (long) GPS TIME (ms)
 * 2 (int) GPS WEEK NUMBER
 * 8 (double) LATITUDE (rad)
 * 8 (double) LONGITUDE (rad)
 * 8 (double) HEIGHT (m)
 * 1 (byte) QUALITY
 */

struct BasePositionAndQualityIndicator {
  Header header;
  uint32_t gps_time_ms;
  uint16_t gps_week_number;
  Llad llh;
  uint8_t quality;

  void switchEndianess() {
    byteswapInPlace(&gps_time_ms);
    byteswapInPlace(&gps_week_number);
    llh.switchEndianess();
  }

  [[nodiscard]] BaseQuality getBaseQuality() const { return toBaseQuality(quality); }
};

/* GSOF #49 : Full Navigation Info
 *
 * 1 (byte)     OUTPUT RECORD TYPE
 * 1 (byte)     RECORD LENGTH
 * 2 (short)    GPS WEEK NUMBER
 * 4 (int)      GPS time in msec of current week
 * 1 (byte)     IMU alignment status
 * 1 (byte)     GPS Quality indicator
 * 8 (double)   Latitude (degrees)
 * 8 (double)   Longitude (degrees)
 * 8 (double)   Altitude (m)
 * 4 (float)    North velocity (m/s)
 * 4 (float)    East velocity  (m/s)
 * 4 (float)    Down velocity  (m/s)
 * 4 (float)    Total speed (m/s)
 * 8 (double)   Roll     (degrees)
 * 8 (double)   Pitch    (degrees)
 * 8 (double)   Heading  (degrees)
 * 8 (double)   Track angle (degrees)
 * 4 (float)    Angular rate about longitudinal axis (X) (deg/sec)
 * 4 (float)    Angular rate about traverse axis (Y) (deg/sec)
 * 4 (float)    Angular rate about down axis (Z) (deg/sec)
 * 4 (float)    Longitudinal accelaration (X) (m/s^2)
 * 4 (float)    Traverse acceleration (Y) (m/s^2)
 * 4 (float)    Down acceleration (Z) (m/s^2)
 */
struct NavigationSolution {
  Header header;
  GpsTime gps_time;
  Status status;
  Llad lla;            // [deg, m]
  Nedf velocity;       // [m / s]
  float total_speed;   // [m / s]
  Rphd attitude;       // [deg]
  double track_angle;  // [deg]
  Rphf angular_rate;   // [deg / s]
  Xyzf acceleration;   // [m / s^2]
  void switchEndianess() {
    gps_time.switchEndianess();
    // status already aligned
    lla.switchEndianess();
    velocity.switchEndianess();
    byteswapInPlace(&total_speed);
    attitude.switchEndianess();
    byteswapInPlace(&track_angle);
    angular_rate.switchEndianess();
    acceleration.switchEndianess();
  }
};

/* GSOF #50 : INS RMS Info
 *
 * 1 (byte)     OUTPUT RECORD TYPE
 * 1 (byte)     RECORD LENGTH
 * 2 (short)    GPS WEEK NUMBER
 * 4 (int)      GPS time in msec of current week
 * 1 (byte)     IMU alignment status
 * 1 (byte)     GPS Quality indicator
 * 4 (float)    North Position RMS m
 * 4 (float)    East Position RMS  m
 * 4 (float)    Down Position RMS  m
 * 4 (float)    North Velocity RMS m/s
 * 4 (float)    East Velocity RMS m/s
 * 4 (float)    Down Velocity RMS m/s
 * 4 (float)    Roll RMS (deg)
 * 4 (float)    Pitch RMS (deg)
 * 4 (float)    Heading RMS (deg)
 */
struct NavigationPerformance {
  Header header;
  GpsTime gps_time;
  Status status;
  Nedf position_rms;
  Nedf velocity_rms;
  Rphf attitude_rms;
  void switchEndianess() {
    gps_time.switchEndianess();
    position_rms.switchEndianess();
    velocity_rms.switchEndianess();
    attitude_rms.switchEndianess();
  }
};

/* GSOF #52 : DMI Raw Data
 *
 * 1 (byte)     OUTPUT RECORD TYPE
 * 1 (byte)     RECORD LENGTH
 * 2 (short)    GPS WEEK NUMBER
 * 4 (int)      GPS time in msec of current week
 * 1 (byte)     Number of raw measurements
 * 2 (short)    TIME Offset - in msecs with respect to the base time
 * 4 (int)      Absolute distance count [DMI pulses]
 * 4 (int)      Up/Down distance count [DMI pulses]
 */
struct DmiRawData {
  Header header;
  GpsTime gps_time;
  uint8_t num_raw_meas;
  uint16_t time_offset;     // [ms]
  uint32_t abs_dist_count;  // [DMI pulses]
  int32_t ud_dist_count;    // [DMI pulses]

  void switchEndianess() {
    gps_time.switchEndianess();
    byteswapInPlace(&time_offset);
    byteswapInPlace(&abs_dist_count);
    byteswapInPlace(&ud_dist_count);
  }
};

/* GSOF #63: INS VNAV Full Nav Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 2 (int) GPS week number
 * 4 (unsigned int) GPS time (ms)
 * 1 (byte) IMU alignment status
 * 1 (byte) GPS quality indicator
 * 8 (double) Latitude (degrees)
 * 8 (double) Longitude (degrees)
 * 8 (double) Altitude (m)
 * 4 (float) North Velocity (m/s)
 * 4 (float) East Velocity (m/s)
 * 4 (float) Down velocity (m/s)
 * 4 (float) Total Speed (m/s)
 * 8 (double) Roll (degrees)
 * 8 (double) Pitch (degrees)
 * 8 (double) Heading (degrees)
 * 8 (double) Track Angle (degrees)
 * 4 (float) Angular rate about longitudinal axis (X) (deg/sec)
 * 4 (float) Angular rate about traverse axis (Y) (deg/sec)
 * 4 (float) Angular rate about down axis (Z) (deg/sec)
 * 4 (float) Longitudinal accelaration (X) (m/s^2)
 * 4 (float) Traverse acceleration (Y) (m/s^2)
 * 4 (float) Down acceleration (Z) (m/s^2)
 * 8 (double) Heave (m)
 */

struct InsVnavFullNavInfo {
  Header header;
  GpsTime gps_time;
  VnavStatus status;
  Llad lla;
  Nedf velocity;
  float total_speed;
  Rphd attitude;
  double track_angle;
  Xyzf angular_rate;
  Xyzf acceleration;
  double heave;
  void switchEndianess() {
    gps_time.switchEndianess();
    lla.switchEndianess();
    velocity.switchEndianess();
    byteswapInPlace(&total_speed);
    attitude.switchEndianess();
    byteswapInPlace(&track_angle);
    angular_rate.switchEndianess();
    acceleration.switchEndianess();
    byteswapInPlace(&heave);
  }
};

static_assert(sizeof(InsVnavFullNavInfo) == 114);

/* GSOF #64: INS VNAV RMS Info
 * 1 (byte) OUTPUT RECORD TYPE
 * 1 (byte) RECORD LENGTH
 * 2 (int) GPS week number
 * 4 (unsigned int) GPS time (ms)
 * 1 (byte) IMU alignment status
 * 1 (byte) GPS quality indicator
 * 4 (float) North Position RMS (m)
 * 4 (float) East Position RMS (m)
 * 4 (float) Down Position RMS (m)
 * 4 (float) North Velocity RMS (m/s)
 * 4 (float) East Velocity RMS (m/s)
 * 4 (float) Down Velocity RMS (m/s)
 * 4 (float) Roll RMS (degrees)
 * 4 (float) Pitch RMS (degrees)
 * 4 (float) Heading RMS (degrees)
 * 4 (float) Heave RMS (meters)
 */

struct InsVnavRmsInfo {
  Header header;
  GpsTime gps_time;
  VnavStatus status;
  Nedf position_rms;
  Nedf velocity_rms;
  Rphf attitude_rms;
  float heave_rms;
  void switchEndianess() {
    gps_time.switchEndianess();
    position_rms.switchEndianess();
    velocity_rms.switchEndianess();
    attitude_rms.switchEndianess();
    byteswapInPlace(&heave_rms);
  }
};

static_assert(sizeof(InsVnavRmsInfo) == 50);

#pragma pack(pop)

namespace impl {

// Test whether class T has the function switchEndianess(void), doesn't check the return type.
struct test_has_switch_endianess {
  template <class T>
  static auto test(T *p) -> decltype(p->switchEndianess(), std::true_type());

  template <class>
  static auto test(...) -> std::false_type;
};

}  // namespace impl

// Type trait for test_has_switch_endianess
template <class T>
struct has_switch_endianess : decltype(impl::test_has_switch_endianess::test<T>(0)) {
  static constexpr bool value = decltype(impl::test_has_switch_endianess::test<T>(0))();
};

template <class T>
inline constexpr bool has_switch_endianess_v = has_switch_endianess<T>::value;

class Message {
 public:
  Message(const std::byte *data, std::size_t length);
  [[nodiscard]] Header getHeader() const;

  template <typename T>
  [[nodiscard]] T as() const;

 private:
  const std::byte *data_;
  std::size_t length_;
};

template <>
[[nodiscard]] AllSvBrief Message::as<AllSvBrief>() const;

template <>
[[nodiscard]] AllSvDetailed Message::as<AllSvDetailed>() const;

template <>
[[nodiscard]] Velocity Message::as<Velocity>() const;

template <>
[[nodiscard]] AttitudeInfo Message::as<AttitudeInfo>() const;

template <>
[[nodiscard]] PositionTypeInformation Message::as<PositionTypeInformation>() const;

/**
 * General case of type punning a message into a specific GSOF message.
 * @tparam T template parameter should be a packed GSOF struct. Compilation will fail if the type is dynamic. It is
 * expected that you write an explicit template specialization if your type requires dynamic parsing.
 * @return
 */
template <typename T>
[[nodiscard]] T Message::as() const {
  static_assert(has_switch_endianess_v<T>, "Class has to implement function switchEndianess().");
  static_assert(std::is_standard_layout_v<T>);
  static_assert(std::is_trivial_v<T>);
  T t;
  std::memcpy(&t, data_, sizeof(T));
  t.switchEndianess();
  return t;
}

}  // namespace trmb::gsof
