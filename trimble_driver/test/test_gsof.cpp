/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <optional>
#include <random>
#include <vector>

#include "gsof_record_builder.h"
#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"

using gsof_test::makeGenoutRecord;
using gsof_test::serialize;
using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

TEST(GsofParsingTest, positionTimeInfoGsof1) {
  using namespace trmb::gsof;

  const auto record = [] {
    PositionTimeInfo expected{};
    expected.header.type                = GSOF_ID_1_POS_TIME;
    expected.header.length              = sizeof(PositionTimeInfo) - sizeof(Header);
    expected.gps_time_ms                = 162117007;
    expected.gps_week                   = 2225;
    expected.number_space_vehicles_used = 25;
    expected.position_flags_1           = 0b1010'1111;
    expected.position_flags_2           = 0;
    expected.init_num                   = 0;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x01);
  ASSERT_EQ(it->getHeader().length, 10);

  auto position_time = it->as<PositionTimeInfo>();
  ASSERT_EQ(position_time.gps_time_ms, 162117007u);
  ASSERT_EQ(position_time.gps_week, 2225);
  ASSERT_EQ(position_time.number_space_vehicles_used, 25);
  ASSERT_TRUE(position_time.isNewPos());
  ASSERT_TRUE(position_time.isClockFix());
  ASSERT_TRUE(position_time.isHCoordinatesComputedHere());
  ASSERT_TRUE(position_time.isHeightComputedHere());
  ASSERT_TRUE(position_time.isLeastSquares());
  ASSERT_TRUE(position_time.isL1PseudoRangeUsed());

  ASSERT_FALSE(position_time.isDiffSoln());
  ASSERT_FALSE(position_time.isDiffPosInPhase());
  ASSERT_FALSE(position_time.isDiffPosFixedInt());
  ASSERT_FALSE(position_time.isOmnistarSoln());
  ASSERT_FALSE(position_time.isStaticConstr());
  ASSERT_FALSE(position_time.isNetworkRtkSoln());
  ASSERT_FALSE(position_time.isRtkLocation());
  ASSERT_FALSE(position_time.isBeaconDGPS());
}

TEST(GsofParsingTest, latLongHeightGsof2) {
  using namespace trmb::gsof;

  const auto record = [] {
    LatLongHeight expected{};
    expected.header.type   = GSOF_ID_2_LLH;
    expected.header.length = sizeof(LatLongHeight) - sizeof(Header);
    expected.lla.latitude  = 0.7654321;
    expected.lla.longitude = -1.3876543;
    expected.lla.altitude  = 168.25;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x02);
  ASSERT_EQ(header.length, 24);

  const auto lla = it->as<LatLongHeight>();

  ASSERT_DOUBLE_EQ(lla.lla.latitude, 0.7654321);
  ASSERT_DOUBLE_EQ(lla.lla.longitude, -1.3876543);
  ASSERT_DOUBLE_EQ(lla.lla.altitude, 168.25);
}

TEST(GsofParsingTest, ecefPositionGsof3) {
  using namespace trmb::gsof;

  const auto record = [] {
    EcefPosition expected{};
    expected.header.type   = GSOF_ID_3_ECEF;
    expected.header.length = sizeof(EcefPosition) - sizeof(Header);
    expected.pos.x         = 848942.99;
    expected.pos.y         = -4527384.15;
    expected.pos.z         = 4397104.00;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x03);
  ASSERT_EQ(header.length, 24);

  const auto ecef = it->as<EcefPosition>();

  ASSERT_DOUBLE_EQ(ecef.pos.x, 848942.99);
  ASSERT_DOUBLE_EQ(ecef.pos.y, -4527384.15);
  ASSERT_DOUBLE_EQ(ecef.pos.z, 4397104.00);
}

TEST(GsofParsingTest, ecefDeltaGsof6) {
  using namespace trmb::gsof;

  const auto record = [] {
    EcefDelta expected{};
    expected.header.type   = GSOF_ID_6_ECEF_DELTA;
    expected.header.length = sizeof(EcefDelta) - sizeof(Header);
    expected.delta.x       = -3473.5847978937672;
    expected.delta.y       = 10602.019044206478;
    expected.delta.z       = 11631.403884239495;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x06);
  ASSERT_EQ(header.length, 24);

  const auto ecef_delta = it->as<EcefDelta>();

  ASSERT_DOUBLE_EQ(ecef_delta.delta.x, -3473.5847978937672);
  ASSERT_DOUBLE_EQ(ecef_delta.delta.y, 10602.019044206478);
  ASSERT_DOUBLE_EQ(ecef_delta.delta.z, 11631.403884239495);
}

TEST(GsofParsingTest, tplaneDeltaGsof7) {
  using namespace trmb::gsof;

  const auto record = [] {
    TangentPlaneDelta expected{};
    expected.header.type   = GSOF_ID_7_TPLANE_ENU;
    expected.header.length = sizeof(TangentPlaneDelta) - sizeof(Header);
    expected.enu.east      = -1456.685759244824;
    expected.enu.north     = 16051.050661617326;
    expected.enu.up        = 43.853000930700546;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x07);
  ASSERT_EQ(header.length, 24);

  const auto tplane_delta = it->as<TangentPlaneDelta>();

  ASSERT_DOUBLE_EQ(tplane_delta.enu.east, -1456.685759244824);
  ASSERT_DOUBLE_EQ(tplane_delta.enu.north, 16051.050661617326);
  ASSERT_DOUBLE_EQ(tplane_delta.enu.up, 43.853000930700546);
}

TEST(GsofParsingTest, velocityGsof8) {
  using namespace trmb::gsof;

  constexpr uint8_t k_length_without_local_heading = 13;

  const auto record = [] {
    // Velocity has an optional trailing field so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> message;
    const auto append = [&message](auto value) {
      if constexpr (sizeof(value) > 1) {
        byteswapInPlace(&value);
      }
      const auto *bytes = reinterpret_cast<const std::byte *>(&value);
      message.insert(message.end(), bytes, bytes + sizeof(value));
    };

    append(static_cast<uint8_t>(GSOF_ID_8_VELOCITY));
    append(k_length_without_local_heading);
    append(static_cast<uint8_t>(0b0000'0101));
    append(0.00597169f);
    append(2.062232f);
    append(-0.0006348496f);

    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x08);
  ASSERT_EQ(header.length, k_length_without_local_heading);

  const auto velocity = it->as<Velocity>();

  ASSERT_FLOAT_EQ(velocity.velocity, 0.00597169);
  ASSERT_FLOAT_EQ(velocity.heading, 2.062232);
  ASSERT_FLOAT_EQ(velocity.vertical_velocity, -0.0006348496);
  ASSERT_FALSE(velocity.local_heading);  // No local heading

  ASSERT_TRUE(velocity.isVelDataValid());
  ASSERT_EQ(velocity.getVelocitySource(), Velocity::VelocitySource::k_consecutive_measurements);
  ASSERT_TRUE(velocity.isHeadingDataValid());
}

TEST(GsofParsingTest, pdopInfoGsof9) {
  using namespace trmb::gsof;

  const auto record = [] {
    PdopInfo expected{};
    expected.header.type     = GSOF_ID_9_DOP;
    expected.header.length   = sizeof(PdopInfo) - sizeof(Header);
    expected.position_dop    = 0.9000375f;
    expected.horiziontal_dop = 0.488651454f;
    expected.vertical_dop    = 0.7558355f;
    expected.time_dop        = 1.13949406f;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x09);
  ASSERT_EQ(header.length, 16);

  const auto dop = it->as<PdopInfo>();

  ASSERT_FLOAT_EQ(dop.position_dop, 0.9000375);
  ASSERT_FLOAT_EQ(dop.horiziontal_dop, 0.488651454);
  ASSERT_FLOAT_EQ(dop.vertical_dop, 0.7558355);
  ASSERT_FLOAT_EQ(dop.time_dop, 1.13949406);
}

TEST(GsofParsingTest, clockInfoGsof10) {
  using namespace trmb::gsof;

  const auto record = [] {
    ClockInfo expected{};
    expected.header.type   = GSOF_ID_10_CLOCK_INFO;
    expected.header.length = sizeof(ClockInfo) - sizeof(Header);
    expected.clock_flags   = 0b0000'0111;
    expected.clock_offset  = 0.007924753666675877;
    expected.freq_offset   = -1.6607935199691757;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 10);
  ASSERT_EQ(header.length, 17);

  const auto clock = it->as<ClockInfo>();

  ASSERT_TRUE(clock.isClockOffsetValid());
  ASSERT_TRUE(clock.isFreqOffsetValid());
  ASSERT_TRUE(clock.isReceiverInAnywhereFixMode());

  ASSERT_DOUBLE_EQ(clock.clock_offset, 0.007924753666675877);
  ASSERT_DOUBLE_EQ(clock.freq_offset, -1.6607935199691757);
}

TEST(GsofParsingTest, positionVcvGsof11) {
  using namespace trmb::gsof;

  const auto record = [] {
    PositionVcvInfo expected{};
    expected.header.type   = GSOF_ID_11_POS_VCV_INFO;
    expected.header.length = sizeof(PositionVcvInfo) - sizeof(Header);
    expected.position_rms  = 0.001f;
    expected.xx            = 0.0004194359f;
    expected.xy            = 0.0000253239959f;
    expected.xz            = 0.0000135725622f;
    expected.yy            = 0.000340281957f;
    expected.yz            = 0.0000268921121f;
    expected.zz            = 0.0003106595f;
    expected.unit_var      = 1.0f;
    expected.num_epochs    = 0;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 11);
  ASSERT_EQ(header.length, 34);

  const auto covariance = it->as<PositionVcvInfo>();

  ASSERT_FLOAT_EQ(covariance.position_rms, 0.001);
  ASSERT_FLOAT_EQ(covariance.xx, 0.0004194359);
  ASSERT_FLOAT_EQ(covariance.xy, 0.0000253239959);
  ASSERT_FLOAT_EQ(covariance.xz, 0.0000135725622);
  ASSERT_FLOAT_EQ(covariance.yy, 0.000340281957);
  ASSERT_FLOAT_EQ(covariance.yz, 0.0000268921121);
  ASSERT_FLOAT_EQ(covariance.zz, 0.0003106595);
  ASSERT_FLOAT_EQ(covariance.unit_var, 1.0f);
  ASSERT_EQ(covariance.num_epochs, 0);
}

TEST(GsofParsingTest, positionSigmaGsof12) {
  using namespace trmb::gsof;

  const auto record = [] {
    PositionSigmaInfo expected{};
    expected.header.type           = GSOF_ID_12_POS_SIGMA;
    expected.header.length         = sizeof(PositionSigmaInfo) - sizeof(Header);
    expected.position_rms          = 0.001f;
    expected.sigma_east            = 0.200461134f;
    expected.sigma_north           = 0.0194671229f;
    expected.covariance_east_north = -0.000291388424f;
    expected.sigma_up              = 0.0169321168f;
    expected.semi_major_axis       = 0.200466454f;
    expected.semi_minor_axis       = 0.0194122624f;
    expected.orientation           = 90.41939f;
    expected.unit_variance         = 1.0f;
    expected.number_epochs         = 0;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 12);
  ASSERT_EQ(header.length, 38);

  const auto sigma = it->as<PositionSigmaInfo>();

  ASSERT_FLOAT_EQ(sigma.position_rms, 0.001);
  ASSERT_FLOAT_EQ(sigma.sigma_east, 0.200461134);
  ASSERT_FLOAT_EQ(sigma.sigma_north, 0.0194671229);
  ASSERT_FLOAT_EQ(sigma.covariance_east_north, -0.000291388424);
  ASSERT_FLOAT_EQ(sigma.sigma_up, 0.0169321168);
  ASSERT_FLOAT_EQ(sigma.semi_major_axis, 0.200466454);
  ASSERT_FLOAT_EQ(sigma.semi_minor_axis, 0.0194122624);
  ASSERT_FLOAT_EQ(sigma.orientation, 90.41939);
  ASSERT_FLOAT_EQ(sigma.unit_variance, 1);
  ASSERT_EQ(sigma.number_epochs, 0);
}

TEST(GsofParsingTest, receiverSerialNumberGsof15) {
  using namespace trmb::gsof;

  const auto record = [] {
    ReceiverSerialNumber expected{};
    expected.header.type   = GSOF_ID_15_REC_SERIAL_NUM;
    expected.header.length = sizeof(ReceiverSerialNumber) - sizeof(Header);
    expected.number        = 573901879;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 15);
  ASSERT_EQ(header.length, 4);

  const auto serial = it->as<ReceiverSerialNumber>();

  ASSERT_EQ(serial.number, 573901879);
}

TEST(GsofParsingTest, currentTimeGsof16) {
  using namespace trmb::gsof;

  const auto record = [] {
    CurrentTime expected{};
    expected.header.type   = GSOF_ID_16_CURR_TIME;
    expected.header.length = sizeof(CurrentTime) - sizeof(Header);
    expected.gps_ms_week   = 162117007;
    expected.gps_week      = 2225;
    expected.utc_offset    = 18;
    expected.flags         = 0b0000'0011;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 16);
  ASSERT_EQ(header.length, 9);

  const auto time = it->as<CurrentTime>();

  ASSERT_EQ(time.gps_ms_week, 162117007u);
  ASSERT_EQ(time.gps_week, 2225u);
  ASSERT_EQ(time.utc_offset, 18u);

  ASSERT_TRUE(time.isTimeInfoValid());
  ASSERT_TRUE(time.isUtcOffsetValid());
}

TEST(GsofParsingTest, attitudeInfoGsof27) {
  using namespace trmb::gsof;

  constexpr uint8_t k_length_with_variance = 70;

  const auto record = [] {
    // AttitudeInfo has an optional trailing variance block so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> message;
    const auto append = [&message](auto value) {
      if constexpr (sizeof(value) > 1) {
        byteswapInPlace(&value);
      }
      const auto *bytes = reinterpret_cast<const std::byte *>(&value);
      message.insert(message.end(), bytes, bytes + sizeof(value));
    };

    append(static_cast<uint8_t>(GSOF_ID_27_ATTITUDE));
    append(k_length_with_variance);
    append(static_cast<uint32_t>(162629000));
    append(static_cast<uint8_t>(0b0000'1111));
    append(static_cast<uint8_t>(27));                                    // num svs
    append(static_cast<uint8_t>(Mode::CalcMode::RTK_FIX));               // calc mode
    append(static_cast<uint8_t>(0));                                     // reserved
    append(-6.92433215936082754188296561892E-3);                         // pitch
    append(3.131870495343419);                                           // yaw
    append(3.135750624153477);                                           // roll
    append(0.0);                                                         // master slave range
    append(static_cast<uint16_t>(10));                                   // pdop
    append(0.000003936937f);                                             // pitch variance
    append(0.0000271099088f);                                            // yaw variance
    append(0.00000382736971f);                                           // roll variance
    append(0.0f);                                                        // pitch-yaw covariance
    append(0.0f);                                                        // pitch-roll covariance
    append(0.0f);                                                        // yaw-roll covariance
    append(0.0f);                                                        // master slave range variance

    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 27);
  ASSERT_EQ(header.length, k_length_with_variance);

  const auto attitude = it->as<AttitudeInfo>();

  ASSERT_EQ(attitude.gps_time, 162629000u);
  ASSERT_TRUE(attitude.isCalibrated());
  ASSERT_TRUE(attitude.isPitchValid());
  ASSERT_TRUE(attitude.isYawValid());
  ASSERT_TRUE(attitude.isRollValid());
  ASSERT_FALSE(attitude.isScalarValid());
  ASSERT_FALSE(attitude.isDiagValid());
  ASSERT_FALSE(attitude.isSlaveStatic());
  ASSERT_FALSE(attitude.isErrValid());
  ASSERT_EQ(attitude.num_svs, 27);
  ASSERT_EQ(attitude.calc_mode.getCalcMode(), Mode::CalcMode::RTK_FIX);
  ASSERT_DOUBLE_EQ(attitude.pyr.pitch, -6.92433215936082754188296561892E-3);
  ASSERT_DOUBLE_EQ(attitude.pyr.yaw, 3.131870495343419);
  ASSERT_DOUBLE_EQ(attitude.pyr.roll, 3.135750624153477);
  ASSERT_DOUBLE_EQ(attitude.master_slave_range, 0.0);
  ASSERT_EQ(attitude.pdop, 10);
  ASSERT_TRUE(attitude.variance.has_value());
  ASSERT_FLOAT_EQ(attitude.variance->pitch, 0.000003936937);
  ASSERT_FLOAT_EQ(attitude.variance->yaw, 0.0000271099088);
  ASSERT_FLOAT_EQ(attitude.variance->roll, 0.00000382736971);
  ASSERT_FLOAT_EQ(attitude.variance->pitch_yaw, 0.0);
  ASSERT_FLOAT_EQ(attitude.variance->pitch_roll, 0.0);
  ASSERT_FLOAT_EQ(attitude.variance->yaw_roll, 0.0);
  ASSERT_FLOAT_EQ(attitude.variance->master_slave_range, 0.0);
}

TEST(GsofParsingTest, allSvBriefInfoGsof33) {
  using namespace trmb::gsof;

  constexpr std::size_t k_num_svs                                    = 4;
  const std::array<uint8_t, k_num_svs> expected_prn                  = {0x0b, 0x17, 0x03, 0x29};
  const std::array<uint8_t, k_num_svs> sv_system                     = {0, 2, 3, 5};
  const std::array<SatelliteType, k_num_svs> expected_satellite_type = {SatelliteType::GPS, SatelliteType::GLONASS,
                                                                       SatelliteType::GALILEO, SatelliteType::BEIDOU};

  const auto record = [&] {
    // AllSvBrief holds a variable length satellite list so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> message;
    const auto append = [&message](uint8_t value) { message.push_back(static_cast<std::byte>(value)); };

    append(GSOF_ID_33_ALL_SV_BRIEF);
    append(static_cast<uint8_t>(1 + k_num_svs * sizeof(SVBriefInfo)));
    append(k_num_svs);
    for (std::size_t i = 0; i < k_num_svs; ++i) {
      append(expected_prn[i]);
      append(sv_system[i]);
      append(0b0000'0111);  // above horizon, assigned to channel, tracked
      append(0);
    }

    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 33);
  ASSERT_EQ(header.length, 1 + k_num_svs * sizeof(SVBriefInfo));

  const auto brief_sv_info = it->as<AllSvBrief>();
  ASSERT_EQ(brief_sv_info.num_svs, k_num_svs);
  ASSERT_EQ(brief_sv_info.sv_info.size(), k_num_svs);

  for (std::size_t i = 0; i < k_num_svs; ++i) {
    const auto &info = brief_sv_info.sv_info[i];
    ASSERT_EQ(info.prn, expected_prn[i]);
    ASSERT_EQ(info.getSVSystemMode(), expected_satellite_type[i]);
    ASSERT_TRUE(info.isAboveHor());
    ASSERT_TRUE(info.isAssignedToChannel());
    ASSERT_TRUE(info.isTracked());
  }
}

TEST(GsofParsingTest, allSvDetailedInfoGsof34) {
  using namespace trmb::gsof;

  constexpr std::size_t k_num_svs = 3;

  const auto record = [] {
    // AllSvDetailed holds a variable length satellite list so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> message;
    const auto append = [&message](auto value) {
      if constexpr (sizeof(value) > 1) {
        byteswapInPlace(&value);
      }
      const auto *bytes = reinterpret_cast<const std::byte *>(&value);
      message.insert(message.end(), bytes, bytes + sizeof(value));
    };

    const auto appendSv = [&append](uint8_t prn, uint8_t sv_system, uint8_t flags1, uint8_t flags2, uint8_t elevation,
                                    uint16_t azimuth, uint8_t snr_l1, uint8_t snr_l2, uint8_t snr_l5) {
      append(prn);
      append(sv_system);
      append(flags1);
      append(flags2);
      append(elevation);
      append(azimuth);
      append(snr_l1);
      append(snr_l2);
      append(snr_l5);
    };

    append(static_cast<uint8_t>(GSOF_ID_34_ALL_SV_DETAIL));
    append(static_cast<uint8_t>(1 + k_num_svs * sizeof(SVDetailedInfo)));
    append(static_cast<uint8_t>(k_num_svs));

    // GPS satellite used in the RTK solution, tracking L2 CS, L5 and L1C
    appendSv(0x17, 0, 0b1111'1111, 0b0001'1100, 45, 180, 40, 41, 42);
    // GLONASS-M satellite that is tracked but unused
    appendSv(0x08, 2, 0b0000'0111, 0b0000'0100, 30, 90, 35, 36, 0);
    // OmniSTAR satellite that is not being tracked
    appendSv(0x1a, 10, 0, 0, 0, 0, 0, 0, 0);

    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 34);
  ASSERT_EQ(header.length, 1 + k_num_svs * sizeof(SVDetailedInfo));

  const auto sv_info = it->as<AllSvDetailed>();
  ASSERT_EQ(sv_info.num_svs, k_num_svs);
  ASSERT_EQ(sv_info.sv_info.size(), k_num_svs);

  const auto &gps = sv_info.sv_info[0];
  ASSERT_EQ(gps.prn, 0x17);
  ASSERT_EQ(gps.getSvType(), SatelliteType::GPS);
  ASSERT_EQ(gps.elevation, 45);
  ASSERT_EQ(gps.azimuth, 180);
  ASSERT_EQ(gps.snr_L1, 40);
  ASSERT_EQ(gps.snr_L2, 41);
  ASSERT_EQ(gps.snr_L5, 42);
  ASSERT_TRUE(gps.isAboveHor());
  ASSERT_TRUE(gps.isAssignedToChannel());
  ASSERT_TRUE(gps.isTracked());
  ASSERT_TRUE(gps.isCurrTrackedDual());
  ASSERT_TRUE(gps.isL1G1Freq());
  ASSERT_TRUE(gps.isL1G2Freq());
  ASSERT_TRUE(gps.isUsedAtCurrentPos());
  ASSERT_TRUE(gps.isUsedInRtkSolution());
  ASSERT_FALSE(gps.isTrackPCodeL1G1());
  ASSERT_FALSE(gps.isTrackPCodeL2());
  ASSERT_TRUE(gps.isTrackingCsOnL2());
  ASSERT_TRUE(gps.isTrackingL5Signal());
  ASSERT_TRUE(gps.isTrackingL1C());
  ASSERT_FALSE(gps.isGlonassMSv());
  ASSERT_FALSE(gps.isGlonassKSv());

  const auto &glonass = sv_info.sv_info[1];
  ASSERT_EQ(glonass.prn, 0x08);
  ASSERT_EQ(glonass.getSvType(), SatelliteType::GLONASS);
  ASSERT_EQ(glonass.azimuth, 90);
  ASSERT_TRUE(glonass.isAboveHor());
  ASSERT_TRUE(glonass.isAssignedToChannel());
  ASSERT_TRUE(glonass.isTracked());
  ASSERT_FALSE(glonass.isCurrTrackedDual());
  ASSERT_FALSE(glonass.isUsedInRtkSolution());
  ASSERT_TRUE(glonass.isGlonassMSv());
  ASSERT_FALSE(glonass.isGlonassKSv());
  ASSERT_FALSE(glonass.isTrackingCsOnL2());

  const auto &omnistar = sv_info.sv_info[2];
  ASSERT_EQ(omnistar.prn, 0x1a);
  ASSERT_EQ(omnistar.getSvType(), SatelliteType::OMNISTAR);
  ASSERT_FALSE(omnistar.isAboveHor());
  ASSERT_FALSE(omnistar.isAssignedToChannel());
  ASSERT_FALSE(omnistar.isTracked());
}

TEST(GsofParsingTest, receivedBaseInfoGsof35) {
  using namespace trmb::gsof;

  const std::array<char, 8> expected_name = {'R', 'T', 'C', 'M', '0', '0', '0', '0'};

  const auto record = [&expected_name] {
    ReceivedBaseInfo expected{};
    expected.header.type        = GSOF_ID_35_RECEIVED_BASE_INFO;
    expected.header.length      = sizeof(ReceivedBaseInfo) - sizeof(Header);
    expected.flags              = 0b0000'1000;  // version 0, base info valid
    expected.name               = expected_name;
    expected.id                 = 0;
    expected.base_lla.latitude  = 0.7630018859478892;
    expected.base_lla.longitude = -1.385119519554126;
    expected.base_lla.altitude  = 101.22497844472646;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 35);
  ASSERT_EQ(header.length, 35);

  const auto base_info = it->as<ReceivedBaseInfo>();

  ASSERT_EQ(expected_name, base_info.name);
  ASSERT_EQ(base_info.getVersionNumber(), 0);
  ASSERT_EQ(base_info.id, 0);
  ASSERT_TRUE(base_info.isBaseInfoValid());

  ASSERT_DOUBLE_EQ(base_info.base_lla.latitude, 0.7630018859478892);
  ASSERT_DOUBLE_EQ(base_info.base_lla.longitude, -1.385119519554126);
  ASSERT_DOUBLE_EQ(base_info.base_lla.altitude, 101.22497844472646);
}

TEST(GsofParsingTest, batteryMemoryInfoGsof37) {
  using namespace trmb::gsof;

  const auto record = [] {
    BatteryMemoryInfo expected{};
    expected.header.type    = GSOF_ID_37_BATTERY_MEM_INFO;
    expected.header.length  = sizeof(BatteryMemoryInfo) - sizeof(Header);
    expected.battery_cap    = 100;
    expected.remaining_time = 1343.1210074074074;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 37);
  ASSERT_EQ(header.length, 10);

  const auto battery = it->as<BatteryMemoryInfo>();

  ASSERT_EQ(battery.battery_cap, 100);
  ASSERT_DOUBLE_EQ(battery.remaining_time, 1343.1210074074074);
}

TEST(GsofParsingTest, positionTypeGsof38) {
  using namespace trmb::gsof;

  constexpr uint8_t k_length_up_to_fw4_94 = 26;

  const auto record = [] {
    // PositionTypeInformation can grow with firmware versions so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> message;
    const auto append = [&message](auto value) {
      if constexpr (sizeof(value) > 1) {
        byteswapInPlace(&value);
      }
      const auto *bytes = reinterpret_cast<const std::byte *>(&value);
      message.insert(message.end(), bytes, bytes + sizeof(value));
    };

    append(static_cast<uint8_t>(GSOF_ID_38_POSITION_TYPE_INFO));
    append(k_length_up_to_fw4_94);
    append(1.0f);                                             // error scale
    append(static_cast<uint8_t>(0b0000'1010));                // RTK fix, initialization integrity check passed
    append(static_cast<uint8_t>(0));                          // rtk condition
    append(0.0f);                                             // correction age
    append(static_cast<uint8_t>(0));                          // network flags
    append(static_cast<uint8_t>(0));                          // network flags 2
    append(static_cast<uint8_t>(0));                          // frame flag
    append(static_cast<uint16_t>(0));                         // itrf epoch
    append(static_cast<uint8_t>(0));                          // tectonic plate
    append(int32_t{0});                                       // rtx ram subscription minutes left
    append(static_cast<uint8_t>(0));                          // pole wobble status flag
    append(0.0f);                                             // pole wobble distance
    append(static_cast<uint8_t>(PositionFix::k_ins_rtk));     // position fix type

    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 38);
  ASSERT_EQ(header.length, k_length_up_to_fw4_94);

  const auto type = it->as<PositionTypeInformation>();
  ASSERT_EQ(type.unparsed_bytes.size(), 0ul);

  ASSERT_FLOAT_EQ(type.error_scale, 1.f);
  ASSERT_FALSE(type.isSolutionWideArea());
  ASSERT_TRUE(type.isRtkFixSolution());
  ASSERT_EQ(type.getSolutionIntegrity(), SolutionIntegrity::k_initialization_passed);
  ASSERT_EQ(type.getRtkCondition(), RtkCondition::k_new_position_computed);
  ASSERT_FALSE(type.isNewPhysicalBaseStationAvailable());
  ASSERT_FLOAT_EQ(type.correction_age, 0.0f);
  ASSERT_EQ(type.getRtcmStatus(), RtcmStatus::k_not_available_or_unknown);
  ASSERT_FALSE(type.isGeofenceEnabledAndTriggered());
  ASSERT_FALSE(type.isRtkRangeLimitExceeded());
  ASSERT_FALSE(type.isXFillPosition());
  ASSERT_FALSE(type.isRtkPosition());
  ASSERT_FALSE(type.isRtxOrXFillLinkDown());

  ASSERT_FALSE(type.isXFillReady());
  ASSERT_FALSE(type.isRtxSolutionRain());
  ASSERT_FALSE(type.isXFillRtxOffsetGood());
  ASSERT_FALSE(type.isCmrxeReceived());
  ASSERT_FALSE(type.isRtxInWetArea());

  ASSERT_EQ(type.frame_flag, 0);
  ASSERT_EQ(type.itrf_epoch, 0);
  ASSERT_EQ(type.tectonic_plate, 0);
  ASSERT_EQ(type.rtx_ram_sub_minutes_left, 0);
  ASSERT_EQ(type.pole_wobble_status_flag, 0);
  ASSERT_FLOAT_EQ(type.pole_wobble_distance, 0.0f);
  ASSERT_EQ(type.getPositionFixType(), PositionFix::k_ins_rtk);
}

TEST(GsofParsingTest, lbandStatusGsof40) {
  using namespace trmb::gsof;

  const std::array<char, 5> expected_name = {'R', 'T', 'X', 'N', 'A'};

  const auto record = [&expected_name] {
    LbandStatusInfo expected{};
    expected.header.type   = GSOF_ID_40_LBAND_STATUS;
    expected.header.length = sizeof(LbandStatusInfo) - sizeof(Header);
    expected.satellite_name = expected_name;
    // This frequency and baud rate were taken from https://positioningservices.trimble.com/resources/sat/ it is the
    // North America RTX beam frequency and baud
    expected.nominal_sat_freq              = 1555.8080f;
    expected.sat_bit_rate                  = 2400;
    expected.c_no                          = 47.0f;
    expected.hpxp_sub_engine               = static_cast<uint8_t>(omnistar::HpXpEngine::k_hp);
    expected.hpxp_library_mode             = static_cast<uint8_t>(omnistar::HpXpLibraryMode::k_not_active);
    expected.vbs_library_mode              = static_cast<uint8_t>(omnistar::VbsLibraryMode::k_not_active);
    expected.beam_mode                     = static_cast<uint8_t>(omnistar::BeamMode::k_tracking);
    expected.omnistar_motion               = static_cast<uint8_t>(omnistar::MotionState::k_unknown);
    expected.sigma_hor_threshold           = 0.3f;
    expected.sigma_ver_threshold           = 0.3f;
    expected.nmea_enc_state                = static_cast<uint8_t>(omnistar::NmeaEncryptionState::k_off);
    expected.i_q_ratio                     = 5.08325863f;
    expected.estimated_bit_error_rate      = 0.000029838955f;
    expected.total_messages                = 8790;
    expected.total_unique_words_with_errors = 3265;
    expected.total_bad_unique_word_bits    = 9939;
    expected.total_num_viterbi_symbols     = 71453248;
    expected.num_corrected_viterbi_symbols = 1147334;
    expected.num_bad_messages              = 39;
    expected.meas_frequency_valid_flag     = 1;
    expected.measured_frequency            = 1555808020.7808383;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 40);
  ASSERT_EQ(header.length, 70);

  const auto lband_status = it->as<LbandStatusInfo>();

  ASSERT_EQ(lband_status.satellite_name, expected_name);

  ASSERT_FLOAT_EQ(lband_status.nominal_sat_freq, 1555.8080f);
  ASSERT_EQ(lband_status.sat_bit_rate, 2400);

  ASSERT_NEAR(static_cast<double>(lband_status.c_no), 47.0, 0.1);
  ASSERT_EQ(lband_status.getHpXpEngine(), omnistar::HpXpEngine::k_hp);
  ASSERT_EQ(lband_status.getHpXpLibraryMode(), omnistar::HpXpLibraryMode::k_not_active);
  ASSERT_EQ(lband_status.getVbsLibraryMode(), omnistar::VbsLibraryMode::k_not_active);
  ASSERT_EQ(lband_status.getBeamMode(), omnistar::BeamMode::k_tracking);
  ASSERT_EQ(lband_status.getMotionState(), omnistar::MotionState::k_unknown);
  ASSERT_FLOAT_EQ(lband_status.sigma_hor_threshold, 0.3f);
  ASSERT_FLOAT_EQ(lband_status.sigma_ver_threshold, 0.3f);
  ASSERT_EQ(lband_status.getNmeaEncryptionState(), omnistar::NmeaEncryptionState::k_off);

  ASSERT_FLOAT_EQ(lband_status.i_q_ratio, 5.08325863);
  ASSERT_FLOAT_EQ(lband_status.estimated_bit_error_rate, 0.000029838955);

  ASSERT_EQ(lband_status.total_messages, 8790u);
  ASSERT_EQ(lband_status.total_unique_words_with_errors, 3265u);
  ASSERT_EQ(lband_status.total_bad_unique_word_bits, 9939u);
  ASSERT_EQ(lband_status.total_num_viterbi_symbols, 71453248u);
  ASSERT_EQ(lband_status.num_corrected_viterbi_symbols, 1147334u);
  ASSERT_EQ(lband_status.num_bad_messages, 39u);

  ASSERT_TRUE(lband_status.isMeasuredFrequencyValid());
  ASSERT_DOUBLE_EQ(lband_status.measured_frequency, 1555808020.7808383);
}
TEST(GsofParsingTest, basePositionQualityGsof41) {
  using namespace trmb::gsof;

  const auto record = [] {
    BasePositionAndQualityIndicator expected{};
    expected.header.type     = GSOF_ID_41_BASE_POSITION_QUALITY;
    expected.header.length   = sizeof(BasePositionAndQualityIndicator) - sizeof(Header);
    expected.gps_time_ms     = 328103007;
    expected.gps_week_number = 2225;
    expected.llh.latitude    = 0.7630018859478892;
    expected.llh.longitude   = -1.385119519554126;
    expected.llh.altitude    = 101.22497844472646;
    expected.quality         = static_cast<uint8_t>(BaseQuality::k_omnistar_xp_hp_or_rtk_float);

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 41);
  ASSERT_EQ(header.length, 31);

  const auto base_qual = it->as<BasePositionAndQualityIndicator>();

  ASSERT_EQ(base_qual.gps_time_ms, 328103007u);
  ASSERT_EQ(base_qual.gps_week_number, 2225u);
  ASSERT_DOUBLE_EQ(base_qual.llh.latitude, 0.7630018859478892);
  ASSERT_DOUBLE_EQ(base_qual.llh.longitude, -1.385119519554126);
  ASSERT_DOUBLE_EQ(base_qual.llh.altitude, 101.22497844472646);
  ASSERT_EQ(base_qual.getBaseQuality(), BaseQuality::k_omnistar_xp_hp_or_rtk_float);
}

TEST(GsofParsingTest, fullNavigationInfoGsof49) {
  using namespace trmb::gsof;

  const auto record = [] {
    NavigationSolution expected{};
    expected.header.type          = GSOF_ID_49_INS_FULL_NAV;
    expected.header.length        = sizeof(NavigationSolution) - sizeof(Header);
    expected.gps_time.week        = 2080;  // week of nov 17th 2019
    expected.gps_time.time_msec   = 162117007;
    expected.status.imu_alignment = static_cast<uint8_t>(Status::ImuAlignmentStatus::FULL_NAV);
    expected.status.gnss          = static_cast<uint8_t>(Status::GnssStatus::FIXED_RTK_MODE);
    // Guesstimated Google Maps lat long of Trimble Applanix Richmond Hill - Canada office
    expected.lla.latitude         = 43.86;
    expected.lla.longitude        = -79.38;
    expected.lla.altitude         = 168.25;
    expected.velocity.north       = 1.5f;
    expected.velocity.east        = -0.25f;
    expected.velocity.down        = 0.125f;
    expected.total_speed          = 1.5207f;
    expected.attitude.roll        = 0.5;
    expected.attitude.pitch       = -1.25;
    expected.attitude.heading     = 91.75;
    expected.track_angle          = 92.5;
    expected.angular_rate.roll    = 0.01f;
    expected.angular_rate.pitch   = -0.02f;
    expected.angular_rate.heading = 0.03f;
    expected.acceleration.x       = 0.1f;
    expected.acceleration.y       = -0.2f;
    expected.acceleration.z       = 9.81f;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x31);
  ASSERT_EQ(it->getHeader().length, 0x68);  // Read our ICD not the GSOF stuff on the web

  const auto solution = it->as<NavigationSolution>();
  ASSERT_EQ(solution.gps_time.week, 2080);
  ASSERT_EQ(solution.gps_time.time_msec, 162117007u);
  ASSERT_DOUBLE_EQ(solution.lla.latitude, 43.86);
  ASSERT_DOUBLE_EQ(solution.lla.longitude, -79.38);
  ASSERT_DOUBLE_EQ(solution.lla.altitude, 168.25);

  ASSERT_EQ(solution.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::FULL_NAV);
  ASSERT_EQ(solution.status.getGnssStatus(), Status::GnssStatus::FIXED_RTK_MODE);

  ASSERT_FLOAT_EQ(solution.velocity.north, 1.5f);
  ASSERT_FLOAT_EQ(solution.velocity.east, -0.25f);
  ASSERT_FLOAT_EQ(solution.velocity.down, 0.125f);
  ASSERT_FLOAT_EQ(solution.total_speed, 1.5207f);

  ASSERT_DOUBLE_EQ(solution.attitude.roll, 0.5);
  ASSERT_DOUBLE_EQ(solution.attitude.pitch, -1.25);
  ASSERT_DOUBLE_EQ(solution.attitude.heading, 91.75);
  ASSERT_DOUBLE_EQ(solution.track_angle, 92.5);

  ASSERT_FLOAT_EQ(solution.angular_rate.roll, 0.01f);
  ASSERT_FLOAT_EQ(solution.angular_rate.pitch, -0.02f);
  ASSERT_FLOAT_EQ(solution.angular_rate.heading, 0.03f);

  ASSERT_FLOAT_EQ(solution.acceleration.x, 0.1f);
  ASSERT_FLOAT_EQ(solution.acceleration.y, -0.2f);
  ASSERT_FLOAT_EQ(solution.acceleration.z, 9.81f);
}

TEST(GsofParsingTest, fullNavigationRmsGsof50) {
  using namespace trmb::gsof;

  const auto record = [] {
    NavigationPerformance expected{};
    expected.header.type          = GSOF_ID_50_INS_RMS;
    expected.header.length        = sizeof(NavigationPerformance) - sizeof(Header);
    expected.gps_time.week        = 2080;
    expected.gps_time.time_msec   = 162117007;
    expected.status.imu_alignment = static_cast<uint8_t>(Status::ImuAlignmentStatus::FULL_NAV);
    expected.status.gnss          = static_cast<uint8_t>(Status::GnssStatus::FIXED_RTK_MODE);
    expected.position_rms.north   = 0.015f;
    expected.position_rms.east    = 0.014f;
    expected.position_rms.down    = 0.025f;
    expected.velocity_rms.north   = 0.005f;
    expected.velocity_rms.east    = 0.004f;
    expected.velocity_rms.down    = 0.007f;
    expected.attitude_rms.roll    = 0.011f;
    expected.attitude_rms.pitch   = 0.012f;
    expected.attitude_rms.heading = 0.045f;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());

  auto it        = message_parser.begin();
  const auto rms = it->as<NavigationPerformance>();
  ASSERT_EQ(rms.gps_time.week, 2080);
  ASSERT_EQ(rms.gps_time.time_msec, 162117007u);

  ASSERT_EQ(rms.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::FULL_NAV);
  ASSERT_EQ(rms.status.getGnssStatus(), Status::GnssStatus::FIXED_RTK_MODE);

  ASSERT_FLOAT_EQ(rms.position_rms.north, 0.015f);
  ASSERT_FLOAT_EQ(rms.position_rms.east, 0.014f);
  ASSERT_FLOAT_EQ(rms.position_rms.down, 0.025f);
  ASSERT_FLOAT_EQ(rms.velocity_rms.north, 0.005f);
  ASSERT_FLOAT_EQ(rms.velocity_rms.east, 0.004f);
  ASSERT_FLOAT_EQ(rms.velocity_rms.down, 0.007f);
  ASSERT_FLOAT_EQ(rms.attitude_rms.roll, 0.011f);
  ASSERT_FLOAT_EQ(rms.attitude_rms.pitch, 0.012f);
  ASSERT_FLOAT_EQ(rms.attitude_rms.heading, 0.045f);

  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, dmiRawDataGsof52) {
  using namespace trmb::gsof;

  const auto record = [] {
    DmiRawData expected{};
    expected.header.type        = GSOF_ID_52_DMI_RAW_DATA;
    expected.header.length      = sizeof(DmiRawData) - sizeof(Header);
    expected.gps_time.week      = 2122;
    expected.gps_time.time_msec = 162117007;
    expected.num_raw_meas       = 10;
    expected.time_offset        = 25;
    expected.abs_dist_count     = 123456;
    expected.ud_dist_count      = -789;

    const auto message = serialize(expected);
    return makeGenoutRecord(message.data(), message.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());

  auto it                 = message_parser.begin();
  const auto dmi_raw_data = it->as<DmiRawData>();
  ASSERT_EQ(dmi_raw_data.gps_time.week, 2122);
  ASSERT_EQ(dmi_raw_data.gps_time.time_msec, 162117007u);
  ASSERT_EQ(dmi_raw_data.num_raw_meas, 10);
  ASSERT_EQ(dmi_raw_data.time_offset, 25);
  ASSERT_EQ(dmi_raw_data.abs_dist_count, 123456ul);
  ASSERT_EQ(dmi_raw_data.ud_dist_count, -789);

  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, iterator) {
  using namespace trmb::gsof;

  const auto record = [] {
    NavigationSolution solution{};
    solution.header.type   = GSOF_ID_49_INS_FULL_NAV;
    solution.header.length = sizeof(NavigationSolution) - sizeof(Header);
    solution.gps_time.week = 2080;

    NavigationPerformance rms{};
    rms.header.type   = GSOF_ID_50_INS_RMS;
    rms.header.length = sizeof(NavigationPerformance) - sizeof(Header);
    rms.gps_time.week = 2080;

    // A single GENOUT record can carry back to back messages
    auto messages           = serialize(solution);
    const auto rms_messages = serialize(rms);
    messages.insert(messages.end(), rms_messages.begin(), rms_messages.end());

    return makeGenoutRecord(messages.data(), messages.size());
  }();

  PublicPacketParser gsof_parser(record.data(), record.size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, streamParser) {
  using namespace trmb::gsof;

  const auto record = [] {
    NavigationSolution solution{};
    solution.header.type   = GSOF_ID_49_INS_FULL_NAV;
    solution.header.length = sizeof(NavigationSolution) - sizeof(Header);
    solution.gps_time.week = 2080;

    NavigationPerformance rms{};
    rms.header.type   = GSOF_ID_50_INS_RMS;
    rms.header.length = sizeof(NavigationPerformance) - sizeof(Header);
    rms.gps_time.week = 2080;

    auto messages           = serialize(solution);
    const auto rms_messages = serialize(rms);
    messages.insert(messages.end(), rms_messages.begin(), rms_messages.end());

    return makeGenoutRecord(messages.data(), messages.size());
  }();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distribution(1, 512);

  std::array<std::uint8_t, 512> buf{};
  StreamPageParser stream_parser;
  std::optional<std::vector<std::byte>> result = std::nullopt;

  stream_parser.registerGsofPageFoundCallback([&record, &result](const std::vector<std::byte> &page) {
    ASSERT_EQ(page.size(), record.size());
    result = page;
  });

  // Pass on random sized buffers to the stream parser
  for (std::size_t bytes_read = 0; bytes_read < record.size();) {
    int bytes_to_read = std::min(distribution(gen), static_cast<int>(record.size() - bytes_read));
    std::memcpy(buf.data(), record.data() + bytes_read, bytes_to_read);
    bytes_read += bytes_to_read;

    stream_parser.readSome(buf.data(), bytes_to_read);
  }

  // Now prove that you can parse the resulting vector of bytes using the packet parser
  ASSERT_TRUE(result.has_value());  // For -Wmaybe-uninitialized

  PublicPacketParser gsof_parser(result->data(), result->size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}
