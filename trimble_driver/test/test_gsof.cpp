/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <chrono>

#include "streaming_test.h"
#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"

using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

static constexpr char k_apx18_ip_addr[]    = "238.0.0.1";
static constexpr int k_apx18_port          = 5018;
static constexpr char k_applus60_ip_addr[] = "238.0.0.1";
static constexpr int k_applus60_port       = 5018;

class Applus60Gsof : public ::testing::Test {
 public:
  std::optional<PublicPacketParser> openPcap(const std::string &filename) {
    pcap_reader_.emplace(filename, k_applus60_ip_addr, k_applus60_port, network::ProtocolType::TCP);
    if (!(*pcap_reader_)) return std::nullopt;

    pcap_reader_->readSingle(&payload_);
    if (payload_.data == nullptr) return std::nullopt;

    return PacketParser(reinterpret_cast<const std::byte *>(payload_.data), payload_.length);
  }

 protected:
  std::optional<network::PcapReader> pcap_reader_ = std::nullopt;
  network::NonOwningBuffer payload_{nullptr, 0};
};

TEST_F(Applus60Gsof, positionTimeInfoGsof1) {
  auto maybe_gsof_parser = openPcap("applus60_gsof1.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x01);
  ASSERT_EQ(it->getHeader().length, 10);

  using namespace trmb::gsof;
  auto position_time = it->as<PositionTimeInfo>();
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

TEST_F(Applus60Gsof, latLongHeightGsof2) {
  auto maybe_gsof_parser = openPcap("applus60_gsof2.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x02);
  ASSERT_EQ(header.length, 24);

  using namespace trmb::gsof;
  const auto lla = it->as<LatLongHeight>();

  // Using approximate office coordinates
  ASSERT_NEAR(lla.lla.latitude, 0.77, 1e-2);
  ASSERT_NEAR(lla.lla.longitude, -1.39, 1e-2);
  ASSERT_NEAR(lla.lla.altitude, 168, 1);
}

TEST_F(Applus60Gsof, ecefPositionGsof3) {
  auto maybe_gsof_parser = openPcap("applus60_gsof3.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x03);
  ASSERT_EQ(header.length, 24);

  using namespace trmb::gsof;
  const auto ecef = it->as<EcefPosition>();

  // Looked at GUI and converted approximate coordinates of degrees-minutes-seconds to ecef
  ASSERT_NEAR(ecef.pos.x, 848942.99, 10);
  ASSERT_NEAR(ecef.pos.y, -4527384.15, 10);
  ASSERT_NEAR(ecef.pos.z, 4397104.00, 10);
}

TEST_F(Applus60Gsof, ecefDeltaGsof6) {
  auto maybe_gsof_parser = openPcap("applus60_gsof6.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x06);
  ASSERT_EQ(header.length, 24);

  using namespace trmb::gsof;
  const auto ecef_delta = it->as<EcefDelta>();

  // Converted from bytes shown in wireshark
  // RTCMv3 input was from apply-bb-01.ddns.net:2101/Downtown_Base_RTCM_32_MSM4
  ASSERT_NEAR(ecef_delta.delta.x, -3473.5847978937672, 1e-4);
  ASSERT_NEAR(ecef_delta.delta.y, 10602.019044206478, 1e-4);
  ASSERT_NEAR(ecef_delta.delta.z, 11631.403884239495, 1e-4);
}

TEST_F(Applus60Gsof, tplaneDeltaGsof7) {
  auto maybe_gsof_parser = openPcap("applus60_gsof7.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x07);
  ASSERT_EQ(header.length, 24);

  using namespace trmb::gsof;
  const auto tplane_delta = it->as<TangentPlaneDelta>();

  // Converted from bytes shown in wireshark
  // RTCMv3 input was from appl-bb-01.ddns.net:2101/Downtown_Base_RTCM_32_MSM4
  ASSERT_NEAR(tplane_delta.enu.east, -1456.685759244824, 1e-4);
  ASSERT_NEAR(tplane_delta.enu.north, 16051.050661617326, 1e-4);
  ASSERT_NEAR(tplane_delta.enu.up, 43.853000930700546, 1e-4);
}

TEST_F(Applus60Gsof, velocityGsof8) {
  auto maybe_gsof_parser = openPcap("applus60_gsof8.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x08);
  ASSERT_EQ(header.length, 13);

  using namespace trmb::gsof;
  const auto velocity = it->as<Velocity>();

  // Converted from bytes shown in wireshark
  ASSERT_FLOAT_EQ(velocity.velocity, 0.00597169);
  ASSERT_FLOAT_EQ(velocity.heading, 2.062232);
  ASSERT_FLOAT_EQ(velocity.vertical_velocity, -0.0006348496);
  ASSERT_FALSE(velocity.local_heading);  // No local heading

  ASSERT_TRUE(velocity.isVelDataValid());
  ASSERT_EQ(velocity.getVelocitySource(), Velocity::VelocitySource::k_consecutive_measurements);
  ASSERT_TRUE(velocity.isHeadingDataValid());
}

TEST_F(Applus60Gsof, pdopInfoGsof9) {
  auto maybe_gsof_parser = openPcap("applus60_gsof9.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 0x09);
  ASSERT_EQ(header.length, 16);

  using namespace trmb::gsof;
  const auto dop = it->as<PdopInfo>();

  // Converted from bytes shown in wireshark
  ASSERT_FLOAT_EQ(dop.position_dop, 0.9000375);
  ASSERT_FLOAT_EQ(dop.horiziontal_dop, 0.488651454);
  ASSERT_FLOAT_EQ(dop.vertical_dop, 0.7558355);
  ASSERT_FLOAT_EQ(dop.time_dop, 1.13949406);
}

TEST_F(Applus60Gsof, clockInfoGsof10) {
  auto maybe_gsof_parser = openPcap("applus60_gsof10.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 10);
  ASSERT_EQ(header.length, 17);

  using namespace trmb::gsof;
  const auto clock = it->as<ClockInfo>();

  ASSERT_TRUE(clock.isClockOffsetValid());
  ASSERT_TRUE(clock.isFreqOffsetValid());
  ASSERT_TRUE(clock.isReceiverInAnywhereFixMode());

  ASSERT_NEAR(clock.clock_offset, 0.007924753666675877, 1e-5);
  ASSERT_NEAR(clock.freq_offset, -1.6607935199691757, 1e-5);
}

TEST_F(Applus60Gsof, positionVcvGsof11) {
  auto maybe_gsof_parser = openPcap("applus60_gsof11.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 11);
  ASSERT_EQ(header.length, 34);

  using namespace trmb::gsof;
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

TEST_F(Applus60Gsof, positionSigmaGsof12) {
  auto maybe_gsof_parser = openPcap("applus60_gsof12.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 12);
  ASSERT_EQ(header.length, 38);

  using namespace trmb::gsof;
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

TEST_F(Applus60Gsof, receiverSerialNumberGsof15) {
  auto maybe_gsof_parser = openPcap("applus60_gsof15.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 15);
  ASSERT_EQ(header.length, 4);

  using namespace trmb::gsof;
  const auto serial = it->as<ReceiverSerialNumber>();

  ASSERT_EQ(serial.number, 573901879);
}

TEST_F(Applus60Gsof, currentTimeGsof16) {
  auto maybe_gsof_parser = openPcap("applus60_gsof16.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 16);
  ASSERT_EQ(header.length, 9);

  using namespace trmb::gsof;
  const auto time = it->as<CurrentTime>();

  ASSERT_EQ(time.gps_ms_week, 162117007u);
  ASSERT_EQ(time.gps_week, 2225u);
  ASSERT_EQ(time.utc_offset, 18u);

  ASSERT_TRUE(time.isTimeInfoValid());
  ASSERT_TRUE(time.isUtcOffsetValid());
}

TEST_F(Applus60Gsof, attitudeInfoGsof27) {
  auto maybe_gsof_parser = openPcap("applus60_gsof27.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 27);
  ASSERT_EQ(header.length, 70);

  using namespace trmb::gsof;
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

TEST_F(Applus60Gsof, allSvBriefInfoGsof33) {
  auto maybe_gsof_parser = openPcap("applus60_gsof33.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 33);
  ASSERT_EQ(header.length, 101);

  using namespace trmb::gsof;
  const auto brief_sv_info = it->as<AllSvBrief>();
  ASSERT_EQ(brief_sv_info.num_svs, 25);
  ASSERT_EQ(brief_sv_info.sv_info.size(), 25ul);
  const std::array<uint8_t, 25> expected_prn                  = {0x0b, 0x19, 0x0c, 0x17, 0x05, 0x1d, 0x0d, 0x02, 0x08,
                                                                 0x07, 0x06, 0x14, 0x14, 0x0f, 0x03, 0x29, 0x0d, 0x20,
                                                                 0x0f, 0x15, 0x1b, 0x07, 0x08, 0x1a, 0x1e};
  const std::array<SatelliteType, 25> expected_satellite_type = {
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GLONASS,
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GPS,
      SatelliteType::GLONASS, SatelliteType::GLONASS, SatelliteType::GPS,     SatelliteType::BEIDOU,
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GALILEO, SatelliteType::BEIDOU,
      SatelliteType::GALILEO, SatelliteType::BEIDOU,  SatelliteType::GALILEO, SatelliteType::GALILEO,
      SatelliteType::BEIDOU,  SatelliteType::GALILEO, SatelliteType::GALILEO, SatelliteType::GALILEO,
      SatelliteType::BEIDOU};

  for (std::size_t i = 0; i < 25; ++i) {
    const auto &info = brief_sv_info.sv_info[i];
    ASSERT_EQ(info.prn, expected_prn[i]);
    ASSERT_EQ(info.getSVSystemMode(), expected_satellite_type[i]);
    ASSERT_TRUE(info.isAboveHor());
    ASSERT_TRUE(info.isAssignedToChannel());
    ASSERT_TRUE(info.isTracked());
  }
}

TEST_F(Applus60Gsof, allSvDetailedInfoGsof34) {
  auto maybe_gsof_parser = openPcap("applus60_gsof34.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 34);
  ASSERT_EQ(header.length, 0xf1);

  using namespace trmb::gsof;
  const auto sv_info = it->as<AllSvDetailed>();
  ASSERT_EQ(sv_info.num_svs, 24);
  ASSERT_EQ(sv_info.sv_info.size(), 24ul);
  const std::array<uint8_t, 24> expected_prn = {0x17, 0x12, 0x17, 0x05, 0x1d, 0x0d, 0x18, 0x02, 0x08, 0x01, 0x14, 0x1e,
                                                0x14, 0x0f, 0x0d, 0x20, 0x1d, 0x21, 0x07, 0x08, 0x1a, 0x1e, 0x17, 0x14};
  const std::array<SatelliteType, 24> expected_satellite_type = {
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GLONASS,  SatelliteType::GPS,
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GLONASS,  SatelliteType::GPS,
      SatelliteType::GLONASS, SatelliteType::GLONASS, SatelliteType::BEIDOU,   SatelliteType::GPS,
      SatelliteType::GPS,     SatelliteType::GPS,     SatelliteType::GALILEO,  SatelliteType::BEIDOU,
      SatelliteType::BEIDOU,  SatelliteType::GALILEO, SatelliteType::GALILEO,  SatelliteType::GALILEO,
      SatelliteType::GALILEO, SatelliteType::BEIDOU,  SatelliteType::OMNISTAR, SatelliteType::BEIDOU};

  const std::array<bool, 24> expected_dual_freq_tracking = {true,  true, false, false, true,  true,  false, true,
                                                            true,  true, false, true,  false, true,  true,  false,
                                                            false, true, true,  true,  true,  false, false, false};

  const std::array<bool, 24> expected_l1g1_freq_reports = {true, true, true, true, true, true, true,  true,
                                                           true, true, true, true, true, true, true,  true,
                                                           true, true, true, true, true, true, false, true};

  const std::array<bool, 24> expected_l1g2_freq_reports = {true, true, false, true, true, true, true,  true,
                                                           true, true, true,  true, true, true, true,  true,
                                                           true, true, true,  true, true, true, false, true};

  const std::array<bool, 24> expected_used_at_current_pos = {false, true, true, true, true, true, false, true,
                                                             false, true, true, true, true, true, true,  true,
                                                             true,  true, true, true, true, true, false, true};

  const std::array<bool, 24> expected_used_in_rtk_solution = {false, false, false, false, true,  false, false, false,
                                                              false, false, false, false, false, false, false, false,
                                                              true,  false, true,  false, false, false, false, false};

  const std::array<bool, 24> expected_pcode_l1g1 = {false, false, false, false, false, false, false, false,
                                                    false, false, true,  false, false, false, true,  true,
                                                    true,  true,  true,  true,  true,  true,  false, false};

  const std::array<bool, 24> expected_cs_l2_tracking = {true,  true,  false, false, true,  false, false, false,
                                                        false, false, false, true,  false, true,  false, false,
                                                        false, false, false, false, false, false, false, false};

  const std::array<bool, 24> expected_l5_tracking = {true,  true,  false, false, false, false, false, false,
                                                     false, false, false, true,  false, false, false, false,
                                                     false, false, false, false, false, false, false, false};

  const std::array<bool, 24> expected_glonass_mtype = {false, false, true,  false, false, false, true,  false,
                                                       true,  true,  false, false, false, false, false, false,
                                                       false, false, false, false, false, false, false, false};

  for (std::size_t i = 0; i < 24; ++i) {
    const auto &info = sv_info.sv_info[i];
    ASSERT_EQ(info.prn, expected_prn[i]);
    ASSERT_EQ(info.getSvType(), expected_satellite_type[i]);

    if (info.getSvType() != SatelliteType::OMNISTAR) {
      ASSERT_TRUE(info.isAboveHor());
      ASSERT_TRUE(info.isAssignedToChannel());
      ASSERT_TRUE(info.isTracked());
    } else {
      ASSERT_FALSE(info.isAboveHor());
      ASSERT_FALSE(info.isAssignedToChannel());
      ASSERT_FALSE(info.isTracked());
    }

    ASSERT_EQ(info.isCurrTrackedDual(), expected_dual_freq_tracking[i]);
    ASSERT_EQ(info.isL1G1Freq(), expected_l1g1_freq_reports[i]);
    ASSERT_EQ(info.isL1G2Freq(), expected_l1g2_freq_reports[i]);
    ASSERT_EQ(info.isUsedAtCurrentPos(), expected_used_at_current_pos[i]);
    ASSERT_EQ(info.isUsedInRtkSolution(), expected_used_in_rtk_solution[i]);

    ASSERT_EQ(info.isTrackPCodeL1G1(), expected_pcode_l1g1[i]);
    ASSERT_FALSE(info.isTrackPCodeL2());
    ASSERT_EQ(info.isTrackingCsOnL2(), expected_cs_l2_tracking[i]) << "with i " << i << " and " << int(info.sv_system);
    ASSERT_EQ(info.isTrackingL5Signal(), expected_l5_tracking[i]);

    ASSERT_EQ(info.isGlonassMSv(), expected_glonass_mtype[i]);
    ASSERT_FALSE(info.isGlonassKSv());
  }
}

TEST_F(Applus60Gsof, receivedBaseInfoGsof35) {
  auto maybe_gsof_parser = openPcap("applus60_gsof35.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 35);
  ASSERT_EQ(header.length, 35);

  using namespace trmb::gsof;
  const auto base_info = it->as<ReceivedBaseInfo>();

  const std::array<char, 8> expected_name = {'R', 'T', 'C', 'M', '0', '0', '0', '0'};
  ASSERT_EQ(expected_name, base_info.name);
  ASSERT_EQ(base_info.getVersionNumber(), 0);
  ASSERT_EQ(base_info.id, 0);
  ASSERT_TRUE(base_info.isBaseInfoValid());

  ASSERT_DOUBLE_EQ(base_info.base_lla.latitude, 0.7630018859478892);
  ASSERT_DOUBLE_EQ(base_info.base_lla.longitude, -1.385119519554126);
  ASSERT_DOUBLE_EQ(base_info.base_lla.altitude, 101.22497844472646);
}

TEST_F(Applus60Gsof, batteryMemoryInfoGsof37) {
  auto maybe_gsof_parser = openPcap("applus60_gsof37.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 37);
  ASSERT_EQ(header.length, 10);

  using namespace trmb::gsof;
  const auto battery = it->as<BatteryMemoryInfo>();

  ASSERT_EQ(battery.battery_cap, 100);
  ASSERT_DOUBLE_EQ(battery.remaining_time, 1343.1210074074074);
}

TEST_F(Applus60Gsof, positionTypeGsof38) {
  auto maybe_gsof_parser = openPcap("applus60_gsof38.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 38);
  ASSERT_EQ(header.length, 26);

  using namespace trmb::gsof;
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

TEST_F(Applus60Gsof, lbandStatusGsof40) {
  auto maybe_gsof_parser = openPcap("applus60_gsof40.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 40);
  ASSERT_EQ(header.length, 70);

  using namespace trmb::gsof;
  const auto lband_status = it->as<LbandStatusInfo>();

  const std::array<char, 5> expected_name = {'R', 'T', 'X', 'N', 'A'};
  ASSERT_EQ(lband_status.satellite_name, expected_name);

  // This frequency and baud rate were taken from https://positioningservices.trimble.com/resources/sat/ it is the
  // North America RTX beam frequency and baud
  ASSERT_FLOAT_EQ(lband_status.nominal_sat_freq, 1555.8080f);
  ASSERT_EQ(lband_status.sat_bit_rate, 2400);

  ASSERT_NEAR(static_cast<double>(lband_status.c_no), 47.0, 0.1);  // GUI just says 47
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
TEST_F(Applus60Gsof, basePositionQualityGsof41) {
  auto maybe_gsof_parser = openPcap("applus60_gsof41.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();

  const auto &header = it->getHeader();
  ASSERT_EQ(header.type, 41);
  ASSERT_EQ(header.length, 31);

  using namespace trmb::gsof;
  const auto base_qual = it->as<BasePositionAndQualityIndicator>();

  ASSERT_EQ(base_qual.gps_time_ms, 328103007u);
  ASSERT_EQ(base_qual.gps_week_number, 2225u);
  ASSERT_DOUBLE_EQ(base_qual.llh.latitude, 0.7630018859478892);
  ASSERT_DOUBLE_EQ(base_qual.llh.longitude, -1.385119519554126);
  ASSERT_DOUBLE_EQ(base_qual.llh.altitude, 101.22497844472646);
  ASSERT_EQ(base_qual.getBaseQuality(), BaseQuality::k_omnistar_xp_hp_or_rtk_float);
}

TEST(GsofParsingTest, fullNavigationInfoGsof49) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcap", k_apx18_ip_addr, k_apx18_port,
                                  network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x31);
  ASSERT_EQ(it->getHeader().length, 0x68);  // Read our ICD not the GSOF stuff on the web

  using namespace trmb::gsof;
  auto solution = it->as<NavigationSolution>();
  ASSERT_EQ(solution.gps_time.week, 2080);  // week of nov 17th 2019
  // Guesstimated Google Maps lat long of Trimble Applanix Richmond Hill - Canada office 😄
  ASSERT_NEAR(solution.lla.latitude, 43.86, 1e-2);
  ASSERT_NEAR(solution.lla.longitude, -79.38, 1e-2);

  ASSERT_NE(solution.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::UNKNOWN);
  ASSERT_NE(solution.status.getGnssStatus(), Status::GnssStatus::UNKNOWN);
}

TEST(GsofParsingTest, fullNavigationRmsGsof50) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullrmsinfo_single.pcap", k_apx18_ip_addr, k_apx18_port,
                                  network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());

  auto it = message_parser.begin();
  using namespace trmb::gsof;
  auto rms = it->as<NavigationPerformance>();
  ASSERT_EQ(rms.gps_time.week, 2080);

  ASSERT_NE(rms.status.getImuAlignmentStatus(), Status::ImuAlignmentStatus::UNKNOWN);
  ASSERT_NE(rms.status.getGnssStatus(), Status::GnssStatus::UNKNOWN);

  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, dmiRawDataGsof52) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_gsof52_dmirawdata_single.pcap", k_apx18_ip_addr, k_apx18_port,
                                  network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());

  auto it = message_parser.begin();
  using namespace trmb::gsof;
  auto dmiRawData = it->as<DmiRawData>();
  ASSERT_EQ(dmiRawData.gps_time.week, 2122);
  ASSERT_EQ(dmiRawData.num_raw_meas, 10);
  ASSERT_EQ(dmiRawData.time_offset, 0);
  ASSERT_EQ(dmiRawData.abs_dist_count, 0ul);
  ASSERT_EQ(dmiRawData.ud_dist_count, 0);

  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, iterator) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcap", k_apx18_ip_addr, k_apx18_port,
                                  network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  PacketParser gsof_parser(reinterpret_cast<const std::byte *>(payload.data), payload.length);

  using namespace trmb;
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}

TEST(GsofParsingTest, streamParser) {
  network::NonOwningBuffer payload{nullptr, 0};
  network::PcapReader pcap_reader("apx_18_fullnavinfo_single.pcap", k_apx18_ip_addr, k_apx18_port,
                                  network::ProtocolType::TCP);
  ASSERT_TRUE(static_cast<bool>(pcap_reader));
  ASSERT_TRUE(pcap_reader.readSingle(&payload));
  ASSERT_NE(payload.data, nullptr);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distribution(1, 512);

  std::array<std::uint8_t, 512> buf{};
  trmb::gsof::StreamPageParser stream_parser;
  std::optional<std::vector<std::byte>> result = std::nullopt;

  stream_parser.registerGsofPageFoundCallback([&payload, &result](const std::vector<std::byte> &page) {
    ASSERT_EQ(page.size(), payload.length);
    result = page;
  });

  // Pass on random sized buffers to the stream parser
  for (std::size_t bytes_read = 0; bytes_read < payload.length;) {
    int bytes_to_read = std::min(distribution(gen), static_cast<int>(payload.length - bytes_read));
    std::memcpy(buf.data(), payload.data + bytes_read, bytes_to_read);
    bytes_read += bytes_to_read;

    stream_parser.readSome(buf.data(), bytes_to_read);
  }

  // Now prove that you can parse the resulting vector of bytes using the packet parser
  ASSERT_TRUE(result.has_value());  // For -Wmaybe-uninitialized

  PacketParser gsof_parser(result->data(), result->size());
  ASSERT_TRUE(gsof_parser.isValid());
  ASSERT_TRUE(gsof_parser.isSupported());

  using namespace trmb;
  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_49_INS_FULL_NAV);
  ++it;
  ASSERT_EQ(it->getHeader().type, gsof::GSOF_ID_50_INS_RMS);
  ++it;
  ASSERT_EQ(it, message_parser.end());
}
