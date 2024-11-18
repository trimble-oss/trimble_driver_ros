/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "streaming_test.h"
#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver/gsof/stream_chapter_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"

using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

static constexpr char k_apx18_ip_addr[] = "238.0.0.1";
static constexpr int k_apx18_port       = 5018;

class Apx18StreamingGsofTest : public StreamingTest {
 public:
  Apx18StreamingGsofTest()
      : StreamingTest("apx_18_fullnav_fullrmsinfo.pcap", k_apx18_ip_addr, k_apx18_port, network::ProtocolType::TCP) {}
};

TEST_F(Apx18StreamingGsofTest, streamParserWithOverlap) {
  trmb::gsof::StreamPageParser stream_parser;
  std::optional<network::NonOwningBuffer> maybe_data = std::nullopt;

  stream_parser.registerGsofPageFoundCallback([](const std::vector<std::byte> &page) {
    PacketParser packet_parser(page.data(), page.size());
    ASSERT_TRUE(packet_parser.isValid());
    ASSERT_TRUE(packet_parser.isSupported());

    // Every record in this pcap contains a full ins nav and the associated RMS error values
    auto msg_parser = packet_parser.getMessageParser();
    ASSERT_TRUE(msg_parser.isSupported());
    ASSERT_TRUE(msg_parser.isValid());

    auto it = msg_parser.begin();
    ASSERT_EQ(it->getHeader().type, trmb::gsof::GSOF_ID_49_INS_FULL_NAV);
    ++it;
    ASSERT_EQ(it->getHeader().type, trmb::gsof::GSOF_ID_50_INS_RMS);
    ++it;
    ASSERT_EQ(it, msg_parser.end());
  });

  for (maybe_data = getData(); maybe_data.has_value(); maybe_data = getData()) {
    stream_parser.readSome(maybe_data->data, maybe_data->length);
  }
}

static constexpr char applus20_ip_addr[] = "238.0.0.1";
static constexpr int applus20_port       = 5018;

/**
 * The AP+ 20 large tx pcap contains 3 DCOL-GENOUT transmissions each split over two packets. The packets should
 * contain GSOF messages 1, 2, 3, 6, 8, 9, 10, 11, 12 and 14.
 */
class ApPlus20StreamingGsofTest : public StreamingTest {
 public:
  ApPlus20StreamingGsofTest()
      : StreamingTest("applus20_large_tx.pcap", applus20_ip_addr, applus20_port, network::ProtocolType::TCP) {}
};

TEST_F(ApPlus20StreamingGsofTest, parseMultiPacketTransmission) {
  trmb::gsof::StreamChapterParser parser;
  std::optional<network::NonOwningBuffer> maybe_data = std::nullopt;

  const std::set<std::uint8_t> expected_gsof_message_ids = {1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 15, 16};

  using namespace trmb::gsof;

  // The local heading field is not present because a planar local coordinate was not loaded, therefore the Velocity
  // message size is 15 not 19.
  const std::size_t k_expected_gsof_velocity_message_size = 15;
  const std::size_t k_expected_payload_size =
      sizeof(PositionTimeInfo) + sizeof(LatLongHeight) + sizeof(EcefPosition) + sizeof(EcefDelta) +
      sizeof(TangentPlaneDelta) + k_expected_gsof_velocity_message_size + sizeof(PdopInfo) + sizeof(ClockInfo) +
      sizeof(PositionVcvInfo) + sizeof(PositionSigmaInfo) + sizeof(ReceiverSerialNumber) + sizeof(CurrentTime);

  parser.registerGsofChapterFoundCallback(
      [k_expected_payload_size, &expected_gsof_message_ids](const std::vector<std::byte> &chapter) {
        ASSERT_EQ(chapter.size(), k_expected_payload_size);
        trmb::gsof::MessageParser message_parser(chapter.data(), chapter.size());

        int parsed_message_count = 0;
        for (const auto &message : message_parser) {
          const trmb::gsof::Header &header = message.getHeader();
          ASSERT_TRUE(expected_gsof_message_ids.count(header.type) > 0)
              << "Unexpectedly encountered message of type " << std::to_string(header.type) << ".";
          ++parsed_message_count;

          // The following are just short sanity checks as the actual full parsing tests are in a different unit test
          if (header.type == GSOF_ID_1_POS_TIME) {
            auto position_time = message.as<PositionTimeInfo>();
            ASSERT_EQ(position_time.gps_week, 2226);
          } else if (header.type == GSOF_ID_2_LLH) {
            auto llh = message.as<LatLongHeight>();
            ASSERT_NEAR(llh.lla.altitude, 165, 1);
          } else if (header.type == GSOF_ID_3_ECEF) {
            auto ecef = message.as<EcefPosition>();
            ASSERT_GT(ecef.pos.z, 0);
          } else if (header.type == GSOF_ID_6_ECEF_DELTA) {
            auto ecef = message.as<EcefDelta>();
            ASSERT_GT(ecef.delta.z, 0);
          } else if (header.type == GSOF_ID_7_TPLANE_ENU) {
            auto tplane_enu_delta = message.as<TangentPlaneDelta>();
            ASSERT_DOUBLE_EQ(tplane_enu_delta.enu.east, 0.0);
            ASSERT_DOUBLE_EQ(tplane_enu_delta.enu.north, 0.0);
            ASSERT_DOUBLE_EQ(tplane_enu_delta.enu.up, 0.0);
          } else if (header.type == GSOF_ID_8_VELOCITY) {
            auto vel = message.as<Velocity>();
            ASSERT_NEAR(vel.velocity, 0.0, 0.1);
            ASSERT_NEAR(vel.vertical_velocity, 0.0, 0.1);
          } else if (header.type == GSOF_ID_9_DOP) {
            auto dop = message.as<PdopInfo>();
            ASSERT_NEAR(dop.position_dop, 0.9f, 0.1);
          } else if (header.type == GSOF_ID_10_CLOCK_INFO) {
            auto clock = message.as<ClockInfo>();
            ASSERT_TRUE(clock.isClockOffsetValid());
            ASSERT_TRUE(clock.isFreqOffsetValid());
          } else if (header.type == GSOF_ID_11_POS_VCV_INFO) {
            auto vcv = message.as<PositionVcvInfo>();
            ASSERT_EQ(vcv.num_epochs, 0);
          } else if (header.type == GSOF_ID_12_POS_SIGMA) {
            auto sigma = message.as<PositionSigmaInfo>();
            ASSERT_EQ(sigma.number_epochs, 0);
          } else if (header.type == GSOF_ID_15_REC_SERIAL_NUM) {
            auto serial = message.as<ReceiverSerialNumber>();
            ASSERT_EQ(serial.number, 573901788);
          } else if (header.type == GSOF_ID_16_CURR_TIME) {
            auto time = message.as<CurrentTime>();
            ASSERT_EQ(time.gps_week, 2226);
          }
        }
        ASSERT_EQ(parsed_message_count, expected_gsof_message_ids.size())
            << "Parsed message count is different from the amount of messages in the GSOF stream.";
      });

  for (maybe_data = getData(); maybe_data.has_value(); maybe_data = getData()) {
    parser.readSome(maybe_data->data, maybe_data->length);
  }
}

class ApPlus20SmallStreamingGsofTest : public StreamingTest {
 public:
  ApPlus20SmallStreamingGsofTest()
      : StreamingTest("applus20_small_tx.pcap", applus20_ip_addr, applus20_port, network::ProtocolType::TCP) {}
};

TEST_F(ApPlus20SmallStreamingGsofTest, singlePageStreamTest) {
  // This test checks if the multipage stream parser works properly with a data stream that doesn't get split over
  // multiple pages
  trmb::gsof::StreamChapterParser parser;
  std::optional<network::NonOwningBuffer> maybe_data = std::nullopt;

  constexpr std::size_t k_expected_chapters = 10;
  std::size_t chapter_count                 = 0;
  uint32_t last_time                        = 0;

  parser.registerGsofChapterFoundCallback([&chapter_count, &last_time](const std::vector<std::byte> &chapter) {
    trmb::gsof::MessageParser message_parser(chapter.data(), chapter.size());

    auto it = message_parser.begin();
    ASSERT_EQ(it->getHeader().type, 49);

    // Check time is monotonic
    auto nav = it->as<trmb::gsof::NavigationSolution>();
    ASSERT_GT(nav.gps_time.time_msec, last_time);
    last_time = nav.gps_time.time_msec;

    ++it;
    ASSERT_EQ(it->getHeader().type, 50);
    ++chapter_count;
  });

  for (maybe_data = getData(); maybe_data.has_value(); maybe_data = getData()) {
    parser.readSome(maybe_data->data, maybe_data->length);
  }

  ASSERT_EQ(chapter_count, k_expected_chapters);
}
