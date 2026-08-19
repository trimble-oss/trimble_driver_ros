/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <vector>

#include "gsof_record_builder.h"
#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver/gsof/stream_chapter_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"

using gsof_test::makeGenoutRecord;
using gsof_test::makeGenoutTransmission;
using gsof_test::serialize;
using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

namespace {

/**
 * @brief Builds a GENOUT record holding a full INS nav solution and its associated RMS error values.
 */
std::vector<std::byte> makeNavAndRmsRecord(uint32_t gps_time_msec) {
  using namespace trmb::gsof;

  NavigationSolution solution{};
  solution.header.type        = GSOF_ID_49_INS_FULL_NAV;
  solution.header.length      = sizeof(NavigationSolution) - sizeof(Header);
  solution.gps_time.week      = 2226;
  solution.gps_time.time_msec = gps_time_msec;

  NavigationPerformance rms{};
  rms.header.type        = GSOF_ID_50_INS_RMS;
  rms.header.length      = sizeof(NavigationPerformance) - sizeof(Header);
  rms.gps_time.week      = 2226;
  rms.gps_time.time_msec = gps_time_msec;

  auto messages           = serialize(solution);
  const auto rms_messages = serialize(rms);
  messages.insert(messages.end(), rms_messages.begin(), rms_messages.end());

  return makeGenoutRecord(messages.data(), messages.size());
}

/**
 * @brief Feeds a byte stream to a parser in chunks that do not line up with the record boundaries.
 */
template <class ParserType>
void readInChunks(ParserType &parser, const std::vector<std::byte> &stream, std::size_t chunk_size) {
  const auto *data = reinterpret_cast<const std::uint8_t *>(stream.data());
  for (std::size_t offset = 0; offset < stream.size(); offset += chunk_size) {
    parser.readSome(data + offset, std::min(chunk_size, stream.size() - offset));
  }
}

}  // namespace

TEST(GsofStreamingTest, streamParserWithOverlap) {
  constexpr std::size_t k_expected_pages = 5;
  constexpr std::size_t k_chunk_size     = 37;  // Deliberately unaligned with the record size

  const auto stream = [] {
    std::vector<std::byte> bytes;
    for (std::size_t i = 0; i < k_expected_pages; ++i) {
      const auto record = makeNavAndRmsRecord(static_cast<uint32_t>(1000 * (i + 1)));
      bytes.insert(bytes.end(), record.begin(), record.end());
    }
    return bytes;
  }();

  trmb::gsof::StreamPageParser stream_parser;
  std::size_t page_count = 0;

  stream_parser.registerGsofPageFoundCallback([&page_count](const std::vector<std::byte> &page) {
    PacketParser packet_parser(page.data(), page.size());
    ASSERT_TRUE(packet_parser.isValid());
    ASSERT_TRUE(packet_parser.isSupported());

    // Every record in this stream contains a full ins nav and the associated RMS error values
    auto msg_parser = packet_parser.getMessageParser();
    ASSERT_TRUE(msg_parser.isSupported());
    ASSERT_TRUE(msg_parser.isValid());

    auto it = msg_parser.begin();
    ASSERT_EQ(it->getHeader().type, trmb::gsof::GSOF_ID_49_INS_FULL_NAV);
    ++it;
    ASSERT_EQ(it->getHeader().type, trmb::gsof::GSOF_ID_50_INS_RMS);
    ++it;
    ASSERT_EQ(it, msg_parser.end());

    ++page_count;
  });

  readInChunks(stream_parser, stream, k_chunk_size);

  ASSERT_EQ(page_count, k_expected_pages);
}

/**
 * The transmissions below hold more messages than a single page can carry, so each one gets split over two pages.
 * The messages are GSOF 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 15 and 16.
 */
TEST(GsofStreamingTest, parseMultiPacketTransmission) {
  using namespace trmb::gsof;

  constexpr std::size_t k_expected_transmissions = 3;
  constexpr std::size_t k_chunk_size             = 53;  // Deliberately unaligned with the page size

  const std::set<std::uint8_t> expected_gsof_message_ids = {1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 15, 16};

  // The local heading field is not present because a planar local coordinate was not loaded, therefore the Velocity
  // message size is 15 not 19.
  const std::size_t k_expected_gsof_velocity_message_size = 15;
  const std::size_t k_expected_payload_size =
      sizeof(PositionTimeInfo) + sizeof(LatLongHeight) + sizeof(EcefPosition) + sizeof(EcefDelta) +
      sizeof(TangentPlaneDelta) + k_expected_gsof_velocity_message_size + sizeof(PdopInfo) + sizeof(ClockInfo) +
      sizeof(PositionVcvInfo) + sizeof(PositionSigmaInfo) + sizeof(ReceiverSerialNumber) + sizeof(CurrentTime);

  const auto gsof_messages = [k_expected_gsof_velocity_message_size] {
    std::vector<std::vector<std::byte>> messages;

    PositionTimeInfo position_time{};
    position_time.header.type                = GSOF_ID_1_POS_TIME;
    position_time.header.length              = sizeof(PositionTimeInfo) - sizeof(Header);
    position_time.gps_time_ms                = 328103007;
    position_time.gps_week                   = 2226;
    position_time.number_space_vehicles_used = 25;
    messages.push_back(serialize(position_time));

    LatLongHeight llh{};
    llh.header.type   = GSOF_ID_2_LLH;
    llh.header.length = sizeof(LatLongHeight) - sizeof(Header);
    llh.lla.latitude  = 0.7654321;
    llh.lla.longitude = -1.3876543;
    llh.lla.altitude  = 165.0;
    messages.push_back(serialize(llh));

    EcefPosition ecef{};
    ecef.header.type   = GSOF_ID_3_ECEF;
    ecef.header.length = sizeof(EcefPosition) - sizeof(Header);
    ecef.pos.x         = 848942.99;
    ecef.pos.y         = -4527384.15;
    ecef.pos.z         = 4397104.00;
    messages.push_back(serialize(ecef));

    EcefDelta ecef_delta{};
    ecef_delta.header.type   = GSOF_ID_6_ECEF_DELTA;
    ecef_delta.header.length = sizeof(EcefDelta) - sizeof(Header);
    ecef_delta.delta.x       = -3473.5847978937672;
    ecef_delta.delta.y       = 10602.019044206478;
    ecef_delta.delta.z       = 11631.403884239495;
    messages.push_back(serialize(ecef_delta));

    TangentPlaneDelta tplane_delta{};
    tplane_delta.header.type   = GSOF_ID_7_TPLANE_ENU;
    tplane_delta.header.length = sizeof(TangentPlaneDelta) - sizeof(Header);
    messages.push_back(serialize(tplane_delta));

    // Velocity has an optional trailing field so it cannot be memcpy'd as a whole struct.
    std::vector<std::byte> velocity;
    const auto append = [&velocity](auto value) {
      if constexpr (sizeof(value) > 1) {
        byteswapInPlace(&value);
      }
      const auto *bytes = reinterpret_cast<const std::byte *>(&value);
      velocity.insert(velocity.end(), bytes, &bytes[sizeof(value)]);
    };
    append(static_cast<uint8_t>(GSOF_ID_8_VELOCITY));
    append(static_cast<uint8_t>(k_expected_gsof_velocity_message_size - sizeof(Header)));
    append(static_cast<uint8_t>(0b0000'0101));
    append(0.0f);  // velocity
    append(0.0f);  // heading
    append(0.0f);  // vertical velocity
    messages.push_back(velocity);

    PdopInfo dop{};
    dop.header.type     = GSOF_ID_9_DOP;
    dop.header.length   = sizeof(PdopInfo) - sizeof(Header);
    dop.position_dop    = 0.9f;
    dop.horiziontal_dop = 0.488651454f;
    dop.vertical_dop    = 0.7558355f;
    dop.time_dop        = 1.13949406f;
    messages.push_back(serialize(dop));

    ClockInfo clock{};
    clock.header.type   = GSOF_ID_10_CLOCK_INFO;
    clock.header.length = sizeof(ClockInfo) - sizeof(Header);
    clock.clock_flags   = 0b0000'0011;  // clock offset and frequency offset valid
    clock.clock_offset  = 0.007924753666675877;
    clock.freq_offset   = -1.6607935199691757;
    messages.push_back(serialize(clock));

    PositionVcvInfo vcv{};
    vcv.header.type   = GSOF_ID_11_POS_VCV_INFO;
    vcv.header.length = sizeof(PositionVcvInfo) - sizeof(Header);
    vcv.num_epochs    = 0;
    messages.push_back(serialize(vcv));

    PositionSigmaInfo sigma{};
    sigma.header.type   = GSOF_ID_12_POS_SIGMA;
    sigma.header.length = sizeof(PositionSigmaInfo) - sizeof(Header);
    sigma.number_epochs = 0;
    messages.push_back(serialize(sigma));

    ReceiverSerialNumber serial{};
    serial.header.type   = GSOF_ID_15_REC_SERIAL_NUM;
    serial.header.length = sizeof(ReceiverSerialNumber) - sizeof(Header);
    serial.number        = 573901788;
    messages.push_back(serialize(serial));

    CurrentTime time{};
    time.header.type   = GSOF_ID_16_CURR_TIME;
    time.header.length = sizeof(CurrentTime) - sizeof(Header);
    time.gps_ms_week   = 328103007;
    time.gps_week      = 2226;
    time.utc_offset    = 18;
    messages.push_back(serialize(time));

    return messages;
  }();

  const auto stream = [&gsof_messages] {
    std::vector<std::byte> bytes;
    for (std::size_t i = 0; i < k_expected_transmissions; ++i) {
      const auto transmission = makeGenoutTransmission(gsof_messages, static_cast<uint8_t>(i));
      bytes.insert(bytes.end(), transmission.begin(), transmission.end());
    }
    return bytes;
  }();

  StreamChapterParser parser;
  std::size_t chapter_count = 0;

  parser.registerGsofChapterFoundCallback(
      [k_expected_payload_size, &expected_gsof_message_ids, &chapter_count](const std::vector<std::byte> &chapter) {
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

        ++chapter_count;
      });

  readInChunks(parser, stream, k_chunk_size);

  ASSERT_EQ(chapter_count, k_expected_transmissions);
}

TEST(GsofStreamingTest, singlePageStreamTest) {
  // This test checks if the multipage stream parser works properly with a data stream that doesn't get split over
  // multiple pages
  constexpr std::size_t k_expected_chapters = 10;
  constexpr std::size_t k_chunk_size        = 41;  // Deliberately unaligned with the record size

  const auto stream = [] {
    std::vector<std::byte> bytes;
    for (std::size_t i = 0; i < k_expected_chapters; ++i) {
      const auto record = makeNavAndRmsRecord(static_cast<uint32_t>(1000 * (i + 1)));
      bytes.insert(bytes.end(), record.begin(), record.end());
    }
    return bytes;
  }();

  trmb::gsof::StreamChapterParser parser;
  std::size_t chapter_count = 0;
  uint32_t last_time        = 0;

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

  readInChunks(parser, stream, k_chunk_size);

  ASSERT_EQ(chapter_count, k_expected_chapters);
}
