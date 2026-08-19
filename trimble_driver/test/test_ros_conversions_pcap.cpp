/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

/**
 * ROS conversion regression test driven by a capture taken from a real receiver. It is built but deliberately not
 * registered with ctest, see trmb_cc_test(NO_DISCOVER). Run the trimble_driver_pcap_test binary from test/data.
 */

#include <gtest/gtest.h>

#ifdef TRMB_TF2_HEADER_DEPRECATED
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include "streaming_test.h"
#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver_ros/conversions.h"

using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

static constexpr char k_sps986_ip_addr[] = "238.0.0.1";  // IP Address of SPS986
static constexpr int k_sps986_port       = 5018;         // TCP Port used

class RosConversionsTest : public ::testing::Test {
 public:
  std::optional<PublicPacketParser> openPcap(const std::string &filename) {
    pcap_reader_.emplace(filename, k_sps986_ip_addr, k_sps986_port, network::ProtocolType::TCP);
    if (!(*pcap_reader_)) return std::nullopt;

    pcap_reader_->readSingle(&payload_);
    if (payload_.data == nullptr) return std::nullopt;

    return PacketParser(reinterpret_cast<const std::byte *>(payload_.data), payload_.length);
  }

 protected:
  std::optional<network::PcapReader> pcap_reader_ = std::nullopt;
  network::NonOwningBuffer payload_{nullptr, 0};
};

TEST_F(RosConversionsTest, gsof1toNavSatStatusPCAPTest) {
  auto maybe_gsof_parser = openPcap("SPS986_gsof1.pcap");
  ASSERT_TRUE(maybe_gsof_parser);
  auto gsof_parser = *maybe_gsof_parser;
  ASSERT_TRUE(gsof_parser.isValid());

  auto message_parser = gsof_parser.getMessageParser();
  ASSERT_TRUE(message_parser.isValid());
  auto it = message_parser.begin();
  ASSERT_EQ(it->getHeader().type, 0x01);
  ASSERT_EQ(it->getHeader().length, 10);

  auto position_time = it->as<trmb::gsof::PositionTimeInfo>();

  auto ros_nav_sat_status = trmb_ros::toNavSatStatus(position_time);
  ASSERT_EQ(ros_nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}
