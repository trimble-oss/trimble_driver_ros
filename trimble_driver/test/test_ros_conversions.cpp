/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include "trimble_driver/gsof/packet_parser.h"
#include "trimble_driver/gsof/stream_page_parser.h"
#ifdef TRMB_TF2_HEADER_DEPRECATED
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include "streaming_test.h"
#include "trimble_driver_ros/conversions.h"

using trmb::gsof::Message;
using trmb::gsof::PacketParser;
using trmb::gsof::PublicPacketParser;

static constexpr char k_ip_addr[] = "238.0.0.1";  // IP Address of SPS986
static constexpr int k_port       = 5018;         // TCP Port used

class RosConversionsTest : public ::testing::Test {
 public:
  std::optional<PublicPacketParser> openPcap(const std::string &filename) {
    pcap_reader_.emplace(filename, k_ip_addr, k_port, network::ProtocolType::TCP);
    if (!(*pcap_reader_)) return std::nullopt;

    pcap_reader_->readSingle(&payload_);
    if (payload_.data == nullptr) return std::nullopt;

    return PacketParser(reinterpret_cast<const std::byte *>(payload_.data), payload_.length);
  }

 protected:
  std::optional<network::PcapReader> pcap_reader_ = std::nullopt;
  network::NonOwningBuffer payload_{nullptr, 0};
};

class RosRep103Test : public ::testing::Test {
 public:
  void setEnu(const double east, const double north, const double up) {
    local_cartesian_.Reverse(east, north, up, ins_solution_.lla.latitude, ins_solution_.lla.longitude,
                             ins_solution_.lla.altitude);
  }
  void getRotMat(tf2::Matrix3x3 &m) {
    tf2::Quaternion q(odom_.pose.pose.orientation.x,
                      odom_.pose.pose.orientation.y,
                      odom_.pose.pose.orientation.z,
                      odom_.pose.pose.orientation.w);
    m = tf2::Matrix3x3(q);
  }
  void getRPY(double &roll, double &pitch, double &yaw) {
    tf2::Matrix3x3 m;
    getRotMat(m);
    m.getRPY(roll, pitch, yaw, 1);
  }

 protected:
  void SetUp() override { setEnu(0.0, 0.0, 0.0); }
  nav_msgs::msg::Odometry odom_;
  trmb::gsof::NavigationSolution ins_solution_;
  GeographicLib::LocalCartesian local_cartesian_ =
      GeographicLib::LocalCartesian(k_ref_latitude, k_ref_longitude, k_ref_altitude);

 private:
  static constexpr double k_ref_latitude  = 39.8367;
  static constexpr double k_ref_longitude = -105.0372;
  static constexpr double k_ref_altitude  = 0.0;
};

TEST(RosConversions, hasHeaderTypeTrait) {
  auto has_header = trmb_ros::HasStdMsgsHeader<geometry_msgs::msg::AccelStamped>::value;
  ASSERT_TRUE(has_header);
}

TEST(RosConversions, doesntHaveHeaderTypeTrait) {
  auto doesnt_have_header = trmb_ros::HasStdMsgsHeader<geometry_msgs::msg::Accel>::value;
  ASSERT_FALSE(doesnt_have_header);

  struct MyTestStruct {
    int header;
  };

  doesnt_have_header = trmb_ros::HasStdMsgsHeader<MyTestStruct>::value;
  ASSERT_FALSE(doesnt_have_header);
}

TEST(RosConversions, gsof1No3DPositiontoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test no 3D position calculated
  gsof1.position_flags_1 = 2;  // 0b00000010
  gsof1.position_flags_2 = 0;  // 0b00000000
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}
TEST(RosConversions, gsof1FixedRTKtoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test Fixed RTK
  gsof1.position_flags_1 = 15;  // 0b00001111
  gsof1.position_flags_2 = 7;   // 0b00000111
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}
TEST(RosConversions, gsof1FloatRTKtoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test Float RTK
  gsof1.position_flags_1 = 15;  // 0b00001111
  gsof1.position_flags_2 = 3;   // 0b00000011
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}
TEST(RosConversions, gsof1DGPStoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test Code (DGPS)
  gsof1.position_flags_1 = 47;  // 0b00101111
  gsof1.position_flags_2 = 5;   // 0b00000101
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}
TEST(RosConversions, gsof1WAAStoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test WAAS
  gsof1.position_flags_1 = 79;  // 0b01001111
  gsof1.position_flags_2 = 6;   // 0b00000110
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX);
}
TEST(RosConversions, gsof1AutonomoustoNavSatStatus) {
  trmb::gsof::PositionTimeInfo gsof1;
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // Test Autonomous Solution
  gsof1.position_flags_1 = 143;  // 0b10001111
  gsof1.position_flags_2 = 2;    // 0b00000010
  nav_sat_status         = trmb_ros::toNavSatStatus(gsof1);
  ASSERT_EQ(nav_sat_status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}

// Test Quaternion Conversion of GSOF49's INS Attitude
TEST(RosConversions, gsof49INSAttitudeToQuaternion) {
  trmb::gsof::NavigationSolution ins_solution;
  GeographicLib::LocalCartesian local_cartesian;
  float tolerance = 0.000001;

  // All in degrees
  ins_solution.attitude.roll    = -3.4;
  ins_solution.attitude.pitch   = -2.1;
  ins_solution.attitude.heading = 277;

  nav_msgs::msg::Odometry odom = trmb_ros::toOdometry(ins_solution, local_cartesian);

  ASSERT_NEAR(odom.pose.pose.orientation.x, 0.0343521, tolerance);
  ASSERT_NEAR(odom.pose.pose.orientation.y, -0.0059356, tolerance);
  ASSERT_NEAR(odom.pose.pose.orientation.z, 0.6626243, tolerance);
  ASSERT_NEAR(odom.pose.pose.orientation.w, -0.7481401, tolerance);
}

// PCAP Test
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

TEST_F(RosRep103Test, transformYaw0Deg) {
  ins_solution_.velocity.north = 1.0f;
  ins_solution_.velocity.east  = 2.0f;
  ins_solution_.velocity.down  = -3.0f;

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  // NED --> FRD to ENU --> FLU
  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);

  EXPECT_NEAR(trmb::rad2deg(roll), 0.0, 1e-10);
  EXPECT_NEAR(trmb::rad2deg(pitch), 0.0, 1e-10);
  EXPECT_FLOAT_EQ(trmb::rad2deg(yaw), 90.0);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.x, ins_solution_.velocity.north);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.y, -ins_solution_.velocity.east);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.z, -ins_solution_.velocity.down);
}

TEST_F(RosRep103Test, transformYaw90Deg) {
  ins_solution_.velocity.north   = 1.0f;
  ins_solution_.velocity.east    = 2.0f;
  ins_solution_.velocity.down    = -3.0f;
  ins_solution_.attitude.roll    = 0.0;
  ins_solution_.attitude.pitch   = 0.0;
  ins_solution_.attitude.heading = 90.0;

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);

  EXPECT_NEAR(trmb::rad2deg(roll), 0.0, 1e-10);
  EXPECT_NEAR(trmb::rad2deg(pitch), 0.0, 1e-10);
  EXPECT_NEAR(trmb::rad2deg(yaw), 0.0, 1e-10);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.x, ins_solution_.velocity.east);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.y, ins_solution_.velocity.north);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.z, -ins_solution_.velocity.down);
}

TEST_F(RosRep103Test, linearVelocityPositive) {
  ins_solution_.velocity.north   = 1.0f;
  ins_solution_.velocity.east    = 1.0f;
  ins_solution_.velocity.down    = 0.0f;
  ins_solution_.attitude.roll    = 0.0;
  ins_solution_.attitude.pitch   = 0.0;
  ins_solution_.attitude.heading = 45.0;

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);

  EXPECT_NEAR(trmb::rad2deg(roll), 0.0, 1e-10);
  EXPECT_NEAR(trmb::rad2deg(pitch), 0.0, 1e-10);
  EXPECT_FLOAT_EQ(trmb::rad2deg(yaw), 45.0);
  EXPECT_NEAR(odom_.twist.twist.linear.x,
              std::sqrt(std::pow(ins_solution_.velocity.east, 2.0) + std::pow(ins_solution_.velocity.north, 2.0)),
              1e-7);
  EXPECT_NEAR(odom_.twist.twist.linear.y, 0.0, 1e-15);
  EXPECT_NEAR(odom_.twist.twist.linear.z, 0.0, 1e-15);
}

TEST_F(RosRep103Test, linearVelocityTangent) {
  ins_solution_.velocity.north   = 1.0f;
  ins_solution_.velocity.east    = 1.0f;
  ins_solution_.velocity.down    = 0.0f;
  ins_solution_.attitude.roll    = 0.0;
  ins_solution_.attitude.pitch   = 0.0;
  ins_solution_.attitude.heading = -45.0;

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);

  EXPECT_NEAR(trmb::rad2deg(roll), 0.0, 1e-10);
  EXPECT_NEAR(trmb::rad2deg(pitch), 0.0, 1e-10);
  EXPECT_FLOAT_EQ(trmb::rad2deg(yaw), 135.0);
  EXPECT_NEAR(odom_.twist.twist.linear.x, 0.0, 1e-15);
  EXPECT_NEAR(odom_.twist.twist.linear.y,
              -std::sqrt(std::pow(ins_solution_.velocity.east, 2.0) + std::pow(ins_solution_.velocity.north, 2.0)),
              1e-7);
  EXPECT_NEAR(odom_.twist.twist.linear.z, 0.0, 1e-15);
}

TEST_F(RosRep103Test, multipleRotations) {
  ins_solution_.velocity.north   = 0.0f;
  ins_solution_.velocity.east    = 1.0f;
  ins_solution_.velocity.down    = 0.0f;
  ins_solution_.attitude.roll    = 45.0;
  ins_solution_.attitude.pitch   = 0.0;
  ins_solution_.attitude.heading = -45.0;
  // Rotation matrix
  // [[ 0.70710678 -0.70710678  0.        ]
  // [-0.5        -0.5        -0.70710678]
  // [ 0.5         0.5        -0.70710678]]

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);

  EXPECT_FLOAT_EQ(trmb::rad2deg(roll), 45.0);
  EXPECT_NEAR(trmb::rad2deg(pitch), 0.0, 1e-10);
  EXPECT_FLOAT_EQ(trmb::rad2deg(yaw), 135.0);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.x, -M_SQRT2 / 2.0);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.y, -0.5);
  EXPECT_FLOAT_EQ(odom_.twist.twist.linear.z, 0.5);
}

TEST_F(RosRep103Test, angularVelocity) {
  // Rates in FRD
  ins_solution_.angular_rate.roll    = 10.0f;
  ins_solution_.angular_rate.pitch   = -20.0f;
  ins_solution_.angular_rate.heading = 30.0f;

  odom_ = trmb_ros::toRep103(ins_solution_, local_cartesian_);

  // Rates in FLU
  EXPECT_FLOAT_EQ(trmb::rad2deg(odom_.twist.twist.angular.x), ins_solution_.angular_rate.roll);
  EXPECT_FLOAT_EQ(trmb::rad2deg(odom_.twist.twist.angular.y), -ins_solution_.angular_rate.pitch);
  EXPECT_FLOAT_EQ(trmb::rad2deg(odom_.twist.twist.angular.z), -ins_solution_.angular_rate.heading);
}