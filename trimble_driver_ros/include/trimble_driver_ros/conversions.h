/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <gsof_msgs/msg/all_sv_brief33.hpp>
#include <gsof_msgs/msg/all_sv_detailed34.hpp>
#include <gsof_msgs/msg/attitude_info27.hpp>
#include <gsof_msgs/msg/base_position_and_quality_indicator41.hpp>
#include <gsof_msgs/msg/battery_memory_info37.hpp>
#include <gsof_msgs/msg/clock_info10.hpp>
#include <gsof_msgs/msg/current_time16.hpp>
#include <gsof_msgs/msg/ecef_delta6.hpp>
#include <gsof_msgs/msg/ecef_position3.hpp>
#include <gsof_msgs/msg/ins_vnav_full_nav_info63.hpp>
#include <gsof_msgs/msg/ins_vnav_rms_info64.hpp>
#include <gsof_msgs/msg/lat_long_height2.hpp>
#include <gsof_msgs/msg/lband_status_info40.hpp>
#include <gsof_msgs/msg/navigation_performance50.hpp>
#include <gsof_msgs/msg/navigation_solution49.hpp>
#include <gsof_msgs/msg/pdop_info9.hpp>
#include <gsof_msgs/msg/position_sigma_info12.hpp>
#include <gsof_msgs/msg/position_time_info1.hpp>
#include <gsof_msgs/msg/position_type_information38.hpp>
#include <gsof_msgs/msg/position_vcv_info11.hpp>
#include <gsof_msgs/msg/received_base_info35.hpp>
#include <gsof_msgs/msg/receiver_serial_number15.hpp>
#include <gsof_msgs/msg/tangent_plane_delta7.hpp>
#include <gsof_msgs/msg/velocity8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <type_traits>

#include "trimble_driver/gsof/message.h"

namespace trmb_ros {

template <typename T, typename = std_msgs::msg::Header>
struct HasStdMsgsHeader : std::false_type {};

/**
 * Type trait that checks if T has a public member of type std_msgs::msg::Header named header.
 * @tparam T
 */
template <typename T>
struct HasStdMsgsHeader<T, decltype(T::header)> : std::true_type {};

template <typename T>
inline constexpr bool HasStdMsgsHeaderV = HasStdMsgsHeader<T>::value;

template <typename T, typename = trmb::gsof::GpsTime>
struct HasGsofGpsTime : std::false_type {};

template <typename T>
struct HasGsofGpsTime<T, decltype(T::gps_time)> : std::true_type {};

geometry_msgs::msg::TransformStamped toTransformStamped(
    const std_msgs::msg::Header &header,
    const decltype(geometry_msgs::msg::TransformStamped::child_frame_id) &child_frame_id,
    const geometry_msgs::msg::Pose &pose);

nav_msgs::msg::Odometry toOdometry(const trmb::gsof::NavigationSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian);
nav_msgs::msg::Odometry toOdometry(const trmb::gsof::NavigationSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian,
                                   const trmb::gsof::NavigationPerformance &ins_solution_rms);

nav_msgs::msg::Odometry toRep103(const trmb::gsof::NavigationSolution &ins_solution,
                                 const GeographicLib::LocalCartesian &local_cartesian);
nav_msgs::msg::Odometry toRep103(const trmb::gsof::NavigationSolution &ins_solution,
                                 const GeographicLib::LocalCartesian &local_cartesian,
                                 const trmb::gsof::NavigationPerformance &ins_solution_rms);

sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::PositionTimeInfo &position_time_info,
                                        const trmb::gsof::LatLongHeight &lat_long_height,
                                        const trmb::gsof::PositionSigmaInfo &position_sigma_info);
sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::NavigationSolution &ins_solution);
sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::NavigationSolution &ins_solution,
                                        const trmb::gsof::NavigationPerformance &covariance);

sensor_msgs::msg::NavSatStatus toNavSatStatus(const trmb::gsof::PositionTimeInfo &position_time_info);

template <typename T>
geometry_msgs::msg::Point toPoint(const trmb::Xyz<T> &xyz) {
  geometry_msgs::msg::Point p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  return p;
}

template <typename T>
geometry_msgs::msg::Point toPoint(const trmb::Ned<T> &ned) {
  geometry_msgs::msg::Point p;
  p.x = ned.north;
  p.y = ned.east;
  p.z = ned.down;
  return p;
}

template <typename T>
geometry_msgs::msg::Point toPoint(const trmb::Enu<T> &enu) {
  geometry_msgs::msg::Point p;
  p.x = enu.east;
  p.y = enu.north;
  p.z = enu.up;
  return p;
}

template <typename T>
geometry_msgs::msg::Vector3 toVector(const trmb::Xyz<T> &xyz) {
  geometry_msgs::msg::Vector3 v;
  v.x = static_cast<double>(xyz.x);
  v.y = static_cast<double>(xyz.y);
  v.z = static_cast<double>(xyz.z);
  return v;
}

template <typename T>
geometry_msgs::msg::Vector3 toVector(const trmb::Ned<T> &ned) {
  geometry_msgs::msg::Vector3 v;
  v.x = static_cast<double>(ned.north);
  v.y = static_cast<double>(ned.east);
  v.z = static_cast<double>(ned.down);
  return v;
}

template <typename T>
geometry_msgs::msg::Vector3 toVector(const trmb::Enu<T> &enu) {
  geometry_msgs::msg::Vector3 v;
  v.x = static_cast<double>(enu.east);
  v.y = static_cast<double>(enu.north);
  v.z = static_cast<double>(enu.up);
  return v;
}

template <typename Scalar>
geometry_msgs::msg::Vector3 toVector(const trmb::Rph<Scalar> &rph) {
  geometry_msgs::msg::Vector3 v;
  v.x = static_cast<double>(rph.roll);
  v.y = static_cast<double>(rph.pitch);
  v.z = static_cast<double>(rph.heading);
  return v;
}

namespace gsof {
template <typename T>
gsof_msgs::msg::LatLongAltitude toRosMessage(const trmb::Lla<T> &lla) {
  gsof_msgs::msg::LatLongAltitude ros_lla;
  ros_lla.latitude  = lla.latitude;
  ros_lla.longitude = lla.longitude;
  ros_lla.altitude  = lla.altitude;
  return ros_lla;
}

template <typename T>
gsof_msgs::msg::NorthEastDownf toRosMessage(const trmb::Ned<T> &ned) {
  gsof_msgs::msg::NorthEastDownf ros_ned;
  ros_ned.north = ned.north;
  ros_ned.east  = ned.east;
  ros_ned.down  = ned.down;
  return ros_ned;
}

gsof_msgs::msg::Vector3f toVector3f(const trmb::Xyzf &);

gsof_msgs::msg::EastNorthUpd toRosMessage(const trmb::Enud &enu);
gsof_msgs::msg::EulerAngle toRosMessage(const trmb::Pyrd &pyr);
gsof_msgs::msg::LatLongAltitude toRosLla(const trmb::Llad &lla);
gsof_msgs::msg::LatLongHeight toRosLlh(const trmb::Llad &lla);
}  // namespace gsof

rclcpp::Time toRosTimeOfTheWeek(const trmb::gsof::GpsTime &gps_time);
rclcpp::Time toRosTimeGpsEpoch(const trmb::gsof::GpsTime &gps_time);

gsof_msgs::msg::GpsTime toRosMessage(const trmb::gsof::GpsTime &gps_time);
gsof_msgs::msg::Status toRosMessage(const trmb::gsof::Status &status);
gsof_msgs::msg::Status toRosMessage(const trmb::gsof::VnavStatus &status);

gsof_msgs::msg::PositionTimeInfo1 toRosMessage(const trmb::gsof::PositionTimeInfo &);
gsof_msgs::msg::LatLongHeight2 toRosMessage(const trmb::gsof::LatLongHeight &);
gsof_msgs::msg::EcefPosition3 toRosMessage(const trmb::gsof::EcefPosition &);
gsof_msgs::msg::EcefDelta6 toRosMessage(const trmb::gsof::EcefDelta &);
gsof_msgs::msg::TangentPlaneDelta7 toRosMessage(const trmb::gsof::TangentPlaneDelta &);
gsof_msgs::msg::Velocity8 toRosMessage(const trmb::gsof::Velocity &);
gsof_msgs::msg::PdopInfo9 toRosMessage(const trmb::gsof::PdopInfo &);
gsof_msgs::msg::ClockInfo10 toRosMessage(const trmb::gsof::ClockInfo &);
gsof_msgs::msg::PositionVcvInfo11 toRosMessage(const trmb::gsof::PositionVcvInfo &);
gsof_msgs::msg::PositionSigmaInfo12 toRosMessage(const trmb::gsof::PositionSigmaInfo &);
gsof_msgs::msg::ReceiverSerialNumber15 toRosMessage(const trmb::gsof::ReceiverSerialNumber &);
gsof_msgs::msg::CurrentTime16 toRosMessage(const trmb::gsof::CurrentTime &);
gsof_msgs::msg::AttitudeVariance toRosMessage(const trmb::gsof::AttitudeInfo::Variance &);
gsof_msgs::msg::AttitudeInfo27 toRosMessage(const trmb::gsof::AttitudeInfo &);
gsof_msgs::msg::SpaceVehicleBriefInfo toRosMessage(const trmb::gsof::SVBriefInfo &);
gsof_msgs::msg::AllSvBrief33 toRosMessage(const trmb::gsof::AllSvBrief &);
gsof_msgs::msg::SpaceVehicleDetailedInfo toRosMessage(const trmb::gsof::SVDetailedInfo &);
gsof_msgs::msg::AllSvDetailed34 toRosMessage(const trmb::gsof::AllSvDetailed &);
gsof_msgs::msg::ReceivedBaseInfo35 toRosMessage(const trmb::gsof::ReceivedBaseInfo &);
gsof_msgs::msg::BatteryMemoryInfo37 toRosMessage(const trmb::gsof::BatteryMemoryInfo &);
gsof_msgs::msg::PositionTypeInformation38 toRosMessage(const trmb::gsof::PositionTypeInformation &);
gsof_msgs::msg::LbandStatusInfo40 toRosMessage(const trmb::gsof::LbandStatusInfo &);
gsof_msgs::msg::BasePositionAndQualityIndicator41 toRosMessage(const trmb::gsof::BasePositionAndQualityIndicator &);
gsof_msgs::msg::NavigationSolution49 toRosMessage(const trmb::gsof::NavigationSolution &);
gsof_msgs::msg::NavigationPerformance50 toRosMessage(const trmb::gsof::NavigationPerformance &);
gsof_msgs::msg::InsVnavFullNavInfo63 toRosMessage(const trmb::gsof::InsVnavFullNavInfo &);
gsof_msgs::msg::InsVnavRmsInfo64 toRosMessage(const trmb::gsof::InsVnavRmsInfo &);

}  // namespace trmb_ros
