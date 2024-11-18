/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver_ros/conversions.h"

#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/logging.hpp>
#ifdef TRMB_TF2_HEADER_DEPRECATED
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include "trimble_driver/math.h"

namespace trmb_ros {

using trmb::deg2rad;

geometry_msgs::msg::TransformStamped toTransformStamped(
    const std_msgs::msg::Header &header,
    const decltype(geometry_msgs::msg::TransformStamped::child_frame_id) &child_frame_id,
    const geometry_msgs::msg::Pose &pose) {
  geometry_msgs::msg::TransformStamped t;
  t.header                  = header;
  t.child_frame_id          = child_frame_id;
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation      = pose.orientation;
  return t;
}

constexpr std::int64_t k_sec_to_nano         = 1e9;
constexpr std::int64_t k_milli_to_nano       = 1e6;
constexpr std::int64_t k_gps_seconds_in_week = 60 * 60 * 24 * 7;

rclcpp::Time toRosTimeOfTheWeek(const trmb::gsof::GpsTime &gps_time) {
  return rclcpp::Time(gps_time.time_msec * k_milli_to_nano, RCL_STEADY_TIME);
}

rclcpp::Time toRosTimeGpsEpoch(const trmb::gsof::GpsTime &gps_time) {
  // GpsTime has week as 16 bit so we don't need to take week rollover into account
  const std::int64_t gps_nanoseconds_since_epoch =
      (k_gps_seconds_in_week * static_cast<std::int64_t>(gps_time.week) * k_sec_to_nano) +
      (k_milli_to_nano * gps_time.time_msec);
  return rclcpp::Time(gps_nanoseconds_since_epoch, RCL_STEADY_TIME);
}

nav_msgs::msg::Odometry toOdometry(const trmb::gsof::NavigationSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = toRosTimeOfTheWeek(ins_solution.gps_time);

  trmb::Enud enu;
  const auto &lla = ins_solution.lla;
  local_cartesian.Forward(lla.latitude, lla.longitude, lla.altitude, enu.east, enu.north, enu.up);

  // Convert to NED while we're at it
  trmb::Nedd ned(enu);
  odom.pose.pose.position = toPoint(ned);

  tf2::Quaternion q;
  q.setRPY(deg2rad(ins_solution.attitude.roll),
           deg2rad(ins_solution.attitude.pitch),
           deg2rad(ins_solution.attitude.heading));
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear  = toVector(ins_solution.velocity);
  odom.twist.twist.angular = toVector(ins_solution.angular_rate);

  odom.twist.twist.angular.x = deg2rad(odom.twist.twist.angular.x);
  odom.twist.twist.angular.y = deg2rad(odom.twist.twist.angular.y);
  odom.twist.twist.angular.z = deg2rad(odom.twist.twist.angular.z);

  odom.pose.covariance[0]  = -1;
  odom.twist.covariance[0] = -1;

  return odom;
}

nav_msgs::msg::Odometry toOdometry(const trmb::gsof::NavigationSolution &ins_solution,
                                   const GeographicLib::LocalCartesian &local_cartesian,
                                   const trmb::gsof::NavigationPerformance &ins_solution_rms) {
  nav_msgs::msg::Odometry odom = toOdometry(ins_solution, local_cartesian);

  odom.pose.covariance[0]  = std::pow(ins_solution_rms.position_rms.north, 2);
  odom.pose.covariance[7]  = std::pow(ins_solution_rms.position_rms.east, 2);
  odom.pose.covariance[14] = std::pow(ins_solution_rms.position_rms.down, 2);
  odom.pose.covariance[21] = std::pow(deg2rad(ins_solution_rms.attitude_rms.roll), 2);
  odom.pose.covariance[28] = std::pow(deg2rad(ins_solution_rms.attitude_rms.pitch), 2);
  odom.pose.covariance[35] = std::pow(deg2rad(ins_solution_rms.attitude_rms.heading), 2);

  odom.twist.covariance[0]  = std::pow(ins_solution_rms.velocity_rms.north, 2);
  odom.twist.covariance[7]  = std::pow(ins_solution_rms.velocity_rms.east, 2);
  odom.twist.covariance[14] = std::pow(ins_solution_rms.velocity_rms.down, 2);

  return odom;
}

nav_msgs::msg::Odometry toRep103(const trmb::gsof::NavigationSolution &ins_solution,
                                 const GeographicLib::LocalCartesian &local_cartesian) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = toRosTimeOfTheWeek(ins_solution.gps_time);

  // Position in ENU
  trmb::Enud enu;
  const auto &lla = ins_solution.lla;
  local_cartesian.Forward(lla.latitude, lla.longitude, lla.altitude, enu.east, enu.north, enu.up);
  odom.pose.pose.position = toPoint(enu);

  // Define rotation frames
  tf2::Quaternion q_frd_ned;  // NED to FRD
  q_frd_ned.setRPY(deg2rad(ins_solution.attitude.roll),
                   deg2rad(ins_solution.attitude.pitch),
                   deg2rad(ins_solution.attitude.heading));
  q_frd_ned = q_frd_ned.inverse();  // Change from active rotation to passive
  tf2::Quaternion q_ned_enu;        // ENU to NED
  q_ned_enu.setRPY(M_PI, 0.0, M_PI_2);
  tf2::Quaternion q_flu_frd;  // FRD to FLU
  q_flu_frd.setRPY(M_PI, 0.0, 0.0);

  // Convert orientation from NED --> FRD to ENU --> FLU
  tf2::Quaternion q_flu_enu  = (q_flu_frd * q_frd_ned * q_ned_enu).normalize();
  odom.pose.pose.orientation = tf2::toMsg(q_flu_enu.inverse());

  // Convert velocity from NED to REP 103 compliant FLU body frame
  tf2::Vector3 v_ned(ins_solution.velocity.north, ins_solution.velocity.east, ins_solution.velocity.down);
  v_ned                     = tf2::quatRotate((q_flu_frd * q_frd_ned).normalize(), v_ned);
  odom.twist.twist.linear.x = v_ned.getX();
  odom.twist.twist.linear.y = v_ned.getY();
  odom.twist.twist.linear.z = v_ned.getZ();

  // Convert rates from FRD to REP 103 compliant FLU body frame
  tf2::Vector3 angular_velocity(deg2rad(ins_solution.angular_rate.roll),
                                deg2rad(ins_solution.angular_rate.pitch),
                                deg2rad(ins_solution.angular_rate.heading));
  angular_velocity           = tf2::quatRotate((q_flu_frd).normalize(), angular_velocity);
  odom.twist.twist.angular.x = angular_velocity.getX();
  odom.twist.twist.angular.y = angular_velocity.getY();
  odom.twist.twist.angular.z = angular_velocity.getZ();

  odom.pose.covariance[0]  = -1;
  odom.twist.covariance[0] = -1;

  return odom;
}

nav_msgs::msg::Odometry toRep103(const trmb::gsof::NavigationSolution &ins_solution,
                                 const GeographicLib::LocalCartesian &local_cartesian,
                                 const trmb::gsof::NavigationPerformance &ins_solution_rms) {
  nav_msgs::msg::Odometry odom = toRep103(ins_solution, local_cartesian);

  odom.pose.covariance[0]  = std::pow(ins_solution_rms.position_rms.east, 2);  // swapped east & north
  odom.pose.covariance[7]  = std::pow(ins_solution_rms.position_rms.north, 2);
  odom.pose.covariance[14] = std::pow(ins_solution_rms.position_rms.down, 2);

  // When the transform is applied to the RMS values for RPY, the result is the original covariance
  // See [doc/covariance_rotation.md]
  odom.pose.covariance[21] = std::pow(deg2rad(ins_solution_rms.attitude_rms.roll), 2);
  odom.pose.covariance[28] = std::pow(deg2rad(ins_solution_rms.attitude_rms.pitch), 2);
  odom.pose.covariance[35] = std::pow(deg2rad(ins_solution_rms.attitude_rms.heading), 2);

  odom.twist.covariance[0]  = std::pow(ins_solution_rms.velocity_rms.east, 2);
  odom.twist.covariance[7]  = std::pow(ins_solution_rms.velocity_rms.north, 2);
  odom.twist.covariance[14] = std::pow(ins_solution_rms.velocity_rms.down, 2);

  return odom;
}

sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::PositionTimeInfo &position_time_info,
                                        const trmb::gsof::LatLongHeight &lat_long_height,
                                        const trmb::gsof::PositionSigmaInfo &position_sigma_info) {
  sensor_msgs::msg::NavSatFix nav_sat_fix;

  nav_sat_fix.latitude  = trmb::rad2deg(lat_long_height.lla.latitude);
  nav_sat_fix.longitude = trmb::rad2deg(lat_long_height.lla.longitude);
  nav_sat_fix.altitude  = lat_long_height.lla.altitude;
  // NavSatFix Covariance Matrix's diagonal is defined as East -> North -> Up
  nav_sat_fix.position_covariance[0] = std::pow(position_sigma_info.sigma_east, 2);
  nav_sat_fix.position_covariance[4] = std::pow(position_sigma_info.sigma_north, 2);
  nav_sat_fix.position_covariance[8] = std::pow(position_sigma_info.sigma_up, 2);
  // GSOF #12 also includes the covariance of east-north
  nav_sat_fix.position_covariance[1] = static_cast<double>(position_sigma_info.covariance_east_north);
  // We only know the entirety of the diagonal
  nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  nav_sat_fix.status = toNavSatStatus(position_time_info);

  return nav_sat_fix;
}

sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::NavigationSolution &ins_solution) {
  sensor_msgs::msg::NavSatFix nav_sat_fix;
  nav_sat_fix.latitude  = ins_solution.lla.latitude;
  nav_sat_fix.longitude = ins_solution.lla.longitude;
  nav_sat_fix.altitude  = ins_solution.lla.altitude;

  nav_sat_fix.position_covariance[0] = -1;  // Means unknown

  using GnssStatus = trmb::gsof::Status::GnssStatus;

  switch (ins_solution.status.getGnssStatus()) {
    case GnssStatus::GNSS_SPS_MODE:
    case GnssStatus::GPS_PPS_MODE:
    case GnssStatus::DIRECT_GEOREFERENCING_MODE:
    case GnssStatus::FLOAT_RTK:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case GnssStatus::DIFFERENTIAL_GPS_SPS:
    case GnssStatus::FIXED_RTK_MODE:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    case GnssStatus::FIX_NOT_AVAILABLE:
    case GnssStatus::UNKNOWN:
      nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
  }

  // Note(Andre) There are 3 GSOF messages giving more details about which space vehicles are being used, we can use
  //  those to populate this field if we really need it. However, the enumerations in the sensor_msgs package don't
  //  cover nearly as many constellations Trimble products can receive from.
  nav_sat_fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  return nav_sat_fix;
}

sensor_msgs::msg::NavSatFix toNavSatFix(const trmb::gsof::NavigationSolution &ins_solution,
                                        const trmb::gsof::NavigationPerformance &covariance) {
  sensor_msgs::msg::NavSatFix nav_sat_fix = toNavSatFix(ins_solution);

  // XXX ROS NavSatFix says position covariance is in ENU
  nav_sat_fix.position_covariance[0] = std::pow(covariance.position_rms.east, 2);
  nav_sat_fix.position_covariance[4] = std::pow(covariance.position_rms.north, 2);
  nav_sat_fix.position_covariance[8] = std::pow(covariance.position_rms.down, 2);

  nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix;
}

sensor_msgs::msg::NavSatStatus toNavSatStatus(const trmb::gsof::PositionTimeInfo &position_time_info) {
  sensor_msgs::msg::NavSatStatus nav_sat_status;
  // It's not possible to determine the service type from GSOF #1. If we really need it, we possibly can add in another
  // message to get this info
  nav_sat_status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  // Determine NavSatStatus status field from GSOF #1 Position Flags
  if (position_time_info.isNewPos() &&  // Flag1 Bits 0 && 1 && 2 && 3: Do we have a calculated 3D position?
      position_time_info.isClockFix() && position_time_info.isHCoordinatesComputedHere() &&
      position_time_info.isHeightComputedHere()) {
    if (position_time_info.isDiffSoln()) {             // Flag2 Bit 0: Differential Position
      if (position_time_info.isDiffPosInPhase()) {     // Flag2 Bit 1: Differential Position Method
        if (position_time_info.isDiffPosFixedInt()) {  // Flag2 Bit 2: Differential Position Method
          // Position is fixed integer phase position (RTK)
          nav_sat_status.status =
              sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;  // Fixed RTK uses Ground Based Augmentation
        } else {
          // Float position (RTK)
          nav_sat_status.status =
              sensor_msgs::msg::NavSatStatus::STATUS_FIX;  // Float RTK is defined as less accurate than Fixed RTK
        }
      } else {
        // Code (DGPS)
        nav_sat_status.status =
            sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;  // Differential GPS uses ground based augmentation
      }
    } else {
      // Differential Position is an autonomous or WAAS solution
      if (position_time_info.isDiffPosFixedInt()) {  // Flag2 Bit 2: Differential Position Method
        // Position is fixed integer phase position (RTK). Uncorrected position is WAAS.
        nav_sat_status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      } else {
        // Code (DGPS) or a float position (RTK). Uncorrected position is autonomous.
        nav_sat_status.status =
            sensor_msgs::msg::NavSatStatus::STATUS_FIX;  // Autonomous solution is designated as an unaugmented position
      }
    }
  } else {
    // If a 3D position isn't calculated (which is what we want) we designate it as "no fix"
    // which means no position is available at all
    nav_sat_status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  return nav_sat_status;
}

namespace gsof {
gsof_msgs::msg::Vector3f toVector3f(const trmb::Xyzf &xyz) {
  gsof_msgs::msg::Vector3f v;
  v.x = xyz.x;
  v.y = xyz.y;
  v.z = xyz.z;
  return v;
}

gsof_msgs::msg::EastNorthUpd toRosMessage(const trmb::Enud &enu) {
  gsof_msgs::msg::EastNorthUpd enu_ros;
  enu_ros.east  = enu.east;
  enu_ros.north = enu.north;
  enu_ros.up    = enu.up;
  return enu_ros;
}

gsof_msgs::msg::EulerAngle toRosMessage(const trmb::Pyrd &pyr) {
  gsof_msgs::msg::EulerAngle e;
  e.pitch = pyr.pitch;
  e.roll  = pyr.roll;
  e.yaw   = pyr.yaw;
  return e;
}

gsof_msgs::msg::LatLongAltitude toRosLla(const trmb::Llad &lla) {
  gsof_msgs::msg::LatLongAltitude l_ros;
  l_ros.latitude  = lla.latitude;
  l_ros.longitude = lla.longitude;
  l_ros.altitude  = lla.altitude;
  return l_ros;
}

gsof_msgs::msg::LatLongHeight toRosLlh(const trmb::Llad &lla) {
  gsof_msgs::msg::LatLongHeight l_ros;
  l_ros.latitude  = lla.latitude;
  l_ros.longitude = lla.longitude;
  l_ros.height    = lla.altitude;
  return l_ros;
}
}  // namespace gsof

gsof_msgs::msg::GpsTime toRosMessage(const trmb::gsof::GpsTime &gps_time) {
  gsof_msgs::msg::GpsTime gps_time_gsof;
  gps_time_gsof.time = gps_time.time_msec;
  gps_time_gsof.week = gps_time.week;
  return gps_time_gsof;
}

gsof_msgs::msg::Status toRosMessage(const trmb::gsof::Status &status) {
  gsof_msgs::msg::Status status_gsof;
  status_gsof.gnss          = status.gnss;
  status_gsof.imu_alignment = status.imu_alignment;
  return status_gsof;
}

gsof_msgs::msg::Status toRosMessage(const trmb::gsof::VnavStatus &status) {
  gsof_msgs::msg::Status status_gsof;
  status_gsof.gnss          = status.gps_quality;
  status_gsof.imu_alignment = status.imu_alignment;
  return status_gsof;
}

gsof_msgs::msg::PositionTimeInfo1 toRosMessage(const trmb::gsof::PositionTimeInfo &p) {
  gsof_msgs::msg::PositionTimeInfo1 p_ros;
  p_ros.header.stamp               = toRosTimeOfTheWeek(trmb::gsof::GpsTime{p.gps_week, p.gps_time_ms});
  p_ros.number_space_vehicles_used = p.number_space_vehicles_used;
  p_ros.position_flags_1           = p.position_flags_1;
  p_ros.position_flags_2           = p.position_flags_2;
  p_ros.gps_time.week              = p.gps_week;
  p_ros.gps_time.time              = p.gps_time_ms;
  p_ros.init_num                   = p.init_num;
  return p_ros;
}

gsof_msgs::msg::LatLongHeight2 toRosMessage(const trmb::gsof::LatLongHeight &l) {
  gsof_msgs::msg::LatLongHeight2 l_ros;
  l_ros.llh = gsof::toRosLlh(l.lla);
  return l_ros;
}

gsof_msgs::msg::EcefPosition3 toRosMessage(const trmb::gsof::EcefPosition &p) {
  gsof_msgs::msg::EcefPosition3 p_ros;
  p_ros.position = toVector(p.pos);
  return p_ros;
}

gsof_msgs::msg::EcefDelta6 toRosMessage(const trmb::gsof::EcefDelta &p) {
  gsof_msgs::msg::EcefDelta6 p_ros;
  p_ros.delta = toVector(p.delta);
  return p_ros;
}

gsof_msgs::msg::TangentPlaneDelta7 toRosMessage(const trmb::gsof::TangentPlaneDelta &t) {
  gsof_msgs::msg::TangentPlaneDelta7 t_ros;
  t_ros.delta = gsof::toRosMessage(t.enu);
  return t_ros;
}

gsof_msgs::msg::Velocity8 toRosMessage(const trmb::gsof::Velocity &v) {
  gsof_msgs::msg::Velocity8 v_ros;
  v_ros.velocity       = v.velocity;
  v_ros.velocity_flags = v.velocity_flags;
  v_ros.heading        = v.heading;
  v_ros.local_heading  = (v.local_heading) ? *v.local_heading : -1;
  return v_ros;
}

gsof_msgs::msg::PdopInfo9 toRosMessage(const trmb::gsof::PdopInfo &p) {
  gsof_msgs::msg::PdopInfo9 p_ros;
  p_ros.position_dop    = p.position_dop;
  p_ros.horiziontal_dop = p.horiziontal_dop;
  p_ros.time_dop        = p.time_dop;
  p_ros.vertical_dop    = p.vertical_dop;
  return p_ros;
}

gsof_msgs::msg::ClockInfo10 toRosMessage(const trmb::gsof::ClockInfo &c) {
  gsof_msgs::msg::ClockInfo10 c_ros;
  c_ros.clock_offset = c.clock_offset;
  c_ros.clock_flags  = c.clock_flags;
  c_ros.freq_offset  = c.freq_offset;
  return c_ros;
}

gsof_msgs::msg::PositionVcvInfo11 toRosMessage(const trmb::gsof::PositionVcvInfo &p) {
  gsof_msgs::msg::PositionVcvInfo11 p_ros;
  p_ros.position_rms = p.position_rms;
  p_ros.xx           = p.xx;
  p_ros.xy           = p.xy;
  p_ros.xz           = p.xz;
  p_ros.yy           = p.yy;
  p_ros.yz           = p.yz;
  p_ros.zz           = p.zz;
  p_ros.unit_var     = p.unit_var;
  p_ros.num_epochs   = p.num_epochs;
  return p_ros;
}

gsof_msgs::msg::PositionSigmaInfo12 toRosMessage(const trmb::gsof::PositionSigmaInfo &p) {
  gsof_msgs::msg::PositionSigmaInfo12 p_ros;
  p_ros.position_rms          = p.position_rms;
  p_ros.sigma_east            = p.sigma_east;
  p_ros.sigma_north           = p.sigma_north;
  p_ros.covariance_east_north = p.covariance_east_north;
  p_ros.sigma_up              = p.sigma_up;
  p_ros.semi_major_axis       = p.semi_major_axis;
  p_ros.semi_minor_axis       = p.semi_minor_axis;
  p_ros.orientation           = p.orientation;
  p_ros.unit_variance         = p.unit_variance;
  p_ros.number_epochs         = p.number_epochs;
  return p_ros;
}

gsof_msgs::msg::ReceiverSerialNumber15 toRosMessage(const trmb::gsof::ReceiverSerialNumber &r) {
  gsof_msgs::msg::ReceiverSerialNumber15 r_ros;
  r_ros.number = r.number;
  return r_ros;
}

gsof_msgs::msg::CurrentTime16 toRosMessage(const trmb::gsof::CurrentTime &c) {
  gsof_msgs::msg::CurrentTime16 c_ros;
  c_ros.header.stamp = toRosTimeOfTheWeek(trmb::gsof::GpsTime{c.gps_week, c.gps_ms_week});
  c_ros.gps_time     = toRosMessage(trmb::gsof::GpsTime{c.gps_week, c.gps_ms_week});
  c_ros.utc_offset   = c.utc_offset;
  c_ros.flags        = c.flags;
  return c_ros;
}

gsof_msgs::msg::AttitudeVariance toRosMessage(const trmb::gsof::AttitudeInfo::Variance &v) {
  gsof_msgs::msg::AttitudeVariance v_ros;
  v_ros.pitch              = v.pitch;
  v_ros.yaw                = v.yaw;
  v_ros.roll               = v.roll;
  v_ros.pitch_yaw          = v.pitch_yaw;
  v_ros.pitch_roll         = v.pitch_roll;
  v_ros.yaw_roll           = v.yaw_roll;
  v_ros.master_slave_range = v.master_slave_range;
  return v_ros;
}

gsof_msgs::msg::AttitudeInfo27 toRosMessage(const trmb::gsof::AttitudeInfo &a) {
  gsof_msgs::msg::AttitudeInfo27 a_ros;
  a_ros.gps_time_of_week          = a.gps_time;
  a_ros.flags                     = a.flags;
  a_ros.number_space_vehicles     = a.num_svs;
  a_ros.calculation_mode          = a.calc_mode.calc_mode;
  a_ros.attitude                  = gsof::toRosMessage(a.pyr);
  a_ros.master_slave_range_meters = a.master_slave_range;
  a_ros.pdop                      = a.pdop;
  if (a.variance.has_value()) {
    a_ros.variance = toRosMessage(*a.variance);
  } else {
    trmb::gsof::AttitudeInfo::Variance v{-1, -1, -1, -1, -1, -1, -1};
    a_ros.variance = toRosMessage(v);
  }
  return a_ros;
}

gsof_msgs::msg::SpaceVehicleBriefInfo toRosMessage(const trmb::gsof::SVBriefInfo &info) {
  gsof_msgs::msg::SpaceVehicleBriefInfo info_ros;
  info_ros.prn       = info.prn;
  info_ros.sv_system = info.sv_system;
  info_ros.sv_flags1 = info.sv_flags1;
  info_ros.sv_flags2 = info.sv_flags2;
  return info_ros;
}

gsof_msgs::msg::AllSvBrief33 toRosMessage(const trmb::gsof::AllSvBrief &all_sv) {
  gsof_msgs::msg::AllSvBrief33 sv_ros;
  sv_ros.sv_info.reserve(all_sv.num_svs);
  for (const auto &sv : all_sv.sv_info) {
    sv_ros.sv_info.emplace_back(toRosMessage(sv));
  }
  return sv_ros;
}

gsof_msgs::msg::SpaceVehicleDetailedInfo toRosMessage(const trmb::gsof::SVDetailedInfo &info) {
  gsof_msgs::msg::SpaceVehicleDetailedInfo info_ros;
  info_ros.prn       = info.prn;
  info_ros.sv_system = info.sv_system;
  info_ros.sv_flags1 = info.sv_flags1;
  info_ros.sv_flags2 = info.sv_flags2;
  info_ros.elevation = info.elevation;
  info_ros.azimuth   = info.azimuth;
  info_ros.snr_l1    = info.snr_L1;
  info_ros.snr_l2    = info.snr_L2;
  info_ros.snr_l5    = info.snr_L5;
  return info_ros;
}

gsof_msgs::msg::AllSvDetailed34 toRosMessage(const trmb::gsof::AllSvDetailed &all_sv) {
  gsof_msgs::msg::AllSvDetailed34 all_sv_ros;
  all_sv_ros.sv_info.reserve(all_sv.num_svs);
  for (const auto &sv : all_sv.sv_info) {
    all_sv_ros.sv_info.emplace_back(toRosMessage(sv));
  }
  return all_sv_ros;
}

gsof_msgs::msg::ReceivedBaseInfo35 toRosMessage(const trmb::gsof::ReceivedBaseInfo &info) {
  gsof_msgs::msg::ReceivedBaseInfo35 info_ros;
  info_ros.flags = info.flags;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
  std::copy(info.name.cbegin(), info.name.cend(), info_ros.name.begin());
#pragma GCC diagnostic pop
  info_ros.id  = info.id;
  info_ros.llh = gsof::toRosLlh(info.base_lla);
  return info_ros;
}

gsof_msgs::msg::BatteryMemoryInfo37 toRosMessage(const trmb::gsof::BatteryMemoryInfo &info) {
  gsof_msgs::msg::BatteryMemoryInfo37 info_ros;
  info_ros.battery_capacity            = info.battery_cap;
  info_ros.remaining_data_logging_time = info.remaining_time;
  return info_ros;
}

gsof_msgs::msg::PositionTypeInformation38 toRosMessage(const trmb::gsof::PositionTypeInformation &info) {
  gsof_msgs::msg::PositionTypeInformation38 info_ros;
  info_ros.error_scale              = info.error_scale;
  info_ros.solution_flags           = info.solution_flags;
  info_ros.rtk_condition            = info.rtk_condition;
  info_ros.correction_age           = info.correction_age;
  info_ros.network_flags            = info.network_flags;
  info_ros.network_flags2           = info.network_flags2;
  info_ros.frame_flag               = info.frame_flag;
  info_ros.itrf_epoch               = info.itrf_epoch;
  info_ros.tectonic_plate           = info.tectonic_plate;
  info_ros.rtx_ram_sub_minutes_left = info.rtx_ram_sub_minutes_left;
  info_ros.pole_wobble_status_flag  = info.pole_wobble_status_flag;
  info_ros.pole_wobble_distance     = info.pole_wobble_distance;
  info_ros.position_fix_type        = info.position_fix_type;
  return info_ros;
}

gsof_msgs::msg::LbandStatusInfo40 toRosMessage(const trmb::gsof::LbandStatusInfo &lband) {
  gsof_msgs::msg::LbandStatusInfo40 lband_ros;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
  std::copy(lband.satellite_name.cbegin(), lband.satellite_name.cend(), lband_ros.satellite_name.begin());
#pragma GCC diagnostic pop
  lband_ros.nominal_sat_freq               = lband.nominal_sat_freq;
  lband_ros.sat_bit_rate                   = lband.sat_bit_rate;
  lband_ros.c_no                           = lband.c_no;
  lband_ros.hpxp_sub_engine                = lband.hpxp_sub_engine;
  lband_ros.hpxp_library_mode              = lband.hpxp_library_mode;
  lband_ros.vbs_library_mode               = lband.vbs_library_mode;
  lband_ros.beam_mode                      = lband.beam_mode;
  lband_ros.omnistar_motion                = lband.omnistar_motion;
  lband_ros.sigma_hor_threshold            = lband.sigma_hor_threshold;
  lband_ros.sigma_ver_threshold            = lband.sigma_ver_threshold;
  lband_ros.nmea_enc_state                 = lband.nmea_enc_state;
  lband_ros.i_q_ratio                      = lband.i_q_ratio;
  lband_ros.estimated_bit_error_rate       = lband.estimated_bit_error_rate;
  lband_ros.total_messages                 = lband.total_messages;
  lband_ros.total_unique_words_with_errors = lband.total_unique_words_with_errors;
  lband_ros.total_bad_unique_word_bits     = lband.total_bad_unique_word_bits;
  lband_ros.total_num_viterbi_symbols      = lband.total_num_viterbi_symbols;
  lband_ros.num_corrected_viterbi_symbols  = lband.num_corrected_viterbi_symbols;
  lband_ros.num_bad_messages               = lband.num_bad_messages;
  lband_ros.meas_frequency_valid_flag      = lband.meas_frequency_valid_flag;
  lband_ros.measured_frequency             = lband.measured_frequency;

  return lband_ros;
}

gsof_msgs::msg::BasePositionAndQualityIndicator41 toRosMessage(const trmb::gsof::BasePositionAndQualityIndicator &ind) {
  trmb::gsof::GpsTime gps_time{ind.gps_week_number, ind.gps_time_ms};
  gsof_msgs::msg::BasePositionAndQualityIndicator41 ind_ros;
  ind_ros.header.stamp = toRosTimeOfTheWeek(gps_time);
  ind_ros.llh          = gsof::toRosLlh(ind.llh);
  ind_ros.gps_time     = toRosMessage(gps_time);
  ind_ros.quality      = ind.quality;
  return ind_ros;
}

gsof_msgs::msg::NavigationSolution49 toRosMessage(const trmb::gsof::NavigationSolution &ins_solution) {
  gsof_msgs::msg::NavigationSolution49 sol;
  sol.header.stamp   = toRosTimeOfTheWeek(ins_solution.gps_time);
  sol.gps_time       = toRosMessage(ins_solution.gps_time);
  sol.status         = toRosMessage(ins_solution.status);
  sol.lla            = gsof::toRosMessage(ins_solution.lla);
  sol.velocity       = gsof::toRosMessage(ins_solution.velocity);
  sol.total_speed    = ins_solution.total_speed;
  sol.roll           = ins_solution.attitude.roll;
  sol.pitch          = ins_solution.attitude.pitch;
  sol.heading        = ins_solution.attitude.heading;
  sol.track_angle    = ins_solution.track_angle;
  sol.ang_rate_long  = ins_solution.angular_rate.roll;
  sol.ang_rate_trans = ins_solution.angular_rate.pitch;
  sol.ang_rate_down  = ins_solution.angular_rate.heading;
  sol.acc_long       = ins_solution.acceleration.x;
  sol.acc_trans      = ins_solution.acceleration.y;
  sol.acc_down       = ins_solution.acceleration.z;

  return sol;
}

gsof_msgs::msg::NavigationPerformance50 toRosMessage(const trmb::gsof::NavigationPerformance &ins_solution_rms) {
  gsof_msgs::msg::NavigationPerformance50 ros_rms;
  ros_rms.header.stamp               = toRosTimeOfTheWeek(ins_solution_rms.gps_time);
  ros_rms.gps_time                   = toRosMessage(ins_solution_rms.gps_time);
  ros_rms.status                     = toRosMessage(ins_solution_rms.status);
  ros_rms.pos_rms_error              = gsof::toRosMessage(ins_solution_rms.position_rms);
  ros_rms.vel_rms_error              = gsof::toRosMessage(ins_solution_rms.velocity_rms);
  ros_rms.attitude_rms_error_roll    = ins_solution_rms.attitude_rms.roll;
  ros_rms.attitude_rms_error_pitch   = ins_solution_rms.attitude_rms.pitch;
  ros_rms.attitude_rms_error_heading = ins_solution_rms.attitude_rms.heading;
  return ros_rms;
}

gsof_msgs::msg::InsVnavFullNavInfo63 toRosMessage(const trmb::gsof::InsVnavFullNavInfo &nav) {
  gsof_msgs::msg::InsVnavFullNavInfo63 nav_ros;
  nav_ros.header.stamp = toRosTimeOfTheWeek(nav.gps_time);
  nav_ros.gps_time     = toRosMessage(nav.gps_time);
  nav_ros.status       = toRosMessage(nav.status);
  nav_ros.lla          = gsof::toRosLla(nav.lla);
  nav_ros.velocity     = gsof::toRosMessage(nav.velocity);
  nav_ros.total_speed  = nav.total_speed;
  nav_ros.roll         = nav.attitude.roll;
  nav_ros.pitch        = nav.attitude.pitch;
  nav_ros.heading      = nav.attitude.heading;
  nav_ros.track_angle  = nav.track_angle;
  nav_ros.angular_rate = gsof::toVector3f(nav.angular_rate);
  nav_ros.acceleration = gsof::toVector3f(nav.acceleration);
  nav_ros.heave        = nav.heave;

  return nav_ros;
}

gsof_msgs::msg::InsVnavRmsInfo64 toRosMessage(const trmb::gsof::InsVnavRmsInfo &rms) {
  gsof_msgs::msg::InsVnavRmsInfo64 rms_ros;
  rms_ros.gps_time               = toRosMessage(rms.gps_time);
  rms_ros.status                 = toRosMessage(rms.status);
  rms_ros.position_rms_meters    = gsof::toRosMessage(rms.position_rms);
  rms_ros.velocity_rms_mps       = gsof::toRosMessage(rms.velocity_rms);
  rms_ros.attitude_rms_degrees.x = rms.attitude_rms.roll;
  rms_ros.attitude_rms_degrees.y = rms.attitude_rms.pitch;
  rms_ros.attitude_rms_degrees.z = rms.attitude_rms.heading;
  rms_ros.heave_rms_meters       = rms.heave_rms;
  return rms_ros;
}

}  // namespace trmb_ros
