/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#include "trimble_driver_ros/gsof_client_ros.h"

#include <gsof_msgs/msg/navigation_performance50.hpp>
#include <gsof_msgs/msg/navigation_solution49.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace trmb_ros {

GsofClientRos::GsofClientRos(const rclcpp::NodeOptions &options)
    : Node(k_default_node_name, options),
      gsof_client_(nullptr),
      parent_frame_id_(),
      child_frame_id_(),
      local_cartesian_(std::nullopt),
      time_source_(util::RosTimeSource::GPS_TIME_OF_WEEK),
      ros_clock_(RCL_ROS_TIME),
      publish_gsof_msgs_(true),
      publish_ros_msgs_(true),
      publish_rep103_(false),
      publish_tf_(true),
      publishers_(),
      transform_broadcaster_(std::nullopt) {
  setupRosParameters();

  gsof_client_ = std::make_unique<trmb::GsofClient>(ip_, port_);

  this->setupRosPublishersAndCallbacks();

  util::Status status = gsof_client_->start();
  if (!status) {
    RCLCPP_ERROR(this->get_logger(), "Error starting GSOF client: %s", status.error_msg().c_str());
    throw connection_error(status.error_msg());
  }

  RCLCPP_INFO(this->get_logger(), "GSOF client connection started to %s:%ld", ip_.c_str(), port_);
}

void GsofClientRos::setupRosParameters() {
  ip_   = this->declare_parameter("ip", "0.0.0.0");
  port_ = this->declare_parameter("port", 5017);

  parent_frame_id_   = this->declare_parameter("parent_frame", k_default_parent_frame);
  child_frame_id_    = this->declare_parameter("child_frame", k_default_child_frame);
  publish_gsof_msgs_ = this->declare_parameter("publish_gsof_msgs", true);
  publish_ros_msgs_  = this->declare_parameter("publish_ros_msgs", true);
  publish_rep103_    = this->declare_parameter("publish_rep103", true);

  bool publish_tf = this->declare_parameter("publish_tf", true);
  if (publish_tf) {
    transform_broadcaster_.emplace(this);
  }

  std::string time_source = this->declare_parameter("time_source", k_default_time_source);
  time_source_            = util::toRosTimeSource(time_source);
}

template <typename RosMessageType>
auto GsofClientRos::getGsofPublisher(trmb::gsof::Id id) -> rclcpp::Publisher<RosMessageType> * {
  std::size_t index = id - 1;
  if (gsof_publishers_[index] == nullptr) {
    auto topic = GsofTopicLookup::getTopic(id);
    if (topic == nullptr) {
      throw std::runtime_error("Error attempting to create a ROS topic for an unsupported GSOF ID.");
    }
    gsof_publishers_[index] = this->create_publisher<RosMessageType>(topic, k_default_qos_history_depth);
  }

  return std::static_pointer_cast<rclcpp::Publisher<RosMessageType>>(gsof_publishers_[index]).get();
}

void GsofClientRos::setupRosPublishersAndCallbacks() {
  if (!gsof_client_) {
    constexpr char error[] =
        "GsofClientRos::setupRosPublishersAndCallbacks called before concrete "
        "GsofClient was instantiated.";
    RCLCPP_ERROR(this->get_logger(), error);
    throw std::runtime_error(error);
  }

  // Note: Put the save gsof callbacks before the publishing callbacks
  registerCallback(trmb::gsof::GSOF_ID_1_POS_TIME, &GsofClientRos::saveGsof1Callback);
  registerCallback(trmb::gsof::GSOF_ID_2_LLH, &GsofClientRos::saveGsof2Callback);
  registerCallback(trmb::gsof::GSOF_ID_12_POS_SIGMA, &GsofClientRos::saveGsof12Callback);
  registerCallback(trmb::gsof::GSOF_ID_49_INS_FULL_NAV, &GsofClientRos::saveGsof49Callback);
  registerCallback(trmb::gsof::GSOF_ID_50_INS_RMS, &GsofClientRos::saveGsof50Callback);

  if (publish_ros_msgs_) {
    registerAndAdvertise<nav_msgs::msg::Odometry>(trmb::gsof::GSOF_ID_49_INS_FULL_NAV,
                                                  &GsofClientRos::publishInsSolutionCallback,
                                                  k_topic_odometry);
    registerCallback(trmb::gsof::GSOF_ID_1_POS_TIME, &GsofClientRos::publishNavSatCallback);
    advertise<sensor_msgs::msg::NavSatFix>(k_topic_navsat);
  }

  if (publish_gsof_msgs_) {
    registerAllSupportedGsofMessageCallbacks();
  }
  createService<trimble_interfaces::srv::GetOrigin>(k_service_get_origin, &GsofClientRos::getOriginCallback);
  createService<trimble_interfaces::srv::SetOrigin>(k_service_set_origin, &GsofClientRos::setOriginCallback);
  createService<std_srvs::srv::Empty>(k_service_reset_origin, &GsofClientRos::resetOriginCallback);
}

template <typename RosMessageType>
void GsofClientRos::registerAndAdvertise(trmb::gsof::Id id,
                                         GsofClientRos::MessageCallback callback,
                                         const std::string &topic) {
  registerCallback(id, callback);
  advertise<RosMessageType>(topic);
}

void GsofClientRos::registerCallback(trmb::gsof::Id id, GsofClientRos::MessageCallback callback) {
  gsof_client_->registerCallback(id, [this, callback](const trmb::gsof::Message &msg) { (this->*callback)(msg); });
}

void GsofClientRos::registerAllSupportedGsofMessageCallbacks() {
  using namespace trmb::gsof;
  registerCallback(
      GSOF_ID_1_POS_TIME,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::PositionTimeInfo1, PositionTimeInfo, GSOF_ID_1_POS_TIME>);
  registerCallback(GSOF_ID_2_LLH,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::LatLongHeight2, LatLongHeight, GSOF_ID_2_LLH>);
  registerCallback(GSOF_ID_3_ECEF,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::EcefPosition3, EcefPosition, GSOF_ID_3_ECEF>);
  registerCallback(GSOF_ID_6_ECEF_DELTA,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::EcefDelta6, EcefDelta, GSOF_ID_6_ECEF_DELTA>);
  registerCallback(
      GSOF_ID_7_TPLANE_ENU,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::TangentPlaneDelta7, TangentPlaneDelta, GSOF_ID_7_TPLANE_ENU>);
  registerCallback(GSOF_ID_8_VELOCITY,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::Velocity8, Velocity, GSOF_ID_8_VELOCITY>);
  registerCallback(GSOF_ID_9_DOP,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::PdopInfo9, PdopInfo, GSOF_ID_9_DOP>);
  registerCallback(GSOF_ID_10_CLOCK_INFO,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::ClockInfo10, ClockInfo, GSOF_ID_10_CLOCK_INFO>);
  registerCallback(
      GSOF_ID_11_POS_VCV_INFO,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::PositionVcvInfo11, PositionVcvInfo, GSOF_ID_11_POS_VCV_INFO>);
  registerCallback(
      GSOF_ID_12_POS_SIGMA,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::PositionSigmaInfo12, PositionSigmaInfo, GSOF_ID_12_POS_SIGMA>);
  registerCallback(GSOF_ID_15_REC_SERIAL_NUM,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::ReceiverSerialNumber15, ReceiverSerialNumber,
                                                      GSOF_ID_15_REC_SERIAL_NUM>);
  registerCallback(
      GSOF_ID_16_CURR_TIME,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::CurrentTime16, CurrentTime, GSOF_ID_16_CURR_TIME>);
  registerCallback(
      GSOF_ID_27_ATTITUDE,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::AttitudeInfo27, AttitudeInfo, GSOF_ID_27_ATTITUDE>);
  registerCallback(
      GSOF_ID_33_ALL_SV_BRIEF,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::AllSvBrief33, AllSvBrief, GSOF_ID_33_ALL_SV_BRIEF>);
  registerCallback(
      GSOF_ID_34_ALL_SV_DETAIL,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::AllSvDetailed34, AllSvDetailed, GSOF_ID_34_ALL_SV_DETAIL>);
  registerCallback(GSOF_ID_35_RECEIVED_BASE_INFO,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::ReceivedBaseInfo35, ReceivedBaseInfo,
                                                      GSOF_ID_35_RECEIVED_BASE_INFO>);
  registerCallback(GSOF_ID_37_BATTERY_MEM_INFO,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::BatteryMemoryInfo37, BatteryMemoryInfo,
                                                      GSOF_ID_37_BATTERY_MEM_INFO>);
  registerCallback(GSOF_ID_38_POSITION_TYPE_INFO,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::PositionTypeInformation38,
                                                      PositionTypeInformation, GSOF_ID_38_POSITION_TYPE_INFO>);
  registerCallback(
      GSOF_ID_40_LBAND_STATUS,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::LbandStatusInfo40, LbandStatusInfo, GSOF_ID_40_LBAND_STATUS>);
  registerCallback(
      GSOF_ID_41_BASE_POSITION_QUALITY,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::BasePositionAndQualityIndicator41,
                                         BasePositionAndQualityIndicator, GSOF_ID_41_BASE_POSITION_QUALITY>);
  registerCallback(GSOF_ID_49_INS_FULL_NAV,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::NavigationSolution49, NavigationSolution,
                                                      GSOF_ID_49_INS_FULL_NAV>);
  registerCallback(GSOF_ID_50_INS_RMS,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::NavigationPerformance50, NavigationPerformance,
                                                      GSOF_ID_50_INS_RMS>);
  registerCallback(GSOF_ID_63_INS_FULL_NAV_KRYPTON,
                   &GsofClientRos::publishGsofMessage<gsof_msgs::msg::InsVnavFullNavInfo63, InsVnavFullNavInfo,
                                                      GSOF_ID_63_INS_FULL_NAV_KRYPTON>);
  registerCallback(
      GSOF_ID_64_INS_RMS_KRYPTON,
      &GsofClientRos::publishGsofMessage<gsof_msgs::msg::InsVnavRmsInfo64, InsVnavRmsInfo, GSOF_ID_64_INS_RMS_KRYPTON>);
}

template <typename ServiceType>
void GsofClientRos::createService(const std::string &service_name, ServiceCallback<ServiceType> service_callback) {
  auto lambda_bound_to_this = [this, service_callback](std::shared_ptr<rmw_request_id_t> request_header,
                                                       typename ServiceType::Request::SharedPtr request,
                                                       typename ServiceType::Response::SharedPtr response) -> void {
    (this->*service_callback)(request_header, request, response);
  };

  typename rclcpp::Service<ServiceType>::SharedPtr service =
      this->create_service<ServiceType>(service_name, lambda_bound_to_this);
  services_.emplace_back(service);  // Should auto case up to std::shared_ptr<rclcpp::ServiceBase>
}

void GsofClientRos::saveGsof1Callback(const trmb::gsof::Message &message) {
  position_time_info_.emplace(message.as<trmb::gsof::PositionTimeInfo>());
}

void GsofClientRos::saveGsof2Callback(const trmb::gsof::Message &message) {
  // We are only grabbing the first message to fill out local_cartesian_
  if (!local_cartesian_) {
    const auto &lla = message.as<trmb::gsof::LatLongHeight>();
    using trmb::rad2deg;  // GSOF 2 returns Lat/Long in radians, local_cartesian_ is in degrees
    RCLCPP_INFO(this->get_logger(), "Initializing local cartesian plane to LLA: %4.6f %4.6f %4.6f",
                rad2deg(lla.lla.latitude), rad2deg(lla.lla.longitude), lla.lla.altitude);
    local_cartesian_.emplace(rad2deg(lla.lla.latitude), rad2deg(lla.lla.longitude), lla.lla.altitude,
                             GeographicLib::Geocentric::WGS84());
  }
  // Store the message normally to be converted to a NavSatFix message and published
  lat_long_height_.emplace(message.as<trmb::gsof::LatLongHeight>());
}

void GsofClientRos::saveGsof12Callback(const trmb::gsof::Message &message) {
  position_sigma_info_.emplace(message.as<trmb::gsof::PositionSigmaInfo>());
}

void GsofClientRos::saveGsof49Callback(const trmb::gsof::Message &message) {
  const auto &ins_solution = message.as<trmb::gsof::NavigationSolution>();
  ins_solution_.emplace(ins_solution);

  if (!local_cartesian_) {
    const auto &lla = ins_solution.lla;
    RCLCPP_INFO(this->get_logger(), "Initializing local cartesian plane to LLA: %4.6f %4.6f %4.6f", lla.latitude,
                lla.longitude, lla.altitude);
    local_cartesian_.emplace(lla.latitude, lla.longitude, lla.altitude, GeographicLib::Geocentric::WGS84());
  }
}

void GsofClientRos::saveGsof50Callback(const trmb::gsof::Message &message) {
  ins_solution_rms_.emplace(message.as<trmb::gsof::NavigationPerformance>());
}

void GsofClientRos::publishNavSatCallback(const trmb::gsof::Message &) {
  // Only publish if GSOF messages #1, 2, and 12 have been received AND we are not receiving GSOF #49
  if ((position_time_info_ && lat_long_height_ && position_sigma_info_) && !ins_solution_) {
    sensor_msgs::msg::NavSatFix nav_sat_fix =
        toNavSatFix(*position_time_info_, *lat_long_height_, *position_sigma_info_);
    nav_sat_fix.header.frame_id = child_frame_id_;  // Using the sensor frame
    // Set the time
    trmb::gsof::GpsTime gps_time_{position_time_info_->gps_week, position_time_info_->gps_time_ms};
    nav_sat_fix.header.stamp = getRosTimestamp(gps_time_);

    using NavSatFix  = sensor_msgs::msg::NavSatFix;
    auto nav_sat_pub = std::static_pointer_cast<rclcpp::Publisher<NavSatFix>>(publishers_[k_topic_navsat]);
    nav_sat_pub->publish(nav_sat_fix);
  }
}

void GsofClientRos::publishInsSolutionCallback(const trmb::gsof::Message &) {
  sensor_msgs::msg::NavSatFix nav_sat_fix =
      ins_solution_rms_ ? toNavSatFix(*ins_solution_, *ins_solution_rms_) : toNavSatFix(*ins_solution_);

  nav_sat_fix.header.frame_id = child_frame_id_;  // Our child frame is the sensor frame
  nav_sat_fix.header.stamp    = getRosTimestamp(ins_solution_->gps_time);

  using NavSatFix  = sensor_msgs::msg::NavSatFix;
  auto nav_sat_pub = std::static_pointer_cast<rclcpp::Publisher<NavSatFix>>(publishers_[k_topic_navsat]);
  nav_sat_pub->publish(nav_sat_fix);

  nav_msgs::msg::Odometry odom;
  if (publish_rep103_) {
    odom = ins_solution_rms_ ? toRep103(*ins_solution_, *local_cartesian_, *ins_solution_rms_)
                             : toRep103(*ins_solution_, *local_cartesian_);
  } else {
    odom = ins_solution_rms_ ? toOdometry(*ins_solution_, *local_cartesian_, *ins_solution_rms_)
                             : toOdometry(*ins_solution_, *local_cartesian_);
  }

  odom.child_frame_id  = child_frame_id_;
  odom.header.frame_id = parent_frame_id_;
  odom.header.stamp    = nav_sat_fix.header.stamp;  // Don't reconvert gps time in case we are using ros time now

  using Odometry = nav_msgs::msg::Odometry;
  auto odom_pub  = std::static_pointer_cast<rclcpp::Publisher<Odometry>>(publishers_[k_topic_odometry]);
  odom_pub->publish(odom);

  if (transform_broadcaster_) {
    transform_broadcaster_->sendTransform(toTransformStamped(odom.header, odom.child_frame_id, odom.pose.pose));
  }
}

rclcpp::Time GsofClientRos::getRosTimestamp(const trmb::gsof::GpsTime &gps_time) {
  switch (time_source_) {
    case util::RosTimeSource::NOW:
      return ros_clock_.now();
    case util::RosTimeSource::GPS_TIME_OF_WEEK:
      return toRosTimeOfTheWeek(gps_time);
    case util::RosTimeSource::GPS:
      return toRosTimeGpsEpoch(gps_time);
    default:
      // Should never happen because we are protected by -Wswitch-enum
      throw std::logic_error("Unhandled RosTimeSource.");
  }
}

void GsofClientRos::getOriginCallback(const std::shared_ptr<rmw_request_id_t>,
                                      const trimble_interfaces::srv::GetOrigin::Request::SharedPtr,
                                      trimble_interfaces::srv::GetOrigin::Response::SharedPtr response) {
  if (local_cartesian_) {
    response->success   = true;
    response->latitude  = local_cartesian_->LatitudeOrigin();
    response->longitude = local_cartesian_->LongitudeOrigin();
    response->altitude  = local_cartesian_->HeightOrigin();
  } else {
    response->success = false;
    RCLCPP_ERROR(
        this->get_logger(),
        "Error: Local cartesian plane origin has not been set. Please ensure GSOF #2 or #49 are being broadcast by "
        "your hardware.");
  }
}

void GsofClientRos::setOriginCallback(const std::shared_ptr<rmw_request_id_t>,
                                      const trimble_interfaces::srv::SetOrigin::Request::SharedPtr request,
                                      trimble_interfaces::srv::SetOrigin::Response::SharedPtr) {
  if (local_cartesian_) {
    local_cartesian_->Reset(request->latitude, request->longitude, request->altitude);
  } else {
    local_cartesian_.emplace(request->latitude, request->longitude, request->altitude);
  }

  RCLCPP_INFO(this->get_logger(),
              "Local Tangent Plane origin reset to Lat: %f Lon: %f Alt: %f",
              request->latitude,
              request->longitude,
              request->altitude);
}

void GsofClientRos::resetOriginCallback(const std::shared_ptr<rmw_request_id_t>,
                                        const std_srvs::srv::Empty::Request::SharedPtr,
                                        std_srvs::srv::Empty::Response::SharedPtr) {
  local_cartesian_.reset();
  RCLCPP_INFO(this->get_logger(), "Local Tangent Plane origin reset.");
}

}  // namespace trmb_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trmb_ros::GsofClientRos)
