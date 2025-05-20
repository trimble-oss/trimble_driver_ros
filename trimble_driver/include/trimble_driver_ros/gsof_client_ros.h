/*
 * Copyright (c) 2024. Trimble Inc.
 * All rights reserved.
 */

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trimble_interfaces/srv/get_origin.hpp>
#include <trimble_interfaces/srv/set_origin.hpp>

#include "trimble_driver/gsof/gsof.h"
#include "trimble_driver/gsof_client.h"
#include "trimble_driver/util/ros_time_source.h"
#include "trimble_driver_ros/conversions.h"
#include "trimble_driver_ros/gsof_topic_lut.h"

namespace trmb_ros {

class GsofClientRos : public rclcpp::Node {
 public:
  struct connection_error : public std::runtime_error {
    explicit connection_error(const std::string &msg = "")
        : std::runtime_error("GsofClientRos could not start a connection to device. " + msg) {}
  };

  explicit GsofClientRos(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 protected:
  static constexpr int k_default_qos_history_depth = 100;

  std::string ip_;
  long port_;
  std::unique_ptr<trmb::GsofClient> gsof_client_;

  // Trade space for time complexity 😅
  std::array<std::shared_ptr<rclcpp::PublisherBase>, 256> gsof_publishers_;

  void setupRosPublishersAndCallbacks();

  template <typename RosMessageType>
  void advertise(const std::string &topic);

 private:
  static constexpr char k_default_node_name[] = "gsof_client";

  static constexpr char k_topic_odometry[] = "~/odom";
  static constexpr char k_topic_navsat[]   = "~/navsat";

  static constexpr char k_service_get_origin[]   = "~/get_origin";
  static constexpr char k_service_set_origin[]   = "~/set_origin";
  static constexpr char k_service_reset_origin[] = "~/reset_origin";

  static constexpr char k_default_parent_frame[] = "ned";
  static constexpr char k_default_child_frame[]  = "ins";
  static constexpr char k_default_time_source[]  = "now";

  std::string parent_frame_id_;
  std::string child_frame_id_;

  std::optional<GeographicLib::LocalCartesian> local_cartesian_;

  util::RosTimeSource time_source_;
  rclcpp::Clock ros_clock_;
  bool publish_gsof_msgs_;
  bool publish_ros_msgs_;
  bool publish_rep103_;
  bool publish_tf_;

  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publishers_;
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
  std::optional<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::optional<trmb::gsof::PositionTimeInfo> position_time_info_;
  std::optional<trmb::gsof::LatLongHeight> lat_long_height_;
  std::optional<trmb::gsof::PositionSigmaInfo> position_sigma_info_;
  std::optional<trmb::gsof::NavigationSolution> ins_solution_;
  std::optional<trmb::gsof::NavigationPerformance> ins_solution_rms_;

  void setupRosParameters();

  template <typename RosMessageType>
  auto getGsofPublisher(trmb::gsof::Id id) -> rclcpp::Publisher<RosMessageType> *;

  using MessageCallback = void (GsofClientRos::*)(const trmb::gsof::Message &msg);
  template <typename RosMessageType>
  void registerAndAdvertise(trmb::gsof::Id id, MessageCallback callback, const std::string &topic);
  void registerCallback(trmb::gsof::Id id, MessageCallback callback);
  void registerAllSupportedGsofMessageCallbacks();
  template <typename ServiceType>
  using ServiceCallback = void (GsofClientRos::*)(const std::shared_ptr<rmw_request_id_t>,
                                                  const typename ServiceType::Request::SharedPtr,
                                                  typename ServiceType::Response::SharedPtr);
  template <typename ServiceType>
  void createService(const std::string &service_name, ServiceCallback<ServiceType> service_callback);

  // Callbacks who's only job is to save data in private members
  void saveGsof1Callback(const trmb::gsof::Message &message);
  void saveGsof2Callback(const trmb::gsof::Message &message);
  void saveGsof12Callback(const trmb::gsof::Message &message);
  void saveGsof49Callback(const trmb::gsof::Message &message);
  void saveGsof50Callback(const trmb::gsof::Message &message);

  // Callbacks to publish data in "standard" ROS messages
  void publishNavSatCallback(const trmb::gsof::Message &message);
  void publishInsSolutionCallback(const trmb::gsof::Message &message);

  /**
   * Callback for translating a GSOF message to ROS, specifically for the case where GpsTime is present and could be
   * translated to rclcpp time
   * @tparam RosMessageType     A type generated using the ROS IDL
   * @tparam NativeMessageType  A C++ GSOF type
   * @tparam gsof_id            The GSOF record id
   * @param message             The GSOF message to be published
   */
  template <class RosMessageType,
            class NativeMessageType,
            std::size_t gsof_id,
            std::enable_if_t<HasStdMsgsHeader<RosMessageType>::value && HasGsofGpsTime<NativeMessageType>::value,
                             bool> = true>
  void publishGsofMessage(const trmb::gsof::Message &message) {
    auto publisher       = getGsofPublisher<RosMessageType>(gsof_id);
    auto gsof_msg        = message.as<NativeMessageType>();
    auto ros_msg         = toRosMessage(gsof_msg);
    ros_msg.header.stamp = getRosTimestamp(gsof_msg.gps_time);
    publisher->publish(ros_msg);
  }

  /**
   * Callback for translating a GSOF message to ROS when there is no time stamp available.
   * @tparam RosMessageType     A type generated using the ROS IDL
   * @tparam NativeMessageType  A C++ GSOF type
   * @tparam gsof_id            The GSOF record id
   * @param message             The GSOF message to be published
   */
  template <class RosMessageType,
            class NativeMessageType,
            std::size_t gsof_id,
            std::enable_if_t<!HasStdMsgsHeader<RosMessageType>::value || !HasGsofGpsTime<NativeMessageType>::value,
                             bool> = true>
  void publishGsofMessage(const trmb::gsof::Message &message) {
    auto publisher = getGsofPublisher<RosMessageType>(gsof_id);
    publisher->publish(toRosMessage(message.as<NativeMessageType>()));
  }

  rclcpp::Time getRosTimestamp(const trmb::gsof::GpsTime &gps_time);

  // Service callbacks
  void getOriginCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const trimble_interfaces::srv::GetOrigin::Request::SharedPtr request,
                         trimble_interfaces::srv::GetOrigin::Response::SharedPtr response);

  void setOriginCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const trimble_interfaces::srv::SetOrigin::Request::SharedPtr request,
                         trimble_interfaces::srv::SetOrigin::Response::SharedPtr response);

  void resetOriginCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std_srvs::srv::Empty::Request::SharedPtr request,
                           std_srvs::srv::Empty::Response::SharedPtr response);
};

template <typename RosMessageType>
void GsofClientRos::advertise(const std::string &topic) {
  publishers_[topic] = this->create_publisher<RosMessageType>(topic, k_default_qos_history_depth);
}

}  // namespace trmb_ros
