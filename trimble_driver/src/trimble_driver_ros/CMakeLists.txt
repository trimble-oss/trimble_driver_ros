trmb_cc_lib(
  NAME
    ros_conversions
  SRCS
    conversions.cpp
  DEPS
    ${PROJECT_NAME}::trimble_driver
  INC_SYS
    ${tf2_geometry_msgs_INCLUDE_DIRS}
  COPTS
    ${COMMON_COMPILE_FLAGS}
)

target_link_libraries(ros_conversions
  PUBLIC
    GeographicLib
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${trimble_gsof_msgs_TARGETS}
)

set(CLIENT_LIBRARIES
    ${diagnostic_msgs_TARGETS}
    GeographicLib
    ${nav_msgs_TARGETS}
    rclcpp::rclcpp
    ${rclcpp_components_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS}
    tf2::tf2
    ${tf2_geometry_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${trimble_gsof_msgs_TARGETS}
    ${trimble_interfaces_TARGETS}
)

trmb_cc_lib(
  SHARED
  NAME
    gsof_client_ros
  SRCS
    gsof_topic_lut.cpp
    gsof_client_ros.cpp
  DEPS
    ${PROJECT_NAME}::trimble_driver
    ${PROJECT_NAME}::ros_conversions
  COPTS
    ${COMMON_COMPILE_FLAGS}
)

target_link_libraries(gsof_client_ros PUBLIC ${CLIENT_LIBRARIES})

if(tf2_geometry_msgs_VERSION_MINOR LESS 25)
  target_compile_definitions(ros_conversions PRIVATE TRMB_TF2_HEADER_DEPRECATED)
endif()

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/trimble_driver_ros/
    DESTINATION include/trimble_driver_ros)
