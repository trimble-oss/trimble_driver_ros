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

ament_target_dependencies(ros_conversions
  PUBLIC
    gsof_msgs
    GeographicLib
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_geometry_msgs
)

set(CLIENT_LIBRARIES
    diagnostic_msgs
    gsof_msgs
    GeographicLib
    nav_msgs
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trimble_interfaces
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

ament_target_dependencies(gsof_client_ros PUBLIC ${CLIENT_LIBRARIES})

if(tf2_geometry_msgs_VERSION_MINOR LESS 25)
  target_compile_definitions(ros_conversions PRIVATE TRMB_TF2_HEADER_DEPRECATED)
endif()

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/trimble_driver_ros/
    DESTINATION include/trimble_driver_ros)
