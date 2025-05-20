add_subdirectory(trimble_driver)
add_subdirectory(trimble_driver_ros)

trmb_cc_exec(
  NAME
    gsof_client_node
  SRCS
    gsof_client_node_ros.cpp
  DEPS
    ${PROJECT_NAME}::gsof_client_ros
    ${rclcpp_LIBRARIES}
  INC_SYS
    ${rclcpp_INCLUDE_DIRS}
  COPTS
    ${COMMON_COMPILE_FLAGS}
)
