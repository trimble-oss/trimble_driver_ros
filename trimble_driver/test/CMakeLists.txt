find_package(GTest REQUIRED)
include(GoogleTest)  # get gtest_discover_tests and other functions/macros
include(CTest)

trmb_cc_test(
    NAME
        trimble_driver_test
    SRCS
        test_byteswap.cpp
        test_client.cpp
        test_gsof.cpp
        test_gsof_streaming.cpp
        test_ros_conversions.cpp
    DEPS
        ${PROJECT_NAME}::trimble_driver
        ${PROJECT_NAME}::ros_conversions
        tf2_geometry_msgs::tf2_geometry_msgs
    COPTS
        ${COMMON_COMPILE_FLAGS}
    WORKING_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/data
    ADD_GTEST_MAIN
)

if(tf2_geometry_msgs_VERSION_MINOR LESS 25)
  target_compile_definitions(trimble_driver_test PRIVATE TRMB_TF2_HEADER_DEPRECATED)
endif()

# Regression tests against captures from real receivers. NO_DISCOVER keeps them out of ctest, and therefore out of
# `colcon test` and the coverage run, because they depend on the Git LFS payloads in test/data. Run them with
# `cd trimble_driver/test/data && <build>/bin/trimble_driver_pcap_test`.
trmb_cc_test(
    NAME
        trimble_driver_pcap_test
    SRCS
        test_gsof_pcap.cpp
        test_gsof_streaming_pcap.cpp
        test_ros_conversions_pcap.cpp
    DEPS
        ${PROJECT_NAME}::trimble_driver
        ${PROJECT_NAME}::ros_conversions
        tf2_geometry_msgs::tf2_geometry_msgs
    COPTS
        ${COMMON_COMPILE_FLAGS}
    WORKING_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}/data
    ADD_GTEST_MAIN
    NO_DISCOVER
)

if(tf2_geometry_msgs_VERSION_MINOR LESS 25)
  target_compile_definitions(trimble_driver_pcap_test PRIVATE TRMB_TF2_HEADER_DEPRECATED)
endif()
