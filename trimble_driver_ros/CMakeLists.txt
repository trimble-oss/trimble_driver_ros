cmake_minimum_required(VERSION 3.20.0)
project(trimble_driver_ros
    VERSION 0.1.0
	LANGUAGES CXX)

option(CODE_COVERAGE "Enable code coverage flags and targets." OFF)

set(ROS_VERSION $ENV{ROS_VERSION})
set(CMAKE_CXX_STANDARD 17)

include(cmake/macros.cmake)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(PCAP REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

set(COMMON_COMPILE_FLAGS
    -Wall
    -Wextra
    -Wcast-qual
    -Wdouble-promotion
    -Werror
    -Wlogical-op
    -Wno-error=attributes
    -Wold-style-cast
    -Woverlength-strings
    -Wpedantic
    -Wpointer-arith
    -Wshadow
    -Wswitch-enum
    -Wunused-local-typedefs
    -Wunused-result
    -Wuseless-cast
    -Wvarargs
    -Wvla
    -Wwrite-strings)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gsof_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trimble_interfaces REQUIRED)

if (CODE_COVERAGE)
    message(STATUS "Building with code coverage enabled")
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

add_subdirectory(src)

## Testing

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(test)
endif()

if(CODE_COVERAGE)
    # Make sure to call this function only after all test targets have been created
    trmb_setup_code_coverage()
endif ()

## Install

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

# These macros only work in the same scope as ament_package() due to a
# variable not being set in the cache
rclcpp_components_register_nodes(
    gsof_client_ros "trimble_driver_ros::GsofClientRos")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)
  
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Threads
  ament_cmake 
  ament_cmake_ros
  diagnostic_msgs 
  gsof_msgs 
  geometry_msgs
  nav_msgs 
  rclcpp 
  rclcpp_components 
  sensor_msgs 
  std_msgs 
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trimble_interfaces)
ament_package(CONFIG_EXTRAS
  cmake/FindGMock.cmake
  cmake/FindPCAP.cmake)