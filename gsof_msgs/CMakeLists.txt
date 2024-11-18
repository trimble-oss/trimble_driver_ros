cmake_minimum_required(VERSION 3.5)

project(gsof_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  # "Fundamental" types
  "msg/AttitudeVariance.msg"
  "msg/EastNorthUpd.msg"
  "msg/EulerAngle.msg"
  "msg/GpsTime.msg"
  "msg/LatLongAltitude.msg"
  "msg/LatLongHeight.msg"
  "msg/NorthEastDownf.msg"
  "msg/SpaceVehicleBriefInfo.msg"
  "msg/SpaceVehicleDetailedInfo.msg"
  "msg/Status.msg"
  "msg/Vector3f.msg"
  # GSOF translations
  "msg/PositionTimeInfo1.msg"
  "msg/LatLongHeight2.msg"
  "msg/EcefPosition3.msg"
  "msg/EcefDelta6.msg"
  "msg/TangentPlaneDelta7.msg"
  "msg/Velocity8.msg"
  "msg/PdopInfo9.msg"
  "msg/ClockInfo10.msg"
  "msg/PositionVcvInfo11.msg"
  "msg/PositionSigmaInfo12.msg"
  "msg/ReceiverSerialNumber15.msg"
  "msg/CurrentTime16.msg"
  "msg/AttitudeInfo27.msg"
  "msg/AllSvBrief33.msg"
  "msg/AllSvDetailed34.msg"
  "msg/ReceivedBaseInfo35.msg"
  "msg/BatteryMemoryInfo37.msg"
  "msg/PositionTypeInformation38.msg"
  "msg/LbandStatusInfo40.msg"
  "msg/BasePositionAndQualityIndicator41.msg"
  "msg/NavigationSolution49.msg"
  "msg/NavigationPerformance50.msg"
  "msg/DmiRawData52.msg"
  "msg/InsVnavFullNavInfo63.msg"
  "msg/InsVnavRmsInfo64.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
