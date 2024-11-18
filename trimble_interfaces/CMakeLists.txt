cmake_minimum_required(VERSION 3.10.0)
project(trimble_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(SERVICE_FILES
    "srv/GetMgrsZone.srv"
    "srv/GetUtmZone.srv"
    "srv/GetOrigin.srv"
    "srv/SetOrigin.srv"
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${SERVICE_FILES}
    ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
