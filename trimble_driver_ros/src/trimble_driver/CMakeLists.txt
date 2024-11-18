add_subdirectory(gsof)
add_subdirectory(network)
add_subdirectory(util)

trmb_cc_lib(
  NAME
    trimble_driver
  SRCS
    gsof_client.cpp
  INC
    ${CMAKE_SOURCE_DIR}/include
  COPTS
    ${COMMON_COMPILE_FLAGS}
  DEPS
    ${PROJECT_NAME}::gsof
    ${PROJECT_NAME}::network
    ${PROJECT_NAME}::util
    Threads::Threads
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
