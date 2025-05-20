include(CheckCXXCompilerFlag)
include(CMakeParseArguments)
include(GNUInstallDirs)

function(install_with_directory)
  cmake_parse_arguments(ARGS
      ""
      "DESTINATION"
      "FILES"
      ${ARGN})

  foreach(FILE ${ARGS_FILES})
    get_filename_component(DIR ${FILE} DIRECTORY)
    INSTALL(FILES ${FILE} DESTINATION ${ARGS_DESTINATION}/${DIR})
  endforeach(FILE ${ARGS_FILES})
endfunction(install_with_directory)


# trmb_cc_lib()
# Inspiration from abseil's absl_cc_library which emulates Bazel's cc_library
# NAME: name of target
# HDRS: List of header files
# SRCS: List of source files
# DEPS: List of libraries to link to
# COPTS: Compile options
# INC: Public non-system includes
# INC_SYS: System includes
# DEFINES: List of public defines
# INTERFACE: Mark this library as an interface library with no .cpp files to compile
# SHARED: Mark this library as an shared library
function(trmb_cc_lib)
  cmake_parse_arguments(TRMB_CC_LIB
      "INTERFACE;SHARED"
      "NAME"
      "HDRS;SRCS;DEPS;COPTS;INC;INC_SYS;DEFINES"
      ${ARGN})

  set(_NAME "${TRMB_CC_LIB_NAME}")
  if (${TRMB_CC_LIB_INTERFACE})
    add_library(${_NAME} INTERFACE)
    target_sources(${_NAME} INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/${TRMB_CC_LIB_SRCS}>)
    target_include_directories(${_NAME} INTERFACE 
        $<BUILD_INTERFACE:${TRMB_CC_LIB_INC}>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(${_NAME}
        INTERFACE ${TRMB_CC_LIB_DEPS})

    target_include_directories(${_NAME} SYSTEM INTERFACE ${TRMB_CC_LIB_INC_SYS})
    target_compile_definitions(${_NAME} INTERFACE ${TRMB_CC_LIB_DEFINES})
  else ()
    if (${TRMB_CC_LIB_SHARED})
      add_library(${_NAME} SHARED "")
    else()
      add_library(${_NAME} "")
    endif ()
    target_sources(${_NAME} PRIVATE ${TRMB_CC_LIB_SRCS} ${TRMB_CC_LIB_HDRS})
    target_include_directories(${_NAME} PUBLIC 
        $<BUILD_INTERFACE:${TRMB_CC_LIB_INC}>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(${_NAME}
        PUBLIC ${TRMB_CC_LIB_DEPS})
    target_compile_options(${_NAME}
        PRIVATE ${TRMB_COMMON_FLAGS} ${TRMB_CC_LIB_COPTS})
    set_target_properties(${_NAME}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

    target_include_directories(${_NAME} SYSTEM PUBLIC ${TRMB_CC_LIB_INC_SYS})
    target_compile_definitions(${_NAME} PUBLIC ${TRMB_CC_LIB_DEFINES})
  endif (${TRMB_CC_LIB_INTERFACE})

  install(TARGETS ${_NAME}
      EXPORT ${PROJECT_NAME}Targets
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  file(RELATIVE_PATH LIBRARY_PATH ${PROJECT_SOURCE_DIR}/src ${CMAKE_CURRENT_LIST_DIR})
  install_with_directory(
    FILES "${TRMB_CC_LIB_HDRS}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_PATH}")

  add_library(${PROJECT_NAME}::${TRMB_CC_LIB_NAME} ALIAS ${_NAME})
endfunction(trmb_cc_lib)


# trmb_cc_exec()
# NAME: Name of target
# SRCS: List of source files
# DEPS: List of libraries to link to
# COPTS: List of private compile options
# INC: Public non-system includes
# INC_SYS: System includes
# INSTALL_TO_BIN: Install executable to standard bin folder instead of lib/${PROJECT_NAME} as
#                 expected by ROS. Useful for executables usable without ROS.
function(trmb_cc_exec)
  cmake_parse_arguments(TRMB_CC_EXEC
      "INSTALL_TO_BIN"
      "NAME"
      "SRCS;DEPS;COPTS;INC;INC_SYS"
      ${ARGN})
  set(_NAME "${TRMB_CC_EXEC_NAME}")
  add_executable(${_NAME} ${TRMB_CC_EXEC_SRCS})
  target_include_directories(${_NAME} PRIVATE ${TRMB_CC_EXEC_INC})
  target_include_directories(${_NAME} SYSTEM PRIVATE ${TRMB_CC_EXEC_INC_SYS})
  target_compile_options(${_NAME} PRIVATE ${TRMB_COMMON_FLAGS} ${TRMB_CC_EXEC_COPTS})
  target_link_libraries(${_NAME}
      PUBLIC ${TRMB_CC_EXEC_DEPS})

  set_target_properties(${_NAME}
          PROPERTIES
          RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
          LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
          ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
          PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

  if(${TRMB_CC_EXEC_INSTALL_TO_BIN})
    set(_RUNTIME_DEST "${CMAKE_INSTALL_BINDIR}")
  else()
    # This is so ROS may find the project executables
    # normally ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME} but is an issue with bloom-generate
    set(_RUNTIME_DEST "lib/${PROJECT_NAME}")
  endif()

  install(TARGETS ${_NAME}
      EXPORT ${PROJECT_NAME}Targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION ${_RUNTIME_DEST})
endfunction(trmb_cc_exec)


# trmb_cc_test()
# NAME: Name of target
# WORKING_DIR: Where to execute the test from
# SRCS: List of source files
# DEPS: List of libraries to link to
# COPTS: List of private compile options
# INC: Public non-system includes
# INC_SYS: System includes
# LINKOPTS: List of link options
# ADD_GTEST_MAIN: Link to the default googletest main() implementation
function(trmb_cc_test)
  cmake_parse_arguments(TRMB_CC_TEST
    "ADD_GTEST_MAIN"
    "NAME;WORKING_DIR"
    "SRCS;DEPS;COPTS;INC;INC_SYS;LINKOPTS"
    ${ARGN})
  set(_NAME "${TRMB_CC_TEST_NAME}")

  add_executable(${_NAME} ${TRMB_CC_TEST_SRCS})

  target_include_directories(${_NAME} PUBLIC ${TRMB_CC_TEST_INC})
  target_include_directories(${_NAME}
    SYSTEM PUBLIC
      ${TRMB_CC_TEST_INC_SYS})

  target_compile_options(${_NAME} PRIVATE ${TRMB_COMMON_FLAGS} ${TRMB_CC_TEST_COPTS})
  target_link_libraries(${_NAME}
    ${TRMB_CC_TEST_DEPS}
    ${TRMB_CC_TEST_LINKOPTS})

  if(TRMB_CC_TEST_ADD_GTEST_MAIN)
    # Note: for some reason, also linking to gmock makes all the tests undiscoverable
    # See https://github.com/google/googletest/issues/2157
    # The workaround here is if you need gmock then you need to write the main yourself.
    target_link_libraries(${_NAME} GTest::gtest_main)
  else()
    target_link_libraries(${_NAME} GTest::gtest GTest::gmock)
  endif()

  set_target_properties(${_NAME}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
      PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

  gtest_discover_tests(${_NAME}
    WORKING_DIRECTORY ${TRMB_CC_TEST_WORKING_DIR}
    XML_OUTPUT_DIR "${CMAKE_BINARY_DIR}/test_results/")

  # Add TOPSRCDIR
  foreach(script_src IN ITEMS ${TRMB_CC_TEST_SRCS})
    set_property(SOURCE ${script_src} APPEND PROPERTY COMPILE_DEFINITIONS "TOP_SRC_DIR=\"${PROJECT_SOURCE_DIR}\"")
  endforeach()

  list(APPEND TRMB_UNIT_TESTS ${_NAME})

endfunction(trmb_cc_test)


# trmb_setup_compile_flags
# Defines a global variable TRMB_COMMON_FLAGS containing a list of compilation flags
# enabling warnings for all targets compiled by the project.
function(trmb_setup_compile_flags)
  if(NOT MSVC)
    list(APPEND TRMB_COMMON_FLAGS
            -Wall
            -Wextra
            -Wcast-qual
            -Wdouble-promotion
            -Wlogical-op
            -Wno-error=attributes
            -Wold-style-cast
            -Woverlength-strings
            -Wpedantic
            -Wpointer-arith
            -Wreturn-type
            -Wshadow
            -Wswitch
            -Wswitch-enum
            -Wunused-local-typedefs
            -Wunused-result
            -Wuseless-cast
            -Wvarargs
            -Wvla
            -Wwrite-strings)

    check_cxx_compiler_flag(-Wduplicated-cond TRMB_WARN_DUPLICATE_CONDITION)
    if(TRMB_WARN_DUPLICATE_CONDITION)
      list(APPEND TRMB_COMMON_FLAGS -Wduplicated-cond)
    endif()

    check_cxx_compiler_flag(-Wswitch-enum TRMB_WARN_SWITCH_ENUM)
    if(TRMB_WARN_SWITCH_ENUM)
      list(APPEND TRMB_COMMON_FLAGS -Wswitch-enum)
    endif()

    # If we are the main project fail the compile step outright by treating warnings as errors
    if(PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
      list(APPEND TRMB_COMMON_FLAGS
              -Werror)
    endif()
  else(NOT MSVC)
    include(ProcessorCount)
    ProcessorCount(NUM_PROCESSORS)

    # If we are the main project fail the compile step outright by treating warnings as errors
    if(PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
      list(APPEND TRMB_COMMON_FLAGS
              /MP${NUM_PROCESSORS}
              /WX
              /wd4351               # Disable warning telling the user that int a{}; is now valid initialization
              /D_USE_MATH_DEFINES)
    endif()
  endif(NOT MSVC)

  list(REMOVE_DUPLICATES TRMB_COMMON_FLAGS)
  # Make the list of common flags global
  SET(TRMB_COMMON_FLAGS "${TRMB_COMMON_FLAGS}" CACHE INTERNAL "TRMB_COMMON_FLAGS")
endfunction(trmb_setup_compile_flags)


# trmb_setup_code_coverage()
# This function has to be called after all the unit tests have been added to TRMB_UNIT_TESTS
# EXCLUDE: Patterns to exclude (can be relative to BASE_DIRECTORY) e.g., "external/*"
function(trmb_setup_code_coverage)
    cmake_parse_arguments(TRMB_CC_COVERAGE
        ""
        ""
        "EXCLUDE"
        ${ARGN})

    setup_target_for_coverage_gcovr_xml(
        NAME ctest_coverage
        EXECUTABLE ctest
        DEPENDENCIES ${TRMB_UNIT_TESTS}
        BASE_DIRECTORY ${PROJECT_SOURCE_DIR}/src
        EXCLUDE ${TRMB_CC_COVERAGE_EXCLUDE})
endfunction(trmb_setup_code_coverage)
