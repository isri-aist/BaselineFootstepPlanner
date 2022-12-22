cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME baseline_footstep_planner)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/${PROJECT_NAME})
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
  )
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(CATKIN_ENABLE_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

# SBPL
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED IMPORTED_TARGET sbpl)
if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED IMPORTED_TARGET sbpl)")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  add_message_files(
    FILES
      Footstep2D.msg
      FootstepSequence2D.msg
      FootstepSequence2DStamped.msg
  )
  generate_messages(
    DEPENDENCIES std_msgs geometry_msgs
  )
  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      roscpp
      std_msgs
      geometry_msgs
      sensor_msgs
    DEPENDS
      SBPL
    INCLUDE_DIRS
      include
    LIBRARIES
      BaselineFootstepPlanner
  )
endif()

add_subdirectory(src)

if(DEFINED CATKIN_DEVEL_PREFIX)
  add_subdirectory(node)
endif()

if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
