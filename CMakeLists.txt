cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME baseline_footstep_planner)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/BaselineFootstepPlanner)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX C)

if(DEFINED USE_ROS2)
  set(DOXYGEN_HTML_OUTPUT html)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

# SBPL
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED IMPORTED_TARGET sbpl)
if(NOT DEFINED USE_ROS2)
  set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED IMPORTED_TARGET sbpl)")
endif()

if(DEFINED USE_ROS2)
  set(msg_files
    "msg/Footstep2D.msg"
    "msg/FootstepSequence2D.msg"
    "msg/FootstepSequence2DStamped.msg"
  ) 

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs
  )

  ament_export_dependencies(rclcpp rosidl_default_runtime std_msgs geometry_msgs sensor_msgs SBPL)
  ament_export_include_directories(include)
  ament_export_libraries(BaselineFootstepPlanner)
  ament_package()
endif()

add_subdirectory(src)

if(DEFINED USE_ROS2)
  add_subdirectory(node)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
