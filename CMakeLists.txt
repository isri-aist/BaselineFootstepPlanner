cmake_minimum_required(VERSION 3.14)

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
option(USE_ROS2 "Use ROS2" OFF)

project(${PROJECT_NAME} LANGUAGES CXX C)

if(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
else()
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

# SBPL
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})

if(USE_ROS2)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Footstep2D.msg"
    "msg/FootstepSequence2D.msg"
    "msg/FootstepSequence2DStamped.msg"
    DEPENDENCIES std_msgs geometry_msgs
  )

  rosidl_get_typesupport_target(cpp_typesupport_target 
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endif()

add_subdirectory(src)

if(USE_ROS2)
  add_subdirectory(node)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(USE_ROS2)
  ament_export_dependencies(rclcpp rosidl_default_runtime std_msgs geometry_msgs sensor_msgs SBPL)
  ament_export_include_directories(include)
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
endif()