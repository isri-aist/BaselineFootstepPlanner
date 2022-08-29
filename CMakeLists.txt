cmake_minimum_required(VERSION 3.1)
project(baseline_footstep_planner)

# If CMAKE_CXX_STANDARD is not specified, the features specified in target_compile_features or INTERFACE_COMPILE_FEATURES
# may cause -std=gnu++11 to be added to the compile options and overwrite the C++ version specified in add_compile_options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(
  -std=c++17
  -Wno-terminate # for SBPL
  )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  )

# SBPL
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED sbpl)
message(NOTICE "- SBPL_INCLUDE_DIRS: ${SBPL_INCLUDE_DIRS}")
message(NOTICE "- SBPL_LIBRARY_DIRS: ${SBPL_LIBRARY_DIRS}")
link_directories(${SBPL_LIBRARY_DIRS})

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
  DEPENDS SBPL
  INCLUDE_DIRS include
  LIBRARIES BaselineFootstepPlanner
  )

add_subdirectory(src)

add_subdirectory(node)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

# OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
# if(INSTALL_DOCUMENTATION)
#   add_subdirectory(doc)
# endif()
