cmake_minimum_required(VERSION 3.1)
project(baseline_footstep_planner)

# If CMAKE_CXX_STANDARD is not specified, the features specified in target_compile_features or INTERFACE_COMPILE_FEATURES
# may cause -std=gnu++11 to be added to the compile options and overwrite the C++ version specified in add_compile_options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

# SBPL
find_package(PkgConfig)
pkg_check_modules(SBPL REQUIRED sbpl)
message(NOTICE "- SBPL_INCLUDE_DIRS: ${SBPL_INCLUDE_DIRS}")
message(NOTICE "- SBPL_LIBRARY_DIRS: ${SBPL_LIBRARY_DIRS}")
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  DEPENDS SBPL
  INCLUDE_DIRS include
  LIBRARIES BaselineFootstepPlanner
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)

# OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
# if(INSTALL_DOCUMENTATION)
#   add_subdirectory(doc)
# endif()
