add_library(BaselineFootstepPlanner
  FootstepState.cpp
  FootstepEnv.cpp
  FootstepEnvConfigRos.cpp
  FootstepDijkstraPathHeuristic.cpp
  FootstepPlanner.cpp
  )
target_include_directories(BaselineFootstepPlanner PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${SBPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(BaselineFootstepPlanner PUBLIC
  ${SBPL_LIBRARIES}
  ${catkin_LIBRARIES}
  )
add_dependencies(BaselineFootstepPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
