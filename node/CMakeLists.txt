add_executable(FootstepPlannerNode
  FootstepPlannerNode.cpp)
target_link_libraries(FootstepPlannerNode PRIVATE
  BaselineFootstepPlanner)
install(TARGETS FootstepPlannerNode DESTINATION lib/${PROJECT_NAME})
