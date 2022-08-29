set(BaselineFootstepPlanner_gtest_list
  TestFootstepPlanner
  )

foreach(NAME IN LISTS BaselineFootstepPlanner_gtest_list)
  catkin_add_gtest(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} BaselineFootstepPlanner)
endforeach()
