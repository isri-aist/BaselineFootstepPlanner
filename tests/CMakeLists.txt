if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_BFP_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest BaselineFootstepPlanner)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_BFP_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} BaselineFootstepPlanner)
  endfunction()
endif()

set(BaselineFootstepPlanner_gtest_list
  TestFootstepPlanner
  )

foreach(NAME IN LISTS BaselineFootstepPlanner_gtest_list)
  add_BFP_test(${NAME})
endforeach()
