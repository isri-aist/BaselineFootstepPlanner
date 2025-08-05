if(USE_ROS2)
  set(ROS_DEPENDENT_SRCS FootstepEnvConfigRos.cpp)
else()
  set(ROS_DEPENDENT_SRCS "")
endif()
add_library(BaselineFootstepPlanner
  FootstepState.cpp
  FootstepEnv.cpp
  FootstepDijkstraPathHeuristic.cpp
  FootstepPlanner.cpp
  ${ROS_DEPENDENT_SRCS}
)
if(USE_ROS2)
  ament_target_dependencies(BaselineFootstepPlanner PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
  )

  target_link_libraries(BaselineFootstepPlanner PUBLIC ${cpp_typesupport_target})
else()
  target_compile_definitions(BaselineFootstepPlanner PUBLIC BFP_STANDALONE)
endif()
target_include_directories(BaselineFootstepPlanner PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_directories(BaselineFootstepPlanner PUBLIC ${SBPL_LIBRARY_DIRS})
target_link_libraries(BaselineFootstepPlanner PUBLIC ${SBPL_LIBRARIES})
target_compile_features(BaselineFootstepPlanner PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(BaselineFootstepPlanner PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS BaselineFootstepPlanner
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/BaselineFootstepPlanner DESTINATION include)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts/ DESTINATION lib/${PROJECT_NAME})
