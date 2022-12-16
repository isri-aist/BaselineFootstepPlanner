add_library(BaselineFootstepPlanner
  FootstepState.cpp
  FootstepEnv.cpp
  FootstepDijkstraPathHeuristic.cpp
  FootstepPlanner.cpp
)
if(DEFINED CATKIN_DEVEL_PREFIX)
  target_include_directories(BaselineFootstepPlanner PUBLIC
    ${catkin_INCLUDE_DIRS}
    )
  target_link_libraries(BaselineFootstepPlanner PUBLIC
    ${catkin_LIBRARIES}
    )
  add_dependencies(BaselineFootstepPlanner
    ${${PROJECT_NAME}_EXPORTED_TARGETS})
else()
  target_compile_definitions(BaselineFootstepPlanner PUBLIC BFP_STANDALONE)
endif()
target_include_directories(BaselineFootstepPlanner PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(BaselineFootstepPlanner PUBLIC PkgConfig::SBPL)
target_compile_features(BaselineFootstepPlanner PUBLIC cxx_std_17)

if(BUILD_SHARED_LIBS)
  set_target_properties(BaselineFootstepPlanner PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
endif()

install(TARGETS BaselineFootstepPlanner
          EXPORT "${TARGETS_EXPORT_NAME}"
          LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/BaselineFootstepPlanner DESTINATION "${INCLUDE_INSTALL_DIR}")
