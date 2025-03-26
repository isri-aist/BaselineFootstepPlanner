find_package(Doxygen REQUIRED)

if(DOXYGEN_FOUND)
  set(DOXYFILE_PATH ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  if (USE_ROS2)
    set(DOXYGEN_HTML_OUTPUT doxygen-html)
  endif()

  add_custom_target(BaselineFootstepPlanner_doc ALL
    ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH}
    DEPENDS ${DOXYFILE_PATH}
    COMMENT "Generating Doxygen documentation"
    )
endif()
