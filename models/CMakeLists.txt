# The set of built-in models should be very small. Only the most essential
# models needed to make Gazebo run without an internet connection should be
# included here.
set (dirs
  ground_plane
  sun
  plowing_effect_trisphere_cycle
  plowing_effect_ground_plane
)

foreach(dir ${dirs})
  install (DIRECTORY ${dir} DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/models)
endforeach()
