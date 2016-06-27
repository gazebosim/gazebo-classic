include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources_local
  plot/Palette.cc
)

set (qt_headers_local
  plot/Palette.hh
)

set (headers_install
)

set (qt_tests_local
  Palette_TEST.cc
)

if (ENABLE_SCREEN_TESTS)
  set (qt_tests_local
    ${qt_tests_local}

  )
endif()

set (sources ${sources} ${sources_local} PARENT_SCOPE)
set (internal_qt_headers ${internal_qt_headers} ${qt_headers_local} PARENT_SCOPE)

gz_build_qt_tests(${qt_tests_local})
gz_install_includes("gui/plot" ${headers_install})
