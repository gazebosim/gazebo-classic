include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources_local
)

set (qt_headers_local
)

set (qt_tests_local

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
