include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if(ENABLE_PROFILER)
  set(
    PROFILER_SRCS
    Profiler.cc
  )

  set(
    PROFILER_HEADERS
    Profiler.hh
  )

  set(
    PROFILER_TESTS
  )

  set(
    Remotery_SRC
    ./Remotery/lib/Remotery.c
    ./Remotery/lib/Remotery.h
  )

  if(APPLE)
    set(
      Remotery_SRC
      ${Remotery_SRC}
      ./Remotery/lib/RemoteryMetal.mm
    )
    find_library(FOUNDATION Foundation
      HINTS /System/Library/Frameworks
    )
  endif()

  set(RMT_ENABLED 1)
  set(RMT_USE_TINYCRT 0)
  set(RMT_USE_CUDA 0)
  set(RMT_USE_D3D11 0)
  set(RMT_USE_OPENGL 0)
  set(RMT_USE_METAL 0)

  if(UNIX OR WIN32)
    set(RMT_USE_OPENGL 1)
  endif()

  if(WIN32)
    set(RMT_USE_D3D11 1)
  endif()

  if(APPLE)
    set (RMT_USE_METAL 1)
  endif()

  configure_file(RemoteryConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/RemoteryConfig.h)

  list(APPEND PROFILER_SRCS ${Remotery_SRC} RemoteryProfilerImpl.cc)

  set (profiler_sources
    ${PROFILER_SRCS}
  )

  set (headers
    ${PROFILER_HEADERS}
  )

  set (gtest_profiler_sources
    ${PROFILER_TESTS}
  )

  gz_add_library(gazebo_profiler ${profiler_sources})

  gz_install_library(gazebo_profiler)

  gz_install_includes("profiler" Profiler.hh)

  target_compile_definitions(gazebo_profiler
    PRIVATE BUILDING_DLL_GZ_PROFILER
  )

endif()
