include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if (HAVE_OPENAL)
  include_directories(${OPENAL_INCLUDE_DIR})
endif()

include_directories(${TBB_INCLUDEDIR}
                    ${tinyxml_INCLUDE_DIRS}
                    ${tinyxml2_INCLUDE_DIRS}
                    ${IGNITION-TRANSPORT_INCLUDE_DIRS})

link_directories(${tinyxml2_LIBRARY_DIRS} ${IGNITION-MSGS_LIBRARY_DIRS})

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
  gz_build_tests(${gtest_profiler_sources} EXTRA_LIBS gazebo_profiler)

  gz_add_library(gazebo_profiler ${profiler_sources})

endif()

set (sources
  Diagnostics.cc
  IgnMsgSdf.cc
  IntrospectionClient.cc
  IntrospectionManager.cc
  LogPlay.cc
  LogRecord.cc
  OpenAL.cc
)

if (NOT USE_EXTERNAL_TINYXML2)
  list (APPEND sources ${CMAKE_SOURCE_DIR}/deps/tinyxml2/gazebo/tinyxml2.cpp)
endif()

if (NOT USE_EXTERNAL_TINYXML)
  set (sources ${sources}
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinystr.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxml.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlerror.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlparser.cpp)
endif()

set (headers
  Diagnostics.hh
  IgnMsgSdf.hh
  IntrospectionClient.hh
  IntrospectionManager.hh
  LogPlay.hh
  LogRecord.hh
  OpenAL.hh
  UtilTypes.hh
  system.hh
)

set (gtest_sources
  Diagnostics_TEST.cc
  IgnMsgSdf_TEST.cc
  IntrospectionClient_TEST.cc
  IntrospectionManager_TEST.cc
  LogPlay_TEST.cc
  LogRecord_TEST.cc
  OpenAL_TEST.cc
  system.hh
)

gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_util)

set (util_headers "")
foreach (hdr ${headers})
  set (util_headers "${util_headers}#include \"gazebo/util/${hdr}\"\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/util.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/util.hh )

gz_add_library(gazebo_util ${sources})

target_compile_definitions(gazebo_util
  PRIVATE BUILDING_DLL_GZ_UTIL BUILDING_DLL
)

if(ENABLE_PROFILER)
  set(PROFILER_LIBRARIES gazebo_profiler)
endif()

add_dependencies(gazebo_util
  gazebo_common
  gazebo_msgs
  ${PROFILER_LIBRARIES}
)

target_link_libraries(gazebo_util
  gazebo_common
  gazebo_transport
  gazebo_msgs
  ${PROFILER_LIBRARIES}
  ${tinyxml2_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
)

# define if tinxml2 major version >= 6
# https://github.com/ignitionrobotics/ign-common/issues/28
if (NOT tinyxml2_VERSION VERSION_LESS "6.0.0")
  message(STATUS "tinyxml2_VERSION ${tinyxml2_VERSION} >= 6.0.0")
  target_compile_definitions(gazebo_util
    PRIVATE "TINYXML2_MAJOR_VERSION_GE_6")
endif()

if (WIN32)
  include_directories(IGNITION-MSGS_INCLUDE_DIR)
endif()

gz_install_library(gazebo_profiler)
gz_install_includes("profiler" ${profiler_headers})

gz_install_library(gazebo_util)
gz_install_includes("util" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/util.hh)
