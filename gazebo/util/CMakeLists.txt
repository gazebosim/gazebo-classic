include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if (HAVE_OPENAL)
  include_directories(${OPENAL_INCLUDE_DIR})
endif()

include_directories(${TBB_INCLUDEDIR}
                    ${tinyxml_INCLUDE_DIRS}
                    ${tinyxml2_INCLUDE_DIRS}
                    ${IGNITION-TRANSPORT_INCLUDE_DIRS})

link_directories(${tinyxml2_LIBRARY_DIRS} ${IGNITION-MSGS_LIBRARY_DIRS})

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

add_dependencies(gazebo_util
  gazebo_common
  gazebo_msgs
)

target_link_libraries(gazebo_util
  gazebo_common
  gazebo_transport
  gazebo_msgs
  ${tinyxml2_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
)

# define if tinxml2 major version >= 6
# https://bitbucket.org/ignitionrobotics/ign-common/issues/28
if (NOT tinyxml2_VERSION VERSION_LESS "6.0.0")
  message(STATUS "tinyxml2_VERSION ${tinyxml2_VERSION} >= 6.0.0")
  target_compile_definitions(gazebo_util
    PRIVATE "TINYXML2_MAJOR_VERSION_GE_6")
endif()

if (WIN32)
  include_directories(IGNITION-MSGS_INCLUDE_DIR)
endif()


gz_install_library(gazebo_util)
gz_install_includes("util" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/util.hh)
