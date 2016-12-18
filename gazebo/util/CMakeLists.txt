include (${gazebo_cmake_dir}/GazeboUtils.cmake)

add_definitions(-DBUILDING_DLL_GZ_UTIL)

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

set (util_headers "" CACHE INTERNAL "util headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(util_headers
    "Utility Headers" "#include \"gazebo/util/${hdr}\"\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/util.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/util.hh )


if (WIN32)
  add_definitions(-DBUILDING_DLL)
endif()

gz_add_library(gazebo_util ${sources})

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

if (WIN32)
  include_directories(IGNITION-MSGS_INCLUDE_DIR)
  set_target_properties(gazebo_util PROPERTIES COMPILE_FLAGS "/FIignition/msgs/System.hh")
endif()


gz_install_library(gazebo_util)
gz_install_includes("util" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/util.hh)
