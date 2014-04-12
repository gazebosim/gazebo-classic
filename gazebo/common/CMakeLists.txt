include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  Events.cc
  ModelDatabase.cc
  Plugin.cc
  SystemPaths.cc
)

set (headers
  Events.hh
  ModelDatabase.hh
  ModelDatabasePrivate.hh
  Plugin.hh
  SystemPaths.hh
  UpdateInfo.hh
)

set (gtest_sources
  SystemPaths_TEST.cc
)

gz_build_tests(${gtest_sources})

set (common_headers "" CACHE INTERNAL "common headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(common_headers
    "Common Headers" "#include \"gazebo/common/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/common.hh.in ${CMAKE_CURRENT_BINARY_DIR}/common.hh )

gz_add_library(gazebo_common ${sources})

set_property(
  SOURCE SystemPaths.cc SystemPaths_TEST.cc ModelDatabase.cc
  PROPERTY COMPILE_DEFINITIONS
  GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}"
  GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH}"
  GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI}"
  OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH}"
)

target_link_libraries(gazebo_common
  gazebo_math
  ${Boost_LIBRARIES}
  ${CURL_LIBRARIES}
  ${libtar_libraries}
  ${SDF_LIBRARIES}
  ${IGNITION_COMMON_LIBRARIES}
  ${IGNITION_MATH_LIBRARIES}
  pthread
)

if (NOT APPLE)
  # rt is used for clock_gettime, which is not available on apple
  target_link_libraries(gazebo_common rt)
endif()

gz_install_library(gazebo_common)
gz_install_includes("common" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/common.hh)
