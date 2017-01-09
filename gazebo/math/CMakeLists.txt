include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  Helpers.cc
  Pose.cc
  Quaternion.cc
  Vector3.cc
)

set (headers
  Helpers.hh
  MathTypes.hh
  Pose.hh
  Quaternion.hh
  Vector3.hh
)

set (gtest_sources
)
gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_math)

set (math_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(math_headers
    "Math Headers" "#include \"gazebo/math/${hdr}\"\n")
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gzmath.hh.in
               ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh )

add_definitions(-DBUILDING_DLL_GZ_MATH)

gz_add_library(gazebo_math ${sources})
target_link_libraries(gazebo_math ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

gz_install_library(gazebo_math)
gz_install_includes("math" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh)
