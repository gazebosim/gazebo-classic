include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  Angle.cc
  Box.cc
  Helpers.cc
  Kmeans.cc
  Matrix3.cc
  Matrix4.cc
  Plane.cc
  Pose.cc
  Quaternion.cc
  Rand.cc
  RotationSpline.cc
  SignalStats.cc
  Spline.cc
  Vector2d.cc
  Vector2i.cc
  Vector3.cc
  Vector3Stats.cc
  Vector4.cc
)

set (headers
  Angle.hh
  Box.hh
  Filter.hh
  Helpers.hh
  Kmeans.hh
  MathTypes.hh
  Matrix3.hh
  Matrix4.hh
  Plane.hh
  Pose.hh
  Quaternion.hh
  Rand.hh
  RotationSpline.hh
  SignalStats.hh
  Spline.hh
  Vector2d.hh
  Vector2i.hh
  Vector3.hh
  Vector3Stats.hh
  Vector4.hh
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Helpers_TEST.cc
  Kmeans_TEST.cc
  Matrix3_TEST.cc
  Plane_TEST.cc
  Pose_TEST.cc
  Quaternion_TEST.cc
  Rand_TEST.cc
  RotationSpline_TEST.cc
  SignalStats_TEST.cc
  Spline_TEST.cc
  Vector2d_TEST.cc
  Vector2i_TEST.cc
  Vector3_TEST.cc
  Vector3Stats_TEST.cc
  Vector4_TEST.cc
)
gz_build_tests(${gtest_sources})

set (math_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(math_headers
    "Math Headers" "#include \"gazebo/math/${hdr}\"\n")
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gzmath.hh.in
               ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh )

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_MATH")

gz_add_library(gazebo_math ${sources})
target_link_libraries(gazebo_math ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

gz_install_library(gazebo_math)
gz_install_includes("math" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh)
