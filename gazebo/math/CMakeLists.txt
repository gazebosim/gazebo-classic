include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  Angle.cc
  Box.cc
  Matrix3.cc
  Matrix4.cc
  Plane.cc
  Pose.cc
  Quaternion.cc
  Rand.cc
  Vector2d.cc
  Vector2i.cc
  Vector3.cc
  Vector4.cc
  Helpers.cc
  Spline.cc
  RotationSpline.cc
)

set (headers
  Angle.hh
  Box.hh
  Matrix3.hh
  Matrix4.hh
  Plane.hh
  Pose.hh
  Quaternion.hh
  Rand.hh
  Vector2d.hh
  Vector2i.hh
  Vector3.hh
  Vector4.hh
  MathTypes.hh
  Helpers.hh
  Spline.hh
  RotationSpline.hh
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Matrix3_TEST.cc
  Plane_TEST.cc
  Pose_TEST.cc
  Quaternion_TEST.cc
  Rand_TEST.cc
  Vector2d_TEST.cc
  Vector2i_TEST.cc
  Vector3_TEST.cc
  Vector4_TEST.cc
  Helpers_TEST.cc
  Spline_TEST.cc
  RotationSpline_TEST.cc
)
gz_build_tests(${gtest_sources})

set (math_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(math_headers
    "Math Headers" "#include \"math/${hdr}\"\n")
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gzmath.hh.in
               ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh )


gz_add_library(gazebo_math ${sources})
target_link_libraries(gazebo_math ${Boost_LIBRARIES})

gz_install_library(gazebo_math)
gz_install_includes("math" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gzmath.hh)
