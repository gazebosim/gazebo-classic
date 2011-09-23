include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set ( sources Angle.cc
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
)

set ( headers Angle.hh
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
)

set (math_headers "" CACHE INTERNAL "transport headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(math_headers "Math Headers" "#include \"math/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/gzmath.h.in ${CMAKE_CURRENT_BINARY_DIR}/gzmath.h )


add_library(gazebo_math SHARED ${sources})

target_link_libraries( gazebo_math ${boost_libraries} )

INSTALL_LIBRARY(gazebo_math)
INSTALL_INCLUDES("math" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gzmath.h)
