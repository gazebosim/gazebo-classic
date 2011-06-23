include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set ( sources Angle.cc
              Box.cc
              Matrix3.cc
              Matrix4.cc
              Plane.cc
              Pose3d.cc
              Quatern.cc
              Rand.cc
              Vector2d.cc
              Vector2i.cc
              Vector3.cc
              Vector4.cc
)

set ( headers Angle.hh
              Box.hh
              Matrix3.hh
              Matrix4.hh
              Plane.hh
              Pose3d.hh
              Quatern.hh
              Rand.hh
              Vector2d.hh
              Vector2i.hh
              Vector3.hh
              Vector4.hh
              MathTypes.hh
)

add_library(gazebo_math SHARED ${sources})

target_link_libraries( gazebo_math ${boost_libraries} )

INSTALL_LIBRARY(gazebo_math)
INSTALL_INCLUDES("math" ${headers})
