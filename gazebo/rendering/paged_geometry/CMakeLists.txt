include (${gazebo_cmake_dir}/GazeboUtils.cmake)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (PAGEDGEOMETRY_ALTERNATE_COORDSYSTEM TRUE CACHE BOOL "PAGEDGEOMETRY_ALTERNATE_COORDSYSTEM" FORCE)
configure_file (PagedGeometryConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/PagedGeometryConfig.h)

FILE(GLOB sources *.cpp *.h)

#gz_add_library(gazebo_paged_geometry ${sources})
add_library(gazebo_paged_geometry STATIC ${sources})

set_target_properties(gazebo_paged_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(gazebo_paged_geometry
  ${ogre_libraries}
)

#gz_install_library(gazebo_paged_geometry)
