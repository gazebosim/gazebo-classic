include (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers
string (REPLACE " " ";" assimp_include_dirs_split "${assimp_include_dirs}")
string (REPLACE " " ";" assimp_library_dirs_split "${assimp_library_dirs}")

include_directories( 
  ${XML_INCLUDE_DIRS} 
  ${PROJECT_BINARY_DIR}/src 
  ${assimp_include_dirs_split}
)

link_directories(  
 ${assimp_library_dirs_split} 
)

set ( sources Messages.cc
              Angle.cc
              AssimpLoader.cc
              Box.cc
              Color.cc
              Diagnostics.cc
              Event.cc
              Events.cc
              GazeboConfig.cc
              GazeboError.cc
              GazeboMessage.cc
              Matrix3.cc
              Matrix4.cc
              Mesh.cc
              MeshLoader.cc
              MeshManager.cc
              OgreLoader.cc
              Param.cc
              Plane.cc
              Plugin.cc
              Pose3d.cc
              Quatern.cc
              Rand.cc
              STLLoader.cc
              Time.cc
              Timer.cc
              Vector2d.cc
              Vector2i.cc
              Vector3.cc
              Vector4.cc
              XMLConfig.cc
              Image.cc
              Material.cc
)

set ( headers Messages.hh
              Angle.hh
              AssimpLoader.hh
              Box.hh
              Color.hh
              Diagnostics.hh
              Event.hh
              Events.hh
              GazeboConfig.hh
              GazeboError.hh
              GazeboMessage.hh
              Global.hh
              Matrix3.hh
              Matrix4.hh
              Mesh.hh
              MeshLoader.hh
              MeshManager.hh
              OgreLoader.hh
              Param.hh
              Plane.hh
              Plugin.hh
              Pose3d.hh
              Quatern.hh
              Rand.hh
              SingletonT.hh
              STLLoader.hh
              Time.hh
              Timer.hh
              Vector2d.hh
              Vector2i.hh
              Vector3.hh
              Vector4.hh
              XMLConfig.hh
              Image.hh 
              Material.hh
              MouseEvent.hh
)

add_library(gazebo_common SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_common PROPERTIES LINK_FLAGS
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

#target_link_libraries( gazebo_common ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})
target_link_libraries( gazebo_common gazebo_msgs
                                     ${boost_libraries}
                                     ${XML_LIBRARIES}
                                     ${assimp_libraries}
                                     ${freeimage_library})

install ( TARGETS gazebo_common DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/common COMPONENT headers)
