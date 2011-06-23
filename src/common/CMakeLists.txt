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
              AssimpLoader.cc
              Color.cc
              Diagnostics.cc
              Event.cc
              Events.cc
              SystemPaths.cc
              Exception.cc
              Console.cc
              Mesh.cc
              MeshLoader.cc
              MeshManager.cc
              OgreLoader.cc
              Param.cc
              Plugin.cc
              STLLoader.cc
              Time.cc
              Timer.cc
              XMLConfig.cc
              Image.cc
              Material.cc
)

set ( headers CommonTypes.hh
              Messages.hh
              AssimpLoader.hh
              Color.hh
              Diagnostics.hh
              Event.hh
              Events.hh
              SystemPaths.hh
              Exception.hh
              Console.hh
              Global.hh
              Mesh.hh
              MeshLoader.hh
              MeshManager.hh
              OgreLoader.hh
              Param.hh
              Plugin.hh
              SingletonT.hh
              STLLoader.hh
              Time.hh
              Timer.hh
              XMLConfig.hh
              Image.hh 
              Material.hh
              MouseEvent.hh
)

add_library(gazebo_common SHARED ${sources})

target_link_libraries( gazebo_common gazebo_msgs
                                     gazebo_math
                                     ${boost_libraries}
                                     ${XML_LIBRARIES}
                                     ${assimp_libraries}
                                     ${freeimage_library})

INSTALL_LIBRARY(gazebo_common)
INSTALL_INCLUDES("common" ${headers})
