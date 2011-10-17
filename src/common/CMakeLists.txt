include (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers

add_definitions(${tinyxml_cflags})

include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${tinyxml_include_dirs}
)

link_directories(  
 ${tinyxml_library_dirs}
)

set ( sources ColladaLoader.cc
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
              STLLoader.cc
              Time.cc
              Timer.cc
              Image.cc
              Material.cc
)

set ( headers CommonTypes.hh
              ColladaLoader.hh
              Color.hh
              Diagnostics.hh
              Event.hh
              Events.hh
              SystemPaths.hh
              Exception.hh
              Console.hh
              Mesh.hh
              MeshLoader.hh
              MeshManager.hh
              OgreLoader.hh
              Plugin.hh
              SingletonT.hh
              STLLoader.hh
              Time.hh
              Timer.hh
              Image.hh 
              Material.hh
              MouseEvent.hh
)

set (common_headers "" CACHE INTERNAL "common headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(common_headers "Common Headers" "#include \"common/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/common.h.in ${CMAKE_CURRENT_BINARY_DIR}/common.h )


add_library(gazebo_common SHARED ${sources})

target_link_libraries( gazebo_common gazebo_math
                                     ${libdl_library}
                                     ${tinyxml_libraries}
                                     ${boost_libraries}
                                     ${freeimage_library}
                                     ${tinyxml_libraries}
                     )

INSTALL_LIBRARY(gazebo_common)
INSTALL_INCLUDES("common" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/common.h)
