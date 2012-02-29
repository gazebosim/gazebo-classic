include (${gazebo_cmake_dir}/GazeboUtils.cmake)

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
              STLLoader.cc
              Time.cc
              Timer.cc
              Image.cc
              Material.cc
              KeyFrame.cc
              Animation.cc
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
              Plugin.hh
              SingletonT.hh
              STLLoader.hh
              Time.hh
              Timer.hh
              Image.hh 
              Material.hh
              MouseEvent.hh
              KeyFrame.hh
              Animation.hh
)

set (common_headers "" CACHE INTERNAL "common headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(common_headers "Common Headers" "#include \"common/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/common.h.in ${CMAKE_CURRENT_BINARY_DIR}/common.h )


add_library(gazebo_common SHARED ${sources})

target_link_libraries(gazebo_common gazebo_math
                                    ${libdl_library}
                                    ${tinyxml_libraries}
                                    ${boost_libraries}
                                    ${freeimage_library}
                                    ${tinyxml_libraries}
                                    ${general_libraries}
                     )

INSTALL_LIBRARY(gazebo_common)
INSTALL_INCLUDES("common" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/common.h)
