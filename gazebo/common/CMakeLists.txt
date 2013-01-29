include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources 
  Animation.cc
  BVHLoader.cc
  ColladaLoader.cc
  Color.cc
  Common.cc
  Console.cc
  Diagnostics.cc
  Event.cc
  Events.cc
  Exception.cc
  Image.cc
  KeyFrame.cc
  LogPlay.cc
  LogRecord.cc
  Material.cc
  Mesh.cc
  MeshLoader.cc
  MeshManager.cc
  ModelDatabase.cc
  PID.cc
  SkeletonAnimation.cc
  Skeleton.cc
  STLLoader.cc
  SystemPaths.cc
  Time.cc
  Timer.cc
  Video.cc
)

set (headers 
  Animation.hh
  BVHLoader.hh
  ColladaLoader.hh
  Common.hh
  CommonTypes.hh
  Color.hh
  Console.hh
  Diagnostics.hh
  Event.hh
  Events.hh
  Exception.hh
  Image.hh 
  KeyFrame.hh
  LogPlay.hh
  LogRecord.hh
  Material.hh
  Mesh.hh
  MeshLoader.hh
  MeshManager.hh
  MouseEvent.hh
  PID.hh
  Plugin.hh
  SkeletonAnimation.hh
  Skeleton.hh
  SingletonT.hh
  STLLoader.hh
  SystemPaths.hh
  Time.hh
  Timer.hh
  UpdateInfo.hh
  Video.hh
  )

set (common_headers "" CACHE INTERNAL "common headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(common_headers
    "Common Headers" "#include \"common/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/common.hh.in ${CMAKE_CURRENT_BINARY_DIR}/common.hh )

gz_add_library(gazebo_common ${sources})
target_link_libraries(gazebo_common gazebo_math
                                    ${libdl_library}
                                    ${Boost_LIBRARIES}
                                    ${freeimage_library}
                                    ${tinyxml_libraries}
                                    ${libavcodec_LIBRARIES}
                                    ${libavformat_LIBRARIES}
                                    ${CURL_LIBRARIES}
                                    ${libswscale_LIBRARIES}
                                    ${libtar_libraries}
                                    rt
                                    )

gz_install_library(gazebo_common)
gz_install_includes("common" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/common.hh)
