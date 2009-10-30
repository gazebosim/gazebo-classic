INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

add_subdirectory(assimp)

APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                      ${gazeboserver_include_dirs_desc} 
                      ${CMAKE_CURRENT_SOURCE_DIR})
