include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  MaterialSwitcher.cc
  SelectionBuffer.cc
  SelectionRenderListener.cc
)

set (headers
  MaterialSwitcher.hh
  SelectionBuffer.hh
  SelectionRenderListener.hh
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_SELECTION_BUFF")

gz_add_library(gazebo_selection_buffer ${sources})

target_link_libraries(gazebo_selection_buffer
  gazebo_common 
  ${ogre_libraries} 
  ${OPENGL_LIBRARIES} 
)
                                      
gz_install_library(gazebo_selection_buffer)
