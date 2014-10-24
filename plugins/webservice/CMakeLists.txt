link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

set (src
  SimpleMOOCPlugin.cc
  MOOCRestApi.cc
  )

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/../msgs
  )

add_library(SimpleMOOCPlugin SHARED ${src} )
target_link_libraries(SimpleMOOCPlugin ${GAZEBO_libraries} SimpleMsgs)
install (TARGETS SimpleMOOCPlugin DESTINATION ${SIMPLEGDK_PLUGIN_INSTALL_DIR})
