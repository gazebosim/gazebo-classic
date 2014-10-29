link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

set (server_inc 
  SimpleMOOCPlugin.hh
  MOOCRestApi.hh
  )


set (server_src 
  SimpleMOOCPlugin.cc
  MOOCRestApi.cc
  )

set (client_inc
  MOOCLoginDialog.hh
  MOOCUIPlugin.hh
  MOOCUIWidget.hh
  )


set (client_src
  MOOCLoginDialog.cc
  MOOCUIPlugin.cc
  MOOCUIWidget.cc
  )

set (qt_headers
    MOOCUIWidget.hh
    MOOCLoginDialog.hh)

include ( ${QT_USE_FILE} )

add_definitions(${QT_DEFINITIONS})

QT4_WRAP_CPP(headers_MOC ${qt_headers})
#QT4_ADD_RESOURCES(resources_RCC ${resources})



include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  )



add_library(SimpleMOOCPlugin SHARED ${server_src} )
target_link_libraries(SimpleMOOCPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS SimpleMOOCPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${server_inc})

add_library(MOOCUIPlugin SHARED ${client_src} ${headers_MOC})
target_link_libraries(MOOCUIPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS MOOCUIPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${client_inc})
