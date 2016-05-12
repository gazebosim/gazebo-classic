set (server_inc
  RestApi.hh
  RestException.hh
  RestWebPlugin.hh
  )

set (server_src
  RestApi.cc
  RestWebPlugin.cc
  )

set (client_inc
  RestUiLoginDialog.hh
  RestUiPlugin.hh
  RestUiWidget.hh
  )

set (client_src
  RestUiLoginDialog.cc
  RestUiPlugin.cc
  RestUiWidget.cc
  )

set (qt_headers
    RestUiWidget.hh
    RestUiLoginDialog.hh
)

include ( ${QT_USE_FILE} )

add_definitions(${QT_DEFINITIONS})

QT4_WRAP_CPP(headers_MOC ${qt_headers})

add_library(RestWebPlugin MODULE ${server_src} )
target_link_libraries(RestWebPlugin libgazebo gazebo_msgs)
install (TARGETS RestWebPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${server_inc})

add_library(RestUiPlugin MODULE ${client_src} ${headers_MOC})
target_link_libraries(RestUiPlugin gazebo_gui gazebo_msgs)
install (TARGETS RestUiPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${client_inc})
