find_package(haptix-comm REQUIRED)

include_directories(
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
	#${HAPTIX-COMM_INCLUDE_DIRS}
)

# Tinyxml are used since the plugins are linking against gazebo_common
# which uses tinxyml. Not defining the link directory here will end up
# failures in clang.
link_directories(
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
  #${HAPTIX-COMM_LIBRARY_DIRS}
)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/
)
set(GAZEBO_TEXTURE_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/textures/
)

QT4_WRAP_CPP(HaptixGUIPlugin_MOC HaptixGUIPlugin.hh TaskButton.hh)
add_library(HaptixGUIPlugin SHARED HaptixGUIPlugin.cc TaskButton.cc
						${HaptixGUIPlugin_MOC})
						#${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxGrasp.pb.cc
						#${HAPTIX-COMM_INCLUDE_DIRS}/haptix/comm/msg/hxCommand.pb.cc)

set_target_properties(HaptixGUIPlugin PROPERTIES COMPILE_FLAGS "-std=c++11")
target_link_libraries(HaptixGUIPlugin libgazebo gazebo_sensors ${ogre_libraries}  ${IGNITION-TRANSPORT_LIBRARIES} haptix-comm ${QT_LIBRARIES})
install (TARGETS HaptixGUIPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})

set (images
  hand.svg
  grasp_1.jpg
  grasp_2.jpg
  grasp_3.jpg
  grasp_4.jpg
  grasp_5.jpg
  grasp_6.jpg
  grip_1.jpg
  grip_2.jpg
  grip_3.jpg
  grip_4.jpg
  pinch_1.jpg
  pinch_2.jpg
)

install (FILES ${images} DESTINATION ${GAZEBO_TEXTURE_INSTALL_DIR})
gz_install_includes("plugins" HaptixGUIPlugin.hh)

