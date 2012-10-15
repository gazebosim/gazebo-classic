set (files Animation.xsd
           CEGUIConfig.xsd
           GUILayout.xsd
           GUIScheme.xsd
           Demo8.scheme
           OgreTray.scheme
           TaharezLook.scheme
           TaharezLookWidgetAliases.scheme
           TaharezLookWidgets.scheme
           VanillaSkin.scheme
           WindowsLook.scheme
           WindowsLookWidgets.scheme
           GazeboSkin.scheme
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/schemes/)




