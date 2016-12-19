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

INSTALL(FILES ${files} DESTINATION ${MEDIA_ROOT_DESTINATION}/gui/schemes/)
