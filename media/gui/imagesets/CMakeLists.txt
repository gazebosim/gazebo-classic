set (files Imageset.xsd
           DriveIcons.imageset
           DriveIcons.png
           FairChar.imageset
           FairChar.tga
           GPN-2000-001437.tga
           logo.png
           OgreTray.imageset
           OgreTrayImages.png
           ReadMe.txt
           TaharezLook.imageset
           TaharezLook.tga
           Vanilla.imageset
           vanilla.tga
           WindowsLook.imageset
           WindowsLook.tga
           gazebo.tga
           Gazebo.imageset
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/gui/imagesets/)

