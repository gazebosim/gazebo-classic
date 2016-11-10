add_subdirectory(audio)
add_subdirectory(dem)
add_subdirectory(fonts)
add_subdirectory(gui)
add_subdirectory(materials)
add_subdirectory(models)

# use glsl 150 shaders if ogre version >= 1.9
if ((NOT ${OGRE_VERSION} VERSION_LESS 1.9.0) AND NOT APPLE)
  add_subdirectory(rtshaderlib150)
else ()
  add_subdirectory(rtshaderlib)
endif()

add_subdirectory(skyx)
