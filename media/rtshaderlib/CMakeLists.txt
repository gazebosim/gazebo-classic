add_subdirectory(materials)

set (files
FFPLib_Common.glsl
FFPLib_Fog.glsl
FFPLib_Lighting.glsl
FFPLib_Texturing.glsl
FFPLib_Transform.glsl
SampleLib_ReflectionMap.glsl
SGXLib_IntegratedPSSM.glsl
SGXLib_NormalMapLighting.glsl
SGXLib_PerPixelLighting.glsl
)

list(APPEND install_files "")

# remove shader version string if ogre version >= 1.9
if (NOT ${OGRE_VERSION} VERSION_LESS 1.9.0)
  foreach (src ${files})
    file (READ ${src} contents)
    string (REGEX REPLACE ".version [0-9]+" "" contents "${contents}")
    set (output_file ${PROJECT_BINARY_DIR}/media/rtshaderlib/${src})
    file (WRITE ${output_file} "${contents}")
    list (APPEND install_files ${output_file})
  endforeach ()
else ()
 set (install_files ${files})
endif ()

install(FILES ${install_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/rtshaderlib)
