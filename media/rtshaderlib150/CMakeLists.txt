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

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/rtshaderlib)

