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

INSTALL(FILES ${files} DESTINATION ${MEDIA_ROOT_DESTINATION}/rtshaderlib)
