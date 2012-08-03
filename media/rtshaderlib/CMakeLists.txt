add_subdirectory(materials)

set (files 
FFPLib_Common.cg
FFPLib_Common.hlsl
FFPLib_Common.glsl
FFPLib_Fog.cg
FFPLib_Fog.glsl
FFPLib_Fog.hlsl
FFPLib_Lighting.cg
FFPLib_Lighting.glsl
FFPLib_Lighting.hlsl
FFPLib_Texturing.cg
FFPLib_Texturing.glsl
FFPLib_Texturing.hlsl
FFPLib_Transform.cg
FFPLib_Transform.glsl
FFPLib_Transform.hlsl
SampleLib_ReflectionMap.cg
SampleLib_ReflectionMap.glsl
SampleLib_ReflectionMap.hlsl
SGXLib_IntegratedPSSM.cg
SGXLib_IntegratedPSSM.glsl
SGXLib_IntegratedPSSM.hlsl
SGXLib_NormalMapLighting.cg
SGXLib_NormalMapLighting.glsl
SGXLib_NormalMapLighting.hlsl
SGXLib_PerPixelLighting.cg
SGXLib_PerPixelLighting.glsl
SGXLib_PerPixelLighting.hlsl
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/rtshaderlib)

