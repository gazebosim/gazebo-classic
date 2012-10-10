set (files 
  CloudsNormal.png
  Clouds.png
  CloudsTile.png
  CMakeLists.txt
  Noise.jpg
  SkyX_Clouds.fragment
  SkyX_Clouds.hlsl
  SkyX_Clouds.vertex
  SkyX_Ground.fragment
  SkyX_Ground.hlsl
  SkyX_Ground.vertex
  SkyX_Lightning.fragment
  SkyX_Lightning.hlsl
  SkyX_Lightning.vertex
  SkyX.material
  SkyX_Moon.fragment
  SkyX_MoonHalo.png
  SkyX_Moon.hlsl
  SkyX_Moon.png
  SkyX_Moon.vertex
  SkyX_Skydome.fragment
  SkyX_Skydome.hlsl
  SkyX_Skydome.vertex
  SkyX_Starfield.png
  SkyX_VolClouds.fragment
  SkyX_VolClouds.hlsl
  SkyX_VolClouds_Lightning.fragment
  SkyX_VolClouds_Lightning.hlsl
  SkyX_VolClouds_Lightning.vertex
  SkyX_VolClouds.vertex
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/skyx)
