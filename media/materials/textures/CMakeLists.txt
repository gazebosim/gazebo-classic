set (files
  beigeWall.jpg
  ceiling_tiled.jpg
  clouds.jpg
  dirt_diffusespecular.png
  dirt_normal.png
  flat_normal.png
  fungus_diffusespecular.png
  fungus_normal.png
  granite2.jpg
  granite.jpg
  grass_diffusespecular.png
  grass.jpg
  grass_normal.png
  hardwood_floor.jpg
  heightmap_bowl.png
  paintedWall.jpg
  pioneerBody.jpg
  projection_filter.png
  random.png
  road1.jpg
  stereo_projection_pattern_high_res.png
  stereo_projection_pattern_high_res_red.png
  terrain_detail.jpg
  terrain.png
  white.bmp
  willowMap.png
  wood.jpg
  WoodPallet.png
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/textures/)
