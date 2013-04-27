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

  aluminium_metal.jpg  
  bark.jpg
  cement_cracked.jpg
  cinder_block.jpg
  grass_dry.jpg  
  metal_ripped.jpg   
  mud_soft_leaves.jpg
  red_brick.jpg
  rock_craggy.jpg
  rough_ground.jpg
  steel_grating.jpg
  tarmac.jpg         
  wall_white_structured.jpg
  wood_planks.jpg
  
  #TODO: REMOVE THIS FROM HERE AND FIND AN APPROPRIATE PLACE!
  2x4x1_box.dae
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/textures/)
